"""Camera capture and detection thread -- DRP-AI hardware accelerated only.

This module runs as a dedicated thread (or process on Core 2) and implements
the full detection pipeline:

  1. **Capture**: Acquire BGR frames from the camera. In stream mode, the C++
     binary owns the camera and writes frames to /dev/shm/v2n_camera. In pipe
     mode, Python opens the camera via OpenCV.

  2. **Detect**: Run YOLO inference via DRP-AI hardware accelerator. Stream
     mode reads detections from shared memory (or JSON fallback from C++
     stdout); pipe mode sends frames via stdin and reads JSON responses.

  3. **Filter**: C++ handles all spatial filtering (confidence threshold, NMS,
     bounding-box size/aspect ratio). Python does temporal tracking only via
     DetectionTracker.

  4. **Track**: Temporal consistency via DetectionTracker -- a detection must
     appear in MIN_HITS consecutive cycles before being accepted. Tracks are
     matched by spatial proximity (bbox center distance < MATCH_DIST pixels)
     and aged out after MAX_AGE cycles without a match.

  5. **Estimate distance**: Use calibrated vision-based distance estimation
     (DistanceEstimator) with reference box height and FOV from GUI settings.
     Calibration can be updated live via the Settings > Detect tab.

Priority: stream mode (zero-copy, C++ owns camera) > pipe mode (Python camera).
No ONNX or CPU fallback -- DRP-AI is required on V2N.
"""

import math
import time

import cv2

from target_nav.utils.log import get_logger

logger = get_logger('app.camera_worker')

# DRP-AI Detector (hardware-accelerated on V2N)
try:
    from target_nav.detectors.drp_binary_detector import (
        DrpBinaryDetector, DrpStreamDetector, find_drp_binary, find_drp_model
    )
    HAS_DRPAI = True
except ImportError:
    HAS_DRPAI = False
    logger.warning("DRP-AI detector not available")

# Distance estimator
try:
    from target_nav.utils import DistanceEstimator
except ImportError:
    DistanceEstimator = None

from target_nav.detectors.detection import Det
from target_nav.detectors.shm_reader import DetectionShmReader, CalibrationShmWriter
from target_nav.config import (
    TARGET_CLASS_NAME, TARGET_DISPLAY_NAME, TARGET_FILTER_CLASSES,
    DEFAULT_FRAME_W, DEFAULT_FRAME_H,
    DEFAULT_REF_BOX_HEIGHT, DEFAULT_REF_DISTANCE, DEFAULT_CAMERA_FOV,
    CAMERA_OPEN_RETRIES, CAMERA_MAX_READ_FAILURES, DETECTION_PIPE_MAX_MISSES,
)
from target_nav.app.detection_filters import DetectionTracker


def _target_cfg():
    """Read target profile constants (class_name, display_name, filter_classes)."""
    return TARGET_CLASS_NAME, TARGET_DISPLAY_NAME, TARGET_FILTER_CLASSES


def _detect_pipe(frame, detector, estimator):
    """Run DRP-AI pipe-mode detection on a single frame.

    Sends the frame to the C++ subprocess via stdin, reads JSON results,
    and converts them to Det objects with distance/angle estimates.

    Args:
        frame: BGR numpy array (HxWx3) to run inference on.
        detector: DrpBinaryDetector instance (pipe mode).
        estimator: DistanceEstimator for distance/angle computation,
            or None to skip estimation.

    Returns:
        Tuple of (list of Det objects, inference_time in seconds).
    """
    detections = []
    infer_time = 0.0
    frame_h = frame.shape[0] if frame is not None else DEFAULT_FRAME_H
    frame_w = frame.shape[1] if frame is not None else DEFAULT_FRAME_W
    try:
        result = detector.detect(frame)
        infer_time = result.inference_time if result.inference_time else 0.0
        if result.success:
            for det in result.detections:
                x1, y1, x2, y2 = det.bbox
                dist = 0.0
                ang = 0.0
                if estimator:
                    dist, ang = estimator.estimate(det)
                detections.append(Det(
                    class_name=det.class_name,
                    confidence=det.confidence,
                    bbox=det.bbox,
                    distance=dist,
                    angle=ang,
                    bbox_clipped=(x1 <= 5 or x2 >= frame_w - 5 or
                                  y1 <= 5 or y2 >= frame_h - 5),
                ))
    except Exception as e:
        logger.error("Detection error: %s", e)
    return detections, infer_time


def _make_estimator(shared_state=None):
    """Create a DistanceEstimator with calibration from shared state.

    Args:
        shared_state: SharedState instance to read calibration values
            (reference height, reference distance, camera FOV) from.
            Falls back to defaults (180px, 1.0m, 60deg) if None.

    Returns:
        Configured DistanceEstimator instance.
    """
    ref_height = DEFAULT_REF_BOX_HEIGHT
    ref_distance = DEFAULT_REF_DISTANCE
    fov = DEFAULT_CAMERA_FOV
    if shared_state is not None:
        ref_height, ref_distance = shared_state.detection.get_calibration()
        fov = shared_state.detection.get_camera_fov()
    return DistanceEstimator(
        reference_box_height=ref_height, reference_distance=ref_distance,
        frame_width=DEFAULT_FRAME_W, frame_height=DEFAULT_FRAME_H,
        horizontal_fov=fov)


_cal_writer = None


def _sync_calibration(estimator, shared_state):
    """Check if calibration changed in GUI and update estimator in-place.

    Also writes calibration to shared memory so C++ DRP-AI can read it.
    Returns True if calibration was updated.
    """
    global _cal_writer
    if shared_state.detection.is_calibration_dirty():
        ref_height, ref_distance = shared_state.detection.get_calibration()
        fov = shared_state.detection.get_camera_fov()
        estimator.reference_box_height = ref_height
        estimator.reference_distance = ref_distance
        estimator.horizontal_fov = fov
        estimator._fov_rad = math.radians(fov)
        estimator._focal_length = (estimator.frame_width / 2.0) / math.tan(estimator._fov_rad / 2.0)
        logger.info("Calibration updated: %spx at %sm, FOV=%s\u00b0", ref_height, ref_distance, fov)

        # Write calibration to shared memory for C++ DRP-AI
        if _cal_writer is None:
            _cal_writer = CalibrationShmWriter()
        _cal_writer.write(ref_height, ref_distance, fov)

        return True
    return False


def _run_stream_mode(shared_state, drp_binary, drp_model):
    """Run the camera pipeline in stream mode (C++ owns camera + inference).

    In stream mode, the C++ binary captures frames internally and writes them
    to /dev/shm/v2n_camera. Python reads frames via mmap (zero-copy) and
    detection JSON via stdout pipe. This is the fastest mode, avoiding the
    ~921KB per-frame pipe transfer of pipe mode.

    This function blocks until shared_state.running becomes False or an error
    occurs. It handles calibration syncing, detection filtering, bbox tracking,
    and publishing results to shared state.

    Args:
        shared_state: SharedState instance for reading settings and publishing results.
        drp_binary: Path to the yolo_detection C++ binary.
        drp_model: Path to the DRP-AI model directory.

    Returns:
        True if stream mode ran (even if it eventually errored out).
        False if stream mode initialization failed (caller should try pipe mode).
    """
    logger.info("Trying DRP-AI stream mode: %s", drp_binary)
    shared_state.detection.set_stream_mode(True)
    shared_state.detection.set_detector_mode("DRP-AI Stream")

    conf_th = shared_state.detection.get_confidence_threshold()
    stream = DrpStreamDetector(
        binary_path=drp_binary, model_dir=drp_model,
        confidence_threshold=conf_th)

    if not stream.initialize():
        logger.warning("Stream mode init failed, falling back")
        shared_state.detection.set_stream_mode(False)
        shared_state.detection.set_detector_mode("Initializing")
        stream.shutdown()
        return False

    logger.info("DRP-AI stream mode active (C++ owns camera)")

    target_class, display_name, filter_classes = _target_cfg()
    estimator = _make_estimator(shared_state)
    tracker = DetectionTracker()
    cached_detections = []

    # Shared memory detection reader (500x faster than JSON)
    det_reader = DetectionShmReader()
    use_shm_dets = False

    try:
        while shared_state.running:
            # Sync calibration from GUI settings
            _sync_calibration(estimator, shared_state)

            # Check if GUI requested DRP-AI restart (new config.ini params)
            if shared_state.detection.check_and_clear_drpai_restart():
                logger.info("DRP-AI restart requested — restarting subprocess")
                det_reader.close()
                stream.shutdown()
                conf_th = shared_state.detection.get_confidence_threshold()
                stream = DrpStreamDetector(
                    binary_path=drp_binary, model_dir=drp_model,
                    confidence_threshold=conf_th)
                if not stream.initialize():
                    logger.error("DRP-AI restart failed — aborting stream mode")
                    break
                det_reader = DetectionShmReader()
                use_shm_dets = False
                tracker.reset()
                cached_detections = []
                logger.info("DRP-AI restarted with updated config.ini")
                continue

            # Check for GO reset (new navigation session)
            if shared_state.detection.check_and_clear_bbox_reset():
                tracker.reset()

            # Read frame from shared memory (zero-copy)
            frame = stream.get_frame()
            if frame is None:
                time.sleep(0.02)  # 50Hz retry — saves CPU vs 200Hz busy-wait
                continue

            # Try shared memory detection reader first (binary struct, ~0.001ms)
            # C++ already applies: confidence threshold, NMS, size/aspect filters,
            # distance estimation with calibrated values.
            # Python only adds: temporal tracking (DetectionTracker).
            shm_result = det_reader.read() if det_reader._attached or det_reader.attach() else None

            # Read raw detections from C++ (SHM or JSON fallback).
            # C++ provides bbox + confidence only. Python computes distance
            # and angle using the DistanceEstimator with GUI calibration,
            # so calibration changes take effect immediately.
            raw_dets = []
            if shm_result is not None:
                shm_dets, infer_ms, _ = shm_result
                if not use_shm_dets:
                    logger.info("Switched to shared memory detection reader")
                    use_shm_dets = True
                for d in (shm_dets or []):
                    # Recompute distance/angle from bbox using Python calibration
                    if estimator and d.height > 0:
                        dist, angle = estimator.estimate(d)
                    else:
                        dist, angle = d.distance, d.angle
                    # Check all 4 edges for clipping (C++ only checks Y axis)
                    x1, y1, x2, y2 = d.bbox
                    clipped = (x1 <= 5 or x2 >= DEFAULT_FRAME_W - 5 or
                               y1 <= 5 or y2 >= DEFAULT_FRAME_H - 5)
                    raw_dets.append(Det(
                        class_name=d.class_name,
                        confidence=d.confidence,
                        bbox=d.bbox,
                        distance=dist,
                        angle=angle,
                        bbox_clipped=clipped,
                    ))
            else:
                json_dets, infer_ms = stream.get_detections()
                if json_dets:
                    conf_th = shared_state.detection.get_confidence_threshold()
                    for d in json_dets:
                        conf = d.get('confidence', 0.0)
                        if conf < conf_th:
                            continue
                        bbox = (int(d.get('x1', 0)), int(d.get('y1', 0)),
                                int(d.get('x2', 0)), int(d.get('y2', 0)))
                        clipped = (d.get('x1', 0) <= 5 or
                                   d.get('x2', 0) >= DEFAULT_FRAME_W - 5 or
                                   d.get('y1', 0) <= 5 or
                                   d.get('y2', 0) >= DEFAULT_FRAME_H - 5)
                        # Build Det then compute distance from bbox
                        det = Det(
                            class_name=d.get('class_name', target_class),
                            confidence=conf, bbox=bbox,
                            bbox_clipped=clipped,
                        )
                        if estimator and det.height > 0:
                            dist, angle = estimator.estimate(det)
                            det = Det(class_name=det.class_name,
                                      confidence=conf, bbox=bbox,
                                      distance=dist, angle=angle,
                                      bbox_clipped=clipped)
                        raw_dets.append(det)

            # Target shape filter — reject objects that don't match target
            # dimensions BEFORE tracking. This prevents false positives
            # from entering the tracker and being displayed.
            # Uses known target ratio from config: width/height.
            from target_nav.config import DEFAULT_TARGET
            TARGET_RATIO = DEFAULT_TARGET['width'] / DEFAULT_TARGET['height']
            RATIO_TOL = 0.6
            ratio_lo = TARGET_RATIO * (1.0 - RATIO_TOL)
            ratio_hi = TARGET_RATIO * (1.0 + RATIO_TOL)

            filtered_dets = []
            for d in raw_dets:
                w, h = d.width, d.height
                if w > 0 and h > 0 and not d.bbox_clipped:
                    ratio = w / h
                    if ratio < ratio_lo or ratio > ratio_hi:
                        continue  # doesn't match target shape
                elif d.bbox_clipped and w > 0 and d.distance > 0.1 and estimator:
                    # Clipped: check width against expected target width
                    exp_h = estimator.reference_box_height * (estimator.reference_distance / d.distance)
                    exp_w = exp_h * TARGET_RATIO
                    if exp_w > 0 and w > exp_w * 3.0:
                        continue  # too wide for the target
                filtered_dets.append(d)

            # Temporal tracker: persists detections through DRP-AI gaps.
            cached_detections = tracker.update(filtered_dets)
            fresh = len(cached_detections) > 0

            info = f"No {display_name} detected"
            if cached_detections:
                best = min(cached_detections, key=lambda d: d.distance)
                info = f"{display_name}: {best.distance:.2f}m, {math.degrees(best.angle):.1f}deg"

            # Pass frame + detections to shared state for GUI display.
            shared_state.detection.set_camera(
                frame, cached_detections, info, fresh_detection=fresh)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        logger.error("Stream error: %s", e)
    finally:
        det_reader.close()
        stream.shutdown()
        logger.info("Stream mode stopped")

    return True


def camera_thread(shared_state):
    """Entry point for the camera/detection thread.

    Runs as a dedicated thread (started by the GUI process). Tries stream mode
    first for best performance, then falls back to pipe mode. There is no CPU
    fallback -- DRP-AI hardware is required on V2N.

    Priority:
      1. DRP-AI stream mode (C++ owns camera -- fastest, zero-copy shm)
      2. DRP-AI pipe mode (Python sends frames to C++ via stdin)
      3. No fallback -- logs error and returns

    The function blocks until shared_state.running becomes False. On exit,
    it releases the camera, shuts down the detector subprocess, and cleans
    up the thread pool executor.

    Args:
        shared_state: SharedState instance for reading settings, publishing
            frames/detections, and checking the running flag.
    """
    from concurrent.futures import ThreadPoolExecutor

    cap = None
    detector = None
    executor = None
    logger.info("Starting (DRP-AI only)...")

    cv2.ocl.setUseOpenCL(False)
    target_class, display_name, filter_classes = _target_cfg()

    if not HAS_DRPAI:
        logger.error("DRP-AI module not available")
        shared_state.detection.set_detector_mode("DRP-AI Unavailable")
        shared_state.detection.set_camera(None, [], "DRP-AI not available")
        return

    try:
        estimator = None
        drp_binary = find_drp_binary()
        drp_model = find_drp_model()

        if not drp_binary or not drp_model:
            logger.error("DRP-AI binary or model not found")
            logger.error("  Binary searched: %s", drp_binary)
            logger.error("  Model searched:  %s", drp_model)
            shared_state.detection.set_detector_mode("DRP-AI Not Found")
            shared_state.detection.set_camera(None, [], "DRP-AI binary/model not found")
            return

        # Disable Renesas OCA (OpenCV Accelerator) to prevent hardware
        # contention with DRP-AI. OCA_Activate with an all-zero list
        # effectively disables hardware acceleration for OpenCV operations.
        # Without this, OpenCV image processing and DRP-AI inference can
        # fight over the same hardware resources, causing intermittent hangs.
        try:
            import ctypes
            lib = ctypes.CDLL('/usr/lib/libopencv_imgproc.so.4.9.0')
            func = lib['_Z12OCA_ActivatePm']
            func.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
            func.restype = ctypes.c_int
            oca_list = (ctypes.c_ulong * 16)(*([0] * 16))
            func(oca_list)
            logger.info("OCA disabled (DRP-AI conflict prevention)")
        except Exception:
            pass

        # --- Try stream mode first (C++ owns camera — fastest) ---
        if _run_stream_mode(shared_state, drp_binary, drp_model):
            return  # Stream mode handled everything

        # --- Fallback: pipe mode (Python camera + DRP-AI inference) ---
        logger.info("Stream mode unavailable, trying DRP-AI pipe mode")
        try:
            conf_th = shared_state.detection.get_confidence_threshold()
            detector = DrpBinaryDetector(
                binary_path=drp_binary, model_dir=drp_model,
                confidence_threshold=conf_th, target_class=target_class)
            if detector.initialize():
                estimator = _make_estimator(shared_state)
                shared_state.detection.set_detector_mode("DRP-AI Pipe")
                logger.info("DRP-AI pipe detector ready")
            else:
                logger.error("DRP-AI pipe init failed")
                shared_state.detection.set_detector_mode("DRP-AI Failed")
                shared_state.detection.set_camera(None, [], "DRP-AI init failed")
                return
        except Exception as e:
            logger.error("DRP-AI pipe failed: %s", e)
            shared_state.detection.set_detector_mode("DRP-AI Failed")
            shared_state.detection.set_camera(None, [], f"DRP-AI error: {e}")
            return

        # Open camera for pipe mode
        for attempt in range(CAMERA_OPEN_RETRIES):
            if not shared_state.running:
                return
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap.isOpened():
                logger.info("Camera opened")
                break
            logger.warning("Camera attempt %d/%d failed",
                           attempt + 1, CAMERA_OPEN_RETRIES)
            time.sleep(1)

        if not cap or not cap.isOpened():
            shared_state.detection.set_camera(None, [], "Camera not available")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        executor = ThreadPoolExecutor(max_workers=1)
        tracker = DetectionTracker()
        pending_future = None
        cached_detections = []
        consecutive_misses = 0
        max_misses = DETECTION_PIPE_MAX_MISSES
        last_detection_time = time.time()
        consecutive_read_failures = 0

        while shared_state.running:
            # Sync calibration from GUI settings
            _sync_calibration(estimator, shared_state)

            # Check if GUI requested DRP-AI restart (new config.ini params)
            if shared_state.detection.check_and_clear_drpai_restart():
                logger.info("DRP-AI restart requested — restarting pipe subprocess")
                if detector:
                    detector.shutdown()
                conf_th = shared_state.detection.get_confidence_threshold()
                detector = DrpBinaryDetector(
                    binary_path=drp_binary, model_dir=drp_model,
                    confidence_threshold=conf_th, target_class=target_class)
                if not detector.initialize():
                    logger.error("DRP-AI pipe restart failed — aborting")
                    break
                tracker.reset()
                cached_detections = []
                pending_future = None
                logger.info("DRP-AI pipe restarted with updated config.ini")
                continue

            # Check for GO reset (new navigation session)
            if shared_state.detection.check_and_clear_bbox_reset():
                tracker.reset()

            cap.grab()
            ret, frame = cap.read()
            if not ret:
                consecutive_read_failures += 1
                if consecutive_read_failures >= CAMERA_MAX_READ_FAILURES:
                    logger.warning("Camera lost, reconnecting...")
                    cap.release()
                    time.sleep(1.0)
                    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_W)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_H)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        logger.info("Camera reconnected")
                    else:
                        logger.error("Reconnect failed")
                    consecutive_read_failures = 0
                time.sleep(0.01)
                continue
            consecutive_read_failures = 0

            # Check async detection result
            fresh = False
            if pending_future is not None and pending_future.done():
                try:
                    new_detections, _ = pending_future.result()
                except Exception:
                    new_detections = []
                pending_future = None
                fresh = True
                last_detection_time = time.time()

                if new_detections:
                    # Temporal consistency: require multiple consecutive frames
                    cached_detections = tracker.update(new_detections)
                    consecutive_misses = 0
                else:
                    tracker.update([])  # age existing tracks
                    consecutive_misses += 1
                    if consecutive_misses >= max_misses:
                        cached_detections = []
                        tracker.reset()

            # Expire old detections
            detect_expiry = shared_state.detection.get_detect_expiry()
            detection_age = time.time() - last_detection_time
            if detection_age > detect_expiry:
                cached_detections = []

            # Submit new detection to DRP-AI
            if pending_future is None and detector:
                pending_future = executor.submit(
                    _detect_pipe, frame.copy(), detector, estimator)

            info = f"No {display_name} detected"
            if cached_detections and detection_age < detect_expiry:
                best = min(cached_detections, key=lambda d: d.distance)
                info = f"{display_name}: {best.distance:.2f}m, {math.degrees(best.angle):.1f}deg"

            # Keep frame as BGR — Cairo panel handles BGR→BGRA directly.
            shared_state.detection.set_camera(
                frame,
                cached_detections if detection_age < detect_expiry else [],
                info, fresh_detection=fresh)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        logger.error("Error: %s", e)
    finally:
        global _cal_writer
        logger.info("Cleaning up...")
        if executor:
            if pending_future is not None and not pending_future.done():
                pending_future.cancel()
            executor.shutdown(wait=True, cancel_futures=True)
        if cap:
            cap.release()
        if detector:
            detector.shutdown()
        if _cal_writer is not None:
            _cal_writer.close()
            _cal_writer = None
        logger.info("Stopped")
