"""Camera capture and detection thread -- DRP-AI hardware accelerated only.

This module runs as a dedicated thread (or process on Core 2) and implements
the full detection pipeline:

  1. **Capture**: Acquire BGR frames from the camera. In stream mode, the C++
     binary owns the camera and writes frames to /dev/shm/v2n_camera. In pipe
     mode, Python opens the camera via OpenCV.

  2. **Detect**: Run YOLO inference via DRP-AI hardware accelerator. Stream
     mode reads JSON from C++ stdout; pipe mode sends frames via stdin and
     reads JSON responses.

  3. **Filter**: Apply multi-layer rejection to raw detections:
     - Confidence threshold
     - Minimum / maximum bounding box size
     - Aspect ratio (target is taller than wide)
     - Distance-size consistency (reject if bbox height does not match
       the expected height at the estimated distance)

  4. **Track**: Temporal consistency via _DetectionTracker -- a detection must
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
import os
import time

import cv2
import numpy as np

# DRP-AI Detector (hardware-accelerated on V2N)
try:
    from bowling_target_nav.detectors.drp_binary_detector import (
        DrpBinaryDetector, DrpStreamDetector, find_drp_binary, find_drp_model
    )
    HAS_DRPAI = True
except ImportError:
    HAS_DRPAI = False
    print("[WARNING] DRP-AI detector not available")

# Distance estimator
try:
    from bowling_target_nav.utils import DistanceEstimator
except ImportError:
    DistanceEstimator = None

from bowling_target_nav.core.config import get_config


class _DetectionTracker:
    """Temporal consistency filter — rejects single-frame flicker detections.

    Tracks detections across inference cycles by spatial proximity.
    A detection must appear in MIN_HITS consecutive cycles before it's accepted.
    Detections that disappear for more than MAX_AGE cycles are dropped.
    """

    MIN_HITS = 2      # consecutive detections before accepted
    MAX_AGE = 3       # cycles without match before track is dropped
    MATCH_DIST = 80   # max pixel distance between bbox centers to match

    def __init__(self):
        self._tracks = []  # list of {cx, cy, bw, bh, hits, age, det_dict}

    def update(self, detections):
        """Update tracks with new detections. Returns only confirmed detections."""
        # Mark all tracks as unmatched
        for t in self._tracks:
            t['matched'] = False

        confirmed = []

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            # Find closest existing track
            best_track = None
            best_dist = self.MATCH_DIST
            for t in self._tracks:
                if t['matched']:
                    continue
                dx = cx - t['cx']
                dy = cy - t['cy']
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < best_dist:
                    best_dist = dist
                    best_track = t

            if best_track is not None:
                # Update existing track
                best_track['cx'] = cx
                best_track['cy'] = cy
                best_track['hits'] += 1
                best_track['age'] = 0
                best_track['matched'] = True
                best_track['det_dict'] = det
                if best_track['hits'] >= self.MIN_HITS:
                    confirmed.append(det)
            else:
                # New track
                self._tracks.append({
                    'cx': cx, 'cy': cy,
                    'hits': 1, 'age': 0,
                    'matched': True, 'det_dict': det,
                })

        # Age unmatched tracks, remove stale ones
        for t in self._tracks:
            if not t['matched']:
                t['age'] += 1
        self._tracks = [t for t in self._tracks if t['age'] <= self.MAX_AGE]

        return confirmed

    def reset(self):
        self._tracks.clear()


class _BboxSizeTracker:
    """Rolling median filter on accepted bbox heights.

    Rejects detections whose bbox height deviates too far from the median
    of recently accepted heights.  This catches false positives where a
    wall or other object is misclassified as the target at a wildly
    different apparent size.

    The first BOOTSTRAP_MIN detections are accepted freely to seed the
    window.  After that, a detection is rejected if its bbox height ratio
    to the median falls outside [1/factor, factor].

    Must be reset on each new GO command (new navigation session).
    """

    BOOTSTRAP_MIN = 3

    def __init__(self, window_size=5, rejection_factor=2.5):
        self._window_size = window_size
        self._factor = rejection_factor
        self._heights = []

    def filter(self, detections):
        """Filter detections by median bbox height consistency.

        Accepts all detections during bootstrap (< BOOTSTRAP_MIN samples).
        After bootstrap, rejects detections whose height deviates from
        the rolling median by more than the rejection factor.

        Args:
            detections: List of detection dicts with 'bbox' key.

        Returns:
            List of accepted detections (subset of input).
        """
        if len(self._heights) < self.BOOTSTRAP_MIN:
            # Bootstrap: accept all, record heights
            for d in detections:
                bh = d['bbox'][3] - d['bbox'][1]
                if bh > 0:
                    self._heights.append(bh)
                    if len(self._heights) > self._window_size:
                        self._heights.pop(0)
            return detections

        median_h = sorted(self._heights)[len(self._heights) // 2]
        if median_h <= 0:
            return detections

        inv_factor = 1.0 / self._factor
        accepted = []
        for d in detections:
            bh = d['bbox'][3] - d['bbox'][1]
            if bh <= 0:
                continue
            ratio = bh / median_h
            if inv_factor <= ratio <= self._factor:
                accepted.append(d)
                self._heights.append(bh)
                if len(self._heights) > self._window_size:
                    self._heights.pop(0)
        return accepted

    def reset(self):
        """Clear all recorded heights (call on each new GO command)."""
        self._heights.clear()


def _target_cfg():
    """Read target profile from config (class_name, display_name, filter_classes)."""
    cfg = get_config()
    t = cfg.detection.target
    return t.class_name, t.display_name, t.filter_classes


def _detect_pipe(frame, detector, estimator):
    """Run DRP-AI pipe detection on a frame. Returns (detections_list, inference_time)."""
    detections = []
    infer_time = 0.0
    frame_h = frame.shape[0] if frame is not None else 480
    try:
        result = detector.detect(frame)
        infer_time = result.inference_time if result.inference_time else 0.0
        if result.success:
            for det in result.detections:
                x1, y1, x2, y2 = det.bbox
                det_dict = {
                    'class_name': det.class_name,
                    'confidence': det.confidence,
                    'bbox': det.bbox,
                    'bbox_clipped': (y1 <= 5 or y2 >= frame_h - 5),
                }
                if estimator:
                    distance, angle = estimator.estimate(det)
                    det_dict['distance'] = distance
                    det_dict['angle'] = angle
                detections.append(det_dict)
    except Exception as e:
        print(f"[Detection] Error: {e}", flush=True)
    return detections, infer_time


def draw_rect_np(img, x1, y1, x2, y2, color, thickness=2):
    """Draw rectangle using numpy (avoids OpenCV DRP acceleration on V2N)."""
    h, w = img.shape[:2]
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w - 1, int(x2)), min(h - 1, int(y2))
    b, g, r = int(color[0]), int(color[1]), int(color[2])
    t = thickness
    img[y1:y1+t, x1:x2, :] = (b, g, r)
    img[y2-t:y2, x1:x2, :] = (b, g, r)
    img[y1:y2, x1:x1+t, :] = (b, g, r)
    img[y1:y2, x2-t:x2, :] = (b, g, r)


def _make_estimator(shared_state=None):
    """Create a DistanceEstimator, reading calibration from state if available."""
    cfg = get_config()
    ref_height = 180.0
    ref_distance = 1.0
    fov = 60.0
    if shared_state is not None:
        ref_height, ref_distance = shared_state.detection.get_calibration()
        fov = shared_state.detection.get_camera_fov()
    return DistanceEstimator(
        reference_box_height=ref_height, reference_distance=ref_distance,
        frame_width=cfg.camera.width, frame_height=cfg.camera.height,
        horizontal_fov=fov)


def _sync_calibration(estimator, shared_state):
    """Check if calibration changed in GUI and update estimator in-place.

    Returns True if calibration was updated.
    """
    if shared_state.detection.is_calibration_dirty():
        ref_height, ref_distance = shared_state.detection.get_calibration()
        fov = shared_state.detection.get_camera_fov()
        estimator.reference_box_height = ref_height
        estimator.reference_distance = ref_distance
        estimator.horizontal_fov = fov
        estimator._fov_rad = math.radians(fov)
        estimator._focal_length = (estimator.frame_width / 2.0) / math.tan(estimator._fov_rad / 2.0)
        print(f"[Camera Thread] Calibration updated: {ref_height}px at {ref_distance}m, FOV={fov}°", flush=True)
        return True
    return False


def _process_raw_detections(raw_dets, frame_h, estimator, confidence_threshold=0.50,
                            frame_w=640, target_class=None,
                            max_aspect=1.0, max_box_pct=0.6,
                            size_tolerance=2.0):
    """Convert raw JSON detection dicts to the format used by the GUI/navigator.

    Filtering layers:
      1. Confidence threshold
      2. Minimum box size (too small = noise)
      3. Maximum box size (max_box_pct of frame = false full-frame detection)
      4. Aspect ratio (bw/bh > max_aspect = reject)
      5. Distance-size consistency (reject if ratio outside 1/size_tolerance..size_tolerance)
    """
    from bowling_target_nav.detectors.base import Detection
    if target_class is None:
        target_class, _, _ = _target_cfg()
    detections = []
    for d in raw_dets:
        conf = d.get('confidence', 0.0)
        if conf < confidence_threshold:
            continue
        x1, y1 = int(d.get('x1', 0)), int(d.get('y1', 0))
        x2, y2 = int(d.get('x2', 0)), int(d.get('y2', 0))
        bw, bh = x2 - x1, y2 - y1

        # Layer 2: Minimum box size — reject tiny noise
        if bw < 10 or bh < 15:
            continue

        # Layer 3: Maximum box size — reject huge false detections
        if bw > frame_w * max_box_pct or bh > frame_h * max_box_pct:
            continue

        # Layer 4: Aspect ratio — target is taller than wide
        if bh > 0 and bw / bh > max_aspect:
            continue
        # Also reject extremely thin slivers (bh/bw > 8 is unrealistic)
        if bw > 0 and bh / bw > 8.0:
            continue

        det_dict = {
            'class_name': d.get('class_name', target_class),
            'confidence': conf,
            'bbox': (x1, y1, x2, y2),
            'bbox_clipped': (y1 <= 5 or y2 >= frame_h - 5),
        }
        # Always recalculate distance from bbox using Python estimator
        # (C++ stream JSON uses hardcoded ref_box_height=100 which is uncalibrated)
        if estimator:
            det_obj = Detection(
                class_name=det_dict['class_name'],
                class_id=d.get('class_id', 0),
                confidence=conf,
                bbox=det_dict['bbox'])
            distance, angle = estimator.estimate(det_obj)

            # Layer 5: Distance-size consistency check
            # At the estimated distance, compute what box height SHOULD be.
            # Reject if actual height deviates too much (false positive).
            if distance > 0.1 and distance < 20.0:
                expected_bh = estimator.reference_box_height * (
                    estimator.reference_distance / distance)
                ratio = bh / expected_bh if expected_bh > 0 else 0
                inv_tol = 1.0 / size_tolerance if size_tolerance > 0 else 0.5
                if ratio < inv_tol or ratio > size_tolerance:
                    continue  # box size inconsistent with estimated distance

            det_dict['distance'] = distance
            det_dict['angle'] = angle
        detections.append(det_dict)
    return detections


def _run_stream_mode(shared_state, drp_binary, drp_model):
    """Stream mode: C++ owns camera + inference. Python reads results via shm + pipe.

    Returns True if stream mode ran (even if it eventually failed).
    Returns False if stream mode is not supported (binary lacks --stream).
    """
    print(f"[Camera Thread] Trying DRP-AI stream mode: {drp_binary}", flush=True)
    shared_state.detection.set_stream_mode(True)
    shared_state.detection.set_detector_mode("DRP-AI Stream")

    conf_th = shared_state.detection.get_confidence_threshold()
    stream = DrpStreamDetector(
        binary_path=drp_binary, model_dir=drp_model,
        confidence_threshold=conf_th)

    if not stream.initialize():
        print("[Camera Thread] Stream mode init failed, falling back", flush=True)
        shared_state.detection.set_stream_mode(False)
        shared_state.detection.set_detector_mode("Initializing")
        stream.shutdown()
        return False

    print("[Camera Thread] DRP-AI stream mode active (C++ owns camera)", flush=True)

    target_class, display_name, filter_classes = _target_cfg()
    estimator = _make_estimator(shared_state)
    tracker = _DetectionTracker()
    bbox_tracker = _BboxSizeTracker()
    cached_detections = []
    last_det_time = time.time()

    try:
        while shared_state.running:
            # Sync calibration from GUI settings
            _sync_calibration(estimator, shared_state)

            # Check for GO reset (new navigation session)
            if shared_state.detection.check_and_clear_bbox_reset():
                bbox_tracker.reset()
                tracker.reset()

            # Read frame from shared memory (zero-copy)
            frame = stream.get_frame()
            if frame is None:
                time.sleep(0.005)
                continue

            # Read latest detections from C++ stdout
            raw_dets, infer_ms = stream.get_detections()

            fresh = False
            if raw_dets:
                from bowling_target_nav.detectors.base import Detection
                conf_th = shared_state.detection.get_confidence_threshold()
                max_aspect = shared_state.detection.get_filter_max_aspect()
                max_box_pct = shared_state.detection.get_filter_max_box_pct()
                cam_cfg = get_config().camera
                frame_h = cam_cfg.height
                frame_w = cam_cfg.width

                filtered = []
                for d in raw_dets:
                    # Layer 1: Confidence
                    conf = d.get('confidence', 0.0)
                    if conf < conf_th:
                        continue

                    x1, y1 = int(d.get('x1', 0)), int(d.get('y1', 0))
                    x2, y2 = int(d.get('x2', 0)), int(d.get('y2', 0))
                    bw, bh = x2 - x1, y2 - y1

                    # Layer 2: Min box size (reject noise)
                    if bw < 10 or bh < 15:
                        continue

                    # Layer 3: Max box size (reject full-frame false positives)
                    if bw > frame_w * max_box_pct or bh > frame_h * max_box_pct:
                        continue

                    # Layer 4: Aspect ratio (bottle is taller than wide)
                    if bh > 0 and bw / bh > max_aspect:
                        continue

                    det_dict = {
                        'class_name': d.get('class_name', target_class),
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2),
                        'bbox_clipped': (y1 <= 5 or y2 >= frame_h - 5),
                        'distance': 1.0,
                        'angle': 0.0,
                    }
                    if estimator and bh > 0:
                        det_obj = Detection(
                            class_name=det_dict['class_name'],
                            class_id=d.get('class_id', 0),
                            confidence=conf,
                            bbox=det_dict['bbox'])
                        det_dict['distance'], det_dict['angle'] = estimator.estimate(det_obj)
                    filtered.append(det_dict)

                # Bbox size tracker (gentle — rejects wildly wrong sizes)
                if filtered:
                    bbox_tracker._window_size = shared_state.detection.get_filter_bbox_window()
                    bbox_tracker._factor = shared_state.detection.get_filter_bbox_factor()
                    filtered = bbox_tracker.filter(filtered)

                cached_detections = filtered
                last_det_time = time.time()
                fresh = True

            # Expire old detections
            detect_expiry = shared_state.detection.get_detect_expiry()
            det_age = time.time() - last_det_time
            if det_age > detect_expiry:
                cached_detections = []

            info = f"No {display_name} detected"

            if cached_detections and det_age < detect_expiry:
                best = min(cached_detections, key=lambda d: d.get('distance', 999))
                info = f"{display_name}: {best['distance']:.2f}m, {math.degrees(best['angle']):.1f}deg"

            # In stream mode, GUI reads frames directly from C++ shm
            # (/dev/shm/v2n_camera) — no need to copy 921KB through IPC.
            # Only send detections (tiny).
            shared_state.detection.set_camera(
                None,
                cached_detections if det_age < detect_expiry else [],
                info, fresh_detection=fresh)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        print(f"[Camera Thread] Stream error: {e}", flush=True)
    finally:
        stream.shutdown()
        print("[Camera Thread] Stream mode stopped", flush=True)

    return True


def camera_thread(shared_state):
    """Camera capture using DRP-AI hardware acceleration only.

    Priority:
      1. DRP-AI stream mode (C++ owns camera — fastest, zero-copy shm)
      2. DRP-AI pipe mode (Python sends frames to C++)
      3. No fallback — DRP-AI is required
    """
    from concurrent.futures import ThreadPoolExecutor

    cap = None
    detector = None
    executor = None
    print("[Camera Thread] Starting (DRP-AI only)...", flush=True)

    cv2.ocl.setUseOpenCL(False)
    target_class, display_name, filter_classes = _target_cfg()

    if not HAS_DRPAI:
        print("[Camera Thread] ERROR: DRP-AI module not available", flush=True)
        shared_state.detection.set_detector_mode("DRP-AI Unavailable")
        shared_state.detection.set_camera(None, [], "DRP-AI not available")
        return

    try:
        estimator = None
        drp_binary = find_drp_binary()
        drp_model = find_drp_model()

        if not drp_binary or not drp_model:
            print(f"[Camera Thread] ERROR: DRP-AI binary or model not found", flush=True)
            print(f"[Camera Thread]   Binary searched: {drp_binary}", flush=True)
            print(f"[Camera Thread]   Model searched:  {drp_model}", flush=True)
            shared_state.detection.set_detector_mode("DRP-AI Not Found")
            shared_state.detection.set_camera(None, [], "DRP-AI binary/model not found")
            return

        # Disable OpenCV OCA to prevent DRP-AI contention
        try:
            import ctypes
            lib = ctypes.CDLL('/usr/lib/libopencv_imgproc.so.4.9.0')
            func = lib['_Z12OCA_ActivatePm']
            func.argtypes = [ctypes.POINTER(ctypes.c_ulong)]
            func.restype = ctypes.c_int
            oca_list = (ctypes.c_ulong * 16)(*([0] * 16))
            func(oca_list)
            print("[Camera Thread] OCA disabled (DRP-AI conflict prevention)", flush=True)
        except Exception:
            pass

        # --- Try stream mode first (C++ owns camera — fastest) ---
        if _run_stream_mode(shared_state, drp_binary, drp_model):
            return  # Stream mode handled everything

        # --- Fallback: pipe mode (Python camera + DRP-AI inference) ---
        print(f"[Camera Thread] Stream mode unavailable, trying DRP-AI pipe mode", flush=True)
        try:
            conf_th = shared_state.detection.get_confidence_threshold()
            detector = DrpBinaryDetector(
                binary_path=drp_binary, model_dir=drp_model,
                confidence_threshold=conf_th, target_class=target_class)
            if detector.initialize():
                estimator = _make_estimator(shared_state)
                shared_state.detection.set_detector_mode("DRP-AI Pipe")
                print("[Camera Thread] DRP-AI pipe detector ready", flush=True)
            else:
                print("[Camera Thread] ERROR: DRP-AI pipe init failed", flush=True)
                shared_state.detection.set_detector_mode("DRP-AI Failed")
                shared_state.detection.set_camera(None, [], "DRP-AI init failed")
                return
        except Exception as e:
            print(f"[Camera Thread] ERROR: DRP-AI pipe failed: {e}", flush=True)
            shared_state.detection.set_detector_mode("DRP-AI Failed")
            shared_state.detection.set_camera(None, [], f"DRP-AI error: {e}")
            return

        # Open camera for pipe mode
        for attempt in range(3):
            if not shared_state.running:
                return
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if cap.isOpened():
                print("[Camera Thread] Camera opened", flush=True)
                break
            print(f"[Camera Thread] Camera attempt {attempt+1}/3 failed", flush=True)
            time.sleep(1)

        if not cap or not cap.isOpened():
            shared_state.detection.set_camera(None, [], "Camera not available")
            return

        cam_cfg = get_config().camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        executor = ThreadPoolExecutor(max_workers=1)
        tracker = _DetectionTracker()
        bbox_tracker = _BboxSizeTracker()
        pending_future = None
        cached_detections = []
        consecutive_misses = 0
        max_misses = 5
        frame_count = 0
        last_fps_time = time.time()
        last_detection_time = time.time()
        consecutive_read_failures = 0

        while shared_state.running:
            # Sync calibration from GUI settings
            _sync_calibration(estimator, shared_state)

            # Check for GO reset (new navigation session)
            if shared_state.detection.check_and_clear_bbox_reset():
                bbox_tracker.reset()
                tracker.reset()

            cap.grab()
            ret, frame = cap.read()
            if not ret:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 30:
                    print("[Camera Thread] Camera lost, reconnecting...", flush=True)
                    cap.release()
                    time.sleep(1.0)
                    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg.height)
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        print("[Camera Thread] Camera reconnected", flush=True)
                    else:
                        print("[Camera Thread] Reconnect failed", flush=True)
                    consecutive_read_failures = 0
                time.sleep(0.01)
                continue
            consecutive_read_failures = 0
            frame_count += 1

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
                    # Sync filter settings from GUI
                    bbox_tracker._window_size = shared_state.detection.get_filter_bbox_window()
                    bbox_tracker._factor = shared_state.detection.get_filter_bbox_factor()
                    # Layer 6: Rolling median bbox height consistency
                    new_detections = bbox_tracker.filter(new_detections)
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

            # Convert BGR→RGB in-place (no copy needed, frame won't be reused)
            rgb_frame = frame[:, :, ::-1]

            info = f"No {display_name} detected"
            if cached_detections and detection_age < detect_expiry:
                best = min(cached_detections, key=lambda d: d.get('distance', 999))
                info = f"{display_name}: {best['distance']:.2f}m, {math.degrees(best['angle']):.1f}deg"

            shared_state.detection.set_camera(
                rgb_frame,
                cached_detections if detection_age < detect_expiry else [],
                info, fresh_detection=fresh)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        print(f"[Camera Thread] ERROR: {e}", flush=True)
    finally:
        print("[Camera Thread] Cleaning up...", flush=True)
        if executor:
            if pending_future is not None and not pending_future.done():
                pending_future.cancel()
            executor.shutdown(wait=True, cancel_futures=True)
        if cap:
            cap.release()
        if detector:
            detector.shutdown()
        print("[Camera Thread] Stopped", flush=True)
