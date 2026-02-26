"""Camera capture thread with DRP-AI stream / pipe / ONNX detection."""

import math
import os
import time
from concurrent.futures import ThreadPoolExecutor

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

# YOLO Detector (ONNX Runtime CPU fallback)
try:
    from bowling_target_nav.detectors import YoloOnnxDetector
    from bowling_target_nav.utils import DistanceEstimator
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False
    print("[WARNING] YOLO detector not available")

TARGET_CLASS = 'bowling-pins'
FILTER_CLASSES = ['bowling-pins']


def find_model_path():
    """Find YOLO model in standard locations."""
    model_names = ['bowling_yolov8m_int8.onnx', 'bowling_yolov8m.onnx', 'bowling_yolov5.onnx']
    search_dirs = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'models'),
        os.path.expanduser('~/ros2_ws/install/bowling_target_nav/share/bowling_target_nav/models'),
        os.path.expanduser('~/ros2_ws/src/bowling_target_nav/models'),
        os.path.expanduser('~/ros2_ws/build/bowling_target_nav/models'),
        '/opt/ros/humble/share/bowling_target_nav/models',
        '/tmp',
    ]
    for name in model_names:
        for d in search_dirs:
            p = os.path.abspath(os.path.join(d, name))
            if os.path.exists(p):
                return p
    return None


def detect_objects(frame, detector, estimator):
    """Run YOLO detection on a frame. Returns (detections_list, inference_time)."""
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


def _make_estimator():
    """Create a DistanceEstimator with default calibration."""
    return DistanceEstimator(
        reference_box_height=100.0, reference_distance=1.0,
        frame_width=640, frame_height=480, horizontal_fov=60.0)


def _process_raw_detections(raw_dets, frame_h, estimator, confidence_threshold=0.35):
    """Convert raw JSON detection dicts to the format used by the GUI/navigator."""
    from bowling_target_nav.detectors.base import Detection
    detections = []
    for d in raw_dets:
        conf = d.get('confidence', 0.0)
        if conf < confidence_threshold:
            continue
        x1, y1 = int(d.get('x1', 0)), int(d.get('y1', 0))
        x2, y2 = int(d.get('x2', 0)), int(d.get('y2', 0))
        det_dict = {
            'class_name': d.get('class_name', 'bowling-pins'),
            'confidence': conf,
            'bbox': (x1, y1, x2, y2),
            'bbox_clipped': (y1 <= 5 or y2 >= frame_h - 5),
        }
        # Use distance/angle from C++ JSON if available (stream mode)
        if 'distance' in d and 'angle' in d:
            det_dict['distance'] = d['distance']
            det_dict['angle'] = d['angle']
        elif estimator:
            det_obj = Detection(
                class_name=det_dict['class_name'],
                class_id=d.get('class_id', 0),
                confidence=conf,
                bbox=det_dict['bbox'])
            distance, angle = estimator.estimate(det_obj)
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

    stream = DrpStreamDetector(
        binary_path=drp_binary, model_dir=drp_model,
        confidence_threshold=0.35)

    if not stream.initialize():
        print("[Camera Thread] Stream mode init failed, falling back", flush=True)
        shared_state.detection.set_stream_mode(False)
        shared_state.detection.set_detector_mode("Initializing")
        stream.shutdown()
        return False

    print("[Camera Thread] DRP-AI stream mode active (C++ owns camera)", flush=True)

    estimator = _make_estimator()
    cached_detections = []
    last_det_time = time.time()

    try:
        while shared_state.running:
            # Read frame from shared memory (zero-copy)
            frame = stream.get_frame()
            if frame is None:
                time.sleep(0.005)
                continue

            # Read latest detections from C++ stdout
            raw_dets, infer_ms = stream.get_detections()

            fresh = False
            if raw_dets:
                cached_detections = _process_raw_detections(
                    raw_dets, frame.shape[0], estimator)
                last_det_time = time.time()
                fresh = True

            # Expire old detections
            detect_expiry = shared_state.detection.get_detect_expiry()
            det_age = time.time() - last_det_time
            if det_age > detect_expiry:
                cached_detections = []

            # Frame already has overlays drawn by C++ — just convert BGR→RGB
            info = "No pins detected"

            if cached_detections and det_age < detect_expiry:
                best = min(cached_detections, key=lambda d: d.get('distance', 999))
                info = f"Pins: {best['distance']:.2f}m, {math.degrees(best['angle']):.1f}deg"

            rgb_frame = frame[:, :, ::-1].copy()
            shared_state.detection.set_camera(
                rgb_frame,
                cached_detections if det_age < detect_expiry else [],
                info, fresh_detection=fresh)

            time.sleep(0.001)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        print(f"[Camera Thread] Stream error: {e}", flush=True)
    finally:
        stream.shutdown()
        print("[Camera Thread] Stream mode stopped", flush=True)

    return True


def camera_thread(shared_state):
    """Camera capture with async YOLO detection.

    Priority:
      1. DRP-AI stream mode (C++ owns camera — fastest)
      2. DRP-AI pipe mode (Python sends frames to C++)
      3. ONNX Runtime on CPU (fallback)
    """
    cap = None
    detector = None
    executor = None
    print("[Camera Thread] Starting...", flush=True)

    cv2.ocl.setUseOpenCL(False)

    try:
        estimator = None
        use_drpai = False

        # Try DRP-AI hardware accelerator first (V2N only)
        if HAS_DRPAI:
            drp_binary = find_drp_binary()
            drp_model = find_drp_model()
            if drp_binary and drp_model:
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

                # Try stream mode first (C++ owns camera — fastest)
                if _run_stream_mode(shared_state, drp_binary, drp_model):
                    return  # Stream mode handled everything

                # Fallback: pipe mode
                print(f"[Camera Thread] Loading DRP-AI pipe detector: {drp_binary}", flush=True)
                try:
                    detector = DrpBinaryDetector(
                        binary_path=drp_binary, model_dir=drp_model,
                        confidence_threshold=0.35, target_class=TARGET_CLASS)
                    if detector.initialize():
                        use_drpai = True
                        estimator = _make_estimator()
                        shared_state.detection.set_detector_mode("DRP-AI Pipe")
                        print("[Camera Thread] DRP-AI pipe detector ready", flush=True)
                    else:
                        detector = None
                except Exception as e:
                    print(f"[Camera Thread] DRP-AI pipe failed: {e}", flush=True)
                    detector = None

        # Fallback: ONNX Runtime on CPU
        if detector is None:
            model_path = find_model_path()
            if model_path and HAS_YOLO:
                try:
                    detector = YoloOnnxDetector(
                        model_path=model_path, confidence_threshold=0.35,
                        nms_threshold=0.45, target_class=TARGET_CLASS,
                        filter_classes=FILTER_CLASSES)
                    if detector.initialize():
                        estimator = _make_estimator()
                        shared_state.detection.set_detector_mode("ONNX CPU")
                        print(f"[Camera Thread] ONNX detector ready (CPU)", flush=True)
                    else:
                        detector = None
                except Exception as e:
                    print(f"[Camera Thread] ONNX failed: {e}", flush=True)
                    detector = None

        if detector is None:
            shared_state.detection.set_detector_mode("None")

        # Open camera (pipe/ONNX modes need Python camera capture)
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

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        executor = ThreadPoolExecutor(max_workers=1)
        pending_future = None
        cached_detections = []
        last_submit_time = 0.0
        consecutive_misses = 0
        max_misses = 5
        frame_count = 0
        last_fps_time = time.time()
        last_detection_time = time.time()
        fps = 0.0
        consecutive_read_failures = 0

        while shared_state.running:
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                consecutive_read_failures += 1
                if consecutive_read_failures >= 30:
                    # Camera disconnected - attempt reconnect
                    print("[Camera Thread] Camera lost, reconnecting...", flush=True)
                    cap.release()
                    time.sleep(1.0)
                    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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
                    cached_detections = new_detections
                    consecutive_misses = 0
                else:
                    consecutive_misses += 1
                    if consecutive_misses >= max_misses:
                        cached_detections = []

            # Expire old detections
            detect_expiry = shared_state.detection.get_detect_expiry()
            detection_age = time.time() - last_detection_time
            if detection_age > detect_expiry:
                cached_detections = []

            # Submit new detection
            now = time.time()
            if pending_future is None and detector:
                pending_future = executor.submit(
                    detect_objects, frame.copy(), detector, estimator)
                last_submit_time = now

            # Draw cached detections
            display_frame = frame.copy()
            info = "No pins detected"

            if cached_detections and detection_age < detect_expiry:
                box_color = (0, 255, 0) if detection_age < 0.3 else (0, 200, 200)
                for det in cached_detections:
                    x1, y1, x2, y2 = det['bbox']
                    if use_drpai:
                        draw_rect_np(display_frame, x1, y1, x2, y2, box_color, 2)
                    else:
                        conf = det['confidence']
                        cv2.rectangle(display_frame, (x1, y1), (x2, y2), box_color, 2)
                        cv2.putText(display_frame, f"Pin {conf:.2f}", (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                        if 'distance' in det:
                            cv2.putText(display_frame, f"{det['distance']:.2f}m",
                                        (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                best = min(cached_detections, key=lambda d: d.get('distance', 999))
                info = f"Pins: {best['distance']:.2f}m, {math.degrees(best['angle']):.1f}deg"

            # FPS counter
            now = time.time()
            if now - last_fps_time >= 1.0:
                fps = frame_count / (now - last_fps_time)
                frame_count = 0
                last_fps_time = now

            if not use_drpai:
                cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            rgb_frame = display_frame[:, :, ::-1].copy()
            shared_state.detection.set_camera(
                rgb_frame,
                cached_detections if detection_age < detect_expiry else [],
                info, fresh_detection=fresh)

            time.sleep(0.001)

    except Exception as e:
        shared_state.add_error('camera_thread', str(e))
        print(f"[Camera Thread] ERROR: {e}", flush=True)
    finally:
        print("[Camera Thread] Cleaning up...", flush=True)
        if executor:
            # Cancel pending future before shutdown to avoid blocking on hung detection
            if pending_future is not None and not pending_future.done():
                pending_future.cancel()
            executor.shutdown(wait=True, cancel_futures=True)
        if cap:
            cap.release()
        if detector:
            detector.shutdown()
        print("[Camera Thread] Stopped", flush=True)
