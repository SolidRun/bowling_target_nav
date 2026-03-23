"""Temporal detection tracking with persistence through detection gaps.

The DRP-AI model at low confidence flickers — detecting the target in
frame 1, missing frame 2-3, detecting again in frame 4. A frame-strict
tracker would drop output on every miss, causing the vision distance
display to blank and the navigator to lose the target.

This tracker keeps outputting the last known detection for a configurable
time after the last match, giving the Kalman filter in the nav process
a continuous stream to work with.

Data flow:
  C++ DRP-AI binary -> shm / JSON -> [Det] -> DetectionTracker
  -> persistent [Det] -> shared_state / navigator
"""

import math
import time

from target_nav.config import DEFAULT_TRACKER_FILTER
from target_nav.detectors.detection import Det


class DetectionTracker:
    """Persistent spatial tracker — outputs detections continuously even
    through DRP-AI detection gaps.

    When the model detects a target, the track is created and outputs
    immediately. When the model misses the next few frames, the track
    continues outputting its last known position (with aging confidence)
    for up to PERSIST_TIME seconds. This gives downstream consumers
    (nav_controller, camera_panel) a continuous detection stream.

    False positive rejection is handled downstream by TargetTracker
    (shape filters, size gate, LiDAR matching).
    """

    def __init__(self, min_hits=None, max_age=None,
                 match_dist=None, bbox_ema=None, persist_time=None):
        cfg = DEFAULT_TRACKER_FILTER
        self.MIN_HITS = min_hits if min_hits is not None else cfg['min_hits']
        self.MAX_AGE = max_age if max_age is not None else cfg['max_age']
        self.MATCH_DIST = match_dist if match_dist is not None else cfg['match_dist_px']
        self.BBOX_EMA = bbox_ema if bbox_ema is not None else cfg['bbox_ema_alpha']
        self.PERSIST_TIME = persist_time if persist_time is not None else cfg.get('persist_time', 10.0)
        self._tracks = []

    def update(self, detections):
        """Match detections to tracks and return all active tracks.

        Returns tracks that were either:
        - Matched this frame (fresh detection, full confidence)
        - Not matched but seen recently (within PERSIST_TIME, decaying confidence)

        This ensures continuous output even when DRP-AI flickers.

        Args:
            detections: List of Det objects from this frame.

        Returns:
            List of Det objects for all active tracks.
        """
        now = time.time()

        for t in self._tracks:
            t['matched'] = False

        # Match each detection to closest existing track
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = x2 - x1
            bh = y2 - y1

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
                a = self.BBOX_EMA
                best_track['cx'] = a * cx + (1 - a) * best_track['cx']
                best_track['cy'] = a * cy + (1 - a) * best_track['cy']
                best_track['bw'] = a * bw + (1 - a) * best_track['bw']
                best_track['bh'] = a * bh + (1 - a) * best_track['bh']
                best_track['hits'] += 1
                best_track['age'] = 0
                best_track['matched'] = True
                best_track['raw_det'] = det
                best_track['last_seen'] = now
                best_track['confidence'] = det.confidence
            else:
                self._tracks.append({
                    'cx': cx, 'cy': cy, 'bw': bw, 'bh': bh,
                    'hits': 1, 'age': 0,
                    'matched': True, 'raw_det': det,
                    'last_seen': now,
                    'confidence': det.confidence,
                })

        # Build output: ALL tracks seen within PERSIST_TIME.
        # Only MATCHED tracks get fresh distance/angle.
        # COASTING tracks output smoothed bbox + last distance/angle
        # with decaying confidence so the Kalman filter knows to coast.
        output = []
        for t in self._tracks:
            raw = t.get('raw_det')
            if raw is None:
                continue

            time_since = now - t['last_seen']
            if time_since > self.PERSIST_TIME:
                continue

            hw = t['bw'] / 2.0
            hh = t['bh'] / 2.0
            smoothed_bbox = (
                int(t['cx'] - hw), int(t['cy'] - hh),
                int(t['cx'] + hw), int(t['cy'] + hh))

            if t['matched']:
                # Fresh detection — use real distance/angle
                conf = t['confidence']
                dist = raw.distance
                angle = raw.angle
            else:
                # Coasting — decay confidence so Kalman knows it's stale.
                # Keep last distance/angle (Kalman will predict from these).
                conf = t['confidence'] * max(0.1, 1.0 - time_since / self.PERSIST_TIME)
                dist = raw.distance
                angle = raw.angle

            output.append(Det(
                class_name=raw.class_name,
                confidence=conf,
                bbox=smoothed_bbox,
                distance=dist,
                angle=angle,
                bbox_clipped=raw.bbox_clipped,
            ))

        # Age unmatched tracks, remove expired
        for t in self._tracks:
            if not t['matched']:
                t['age'] += 1
        self._tracks = [t for t in self._tracks
                        if t['age'] <= self.MAX_AGE
                        and now - t['last_seen'] < self.PERSIST_TIME * 2]

        return output

    def reset(self):
        """Clear all tracks (call on each new GO command)."""
        self._tracks.clear()
