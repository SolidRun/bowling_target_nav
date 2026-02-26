#!/usr/bin/env python3
"""Lightweight web-based remote desktop for V2N Weston.

Captures the screen via DRM framebuffer, streams as MJPEG over HTTP,
and injects mouse events via uinput for interactive control.

Usage:
    Open http://192.168.50.1:8080 in your browser.
"""

import os
import sys
import mmap
import time
import struct
import fcntl
import ctypes
import threading
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ─── Configuration ───
LISTEN_PORT = 8080
JPEG_QUALITY = 50
SCALE = 1.0          # Stream at native resolution
TARGET_FPS = 15
ROTATE_180 = True     # DSI-1 is mounted upside-down (weston transform=rotate-180)

# ─── DRM constants ───
DRM_IOCTL_MODE_MAP_DUMB = 0xC01064B3

# ─── uinput constants ───
EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
SYN_REPORT = 0x00
ABS_X, ABS_Y = 0x00, 0x01
BTN_LEFT, BTN_RIGHT, BTN_TOUCH = 0x110, 0x111, 0x14A
UI_SET_EVBIT = 0x40045564
UI_SET_KEYBIT = 0x40045565
UI_SET_ABSBIT = 0x40045567
UI_DEV_CREATE = 0x5501
UI_DEV_DESTROY = 0x5502
UINPUT_MAX_NAME = 80
ABS_CNT = 0x40


# ─── DRM ctypes structures ───
class _DrmModeRes(ctypes.Structure):
    _fields_ = [
        ("count_fbs", ctypes.c_int), ("fbs", ctypes.POINTER(ctypes.c_uint32)),
        ("count_crtcs", ctypes.c_int), ("crtcs", ctypes.POINTER(ctypes.c_uint32)),
        ("count_connectors", ctypes.c_int), ("connectors", ctypes.POINTER(ctypes.c_uint32)),
        ("count_encoders", ctypes.c_int), ("encoders", ctypes.POINTER(ctypes.c_uint32)),
        ("min_width", ctypes.c_uint32), ("max_width", ctypes.c_uint32),
        ("min_height", ctypes.c_uint32), ("max_height", ctypes.c_uint32),
    ]


class _DrmModeCrtc(ctypes.Structure):
    _fields_ = [
        ("crtc_id", ctypes.c_uint32), ("buffer_id", ctypes.c_uint32),
        ("x", ctypes.c_uint32), ("y", ctypes.c_uint32),
        ("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
        ("mode_valid", ctypes.c_int),
    ]


class _DrmModeFB(ctypes.Structure):
    _fields_ = [
        ("fb_id", ctypes.c_uint32), ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32), ("pitch", ctypes.c_uint32),
        ("bpp", ctypes.c_uint32), ("depth", ctypes.c_uint32),
        ("handle", ctypes.c_uint32),
    ]


_libdrm = ctypes.CDLL("libdrm.so.2")
_libdrm.drmModeGetResources.restype = ctypes.POINTER(_DrmModeRes)
_libdrm.drmModeGetCrtc.restype = ctypes.POINTER(_DrmModeCrtc)
_libdrm.drmModeGetFB.restype = ctypes.POINTER(_DrmModeFB)


class DRMCapture:
    """Screen capture via DRM framebuffer mmap."""

    def __init__(self):
        self.fd = os.open("/dev/dri/card0", os.O_RDWR)
        self._fb_cache = {}  # fb_id -> (mmap, width, height, pitch)
        self.width = 0
        self.height = 0

        # Find active CRTC
        res = _libdrm.drmModeGetResources(self.fd)
        if not res or res.contents.count_crtcs == 0:
            raise RuntimeError("No DRM CRTCs found")
        self.crtc_id = res.contents.crtcs[0]
        _libdrm.drmModeFreeResources(res)

        # Initial grab to detect dimensions
        frame = self._grab_raw()
        if frame is not None:
            self.height, self.width = frame.shape[:2]
            print(f"  Screen: {self.width}x{self.height}")

        # Scaled dimensions for streaming
        self.stream_w = int(self.width * SCALE)
        self.stream_h = int(self.height * SCALE)
        print(f"  Stream: {self.stream_w}x{self.stream_h} (scale={SCALE})")

    def _get_fb_mmap(self, fb_id):
        if fb_id in self._fb_cache:
            return self._fb_cache[fb_id]

        fb = _libdrm.drmModeGetFB(self.fd, fb_id)
        if not fb:
            return None
        f = fb.contents
        handle, pitch, w, h = f.handle, f.pitch, f.width, f.height

        buf = struct.pack("=IIQ", handle, 0, 0)
        result = fcntl.ioctl(self.fd, DRM_IOCTL_MODE_MAP_DUMB, buf)
        _, _, offset = struct.unpack("=IIQ", result)

        mm = mmap.mmap(self.fd, pitch * h, mmap.MAP_SHARED, mmap.PROT_READ,
                        offset=offset)
        self._fb_cache[fb_id] = (mm, w, h, pitch)
        _libdrm.drmModeFreeFB(fb)

        # Evict old entries (keep max 4 buffers)
        if len(self._fb_cache) > 4:
            oldest = next(iter(self._fb_cache))
            if oldest != fb_id:
                self._fb_cache[oldest][0].close()
                del self._fb_cache[oldest]

        return (mm, w, h, pitch)

    def _grab_raw(self):
        """Grab current screen as BGR numpy array (native resolution)."""
        crtc = _libdrm.drmModeGetCrtc(self.fd, self.crtc_id)
        if not crtc:
            return None
        fb_id = crtc.contents.buffer_id
        _libdrm.drmModeFreeCrtc(crtc)

        if fb_id == 0:
            return None

        entry = self._get_fb_mmap(fb_id)
        if not entry:
            return None
        mm, w, h, pitch = entry

        mm.seek(0)
        raw = mm.read(pitch * h)

        # BGRx layout: pitch may have padding beyond width*4
        frame = np.frombuffer(raw, dtype=np.uint8).reshape(h, pitch // 4, 4)
        frame = frame[:, :w, :3].copy()  # BGRx -> BGR, crop padding
        return frame

    def grab_jpeg(self, quality=JPEG_QUALITY):
        """Grab screen, rotate, downscale, encode to JPEG."""
        frame = self._grab_raw()
        if frame is None:
            return None

        # Rotate 180 via numpy (faster than cv2.rotate on small ARM CPU)
        if ROTATE_180:
            frame = frame[::-1, ::-1]

        # Downscale to reduce JPEG encode time
        if SCALE < 1.0:
            frame = cv2.resize(frame, (self.stream_w, self.stream_h),
                               interpolation=cv2.INTER_NEAREST)

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def close(self):
        for _, (mm, *_) in self._fb_cache.items():
            mm.close()
        self._fb_cache.clear()
        os.close(self.fd)


class UInputMouse:
    """Virtual mouse via /dev/uinput for Weston input injection."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.fd = os.open("/dev/uinput", os.O_WRONLY | os.O_NONBLOCK)

        # Enable event types
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_KEY)
        fcntl.ioctl(self.fd, UI_SET_EVBIT, EV_ABS)
        for btn in (BTN_LEFT, BTN_RIGHT, BTN_TOUCH):
            fcntl.ioctl(self.fd, UI_SET_KEYBIT, btn)
        fcntl.ioctl(self.fd, UI_SET_ABSBIT, ABS_X)
        fcntl.ioctl(self.fd, UI_SET_ABSBIT, ABS_Y)

        # Write legacy uinput_user_dev struct
        name = b'v2n-remote-mouse\x00' + b'\x00' * (UINPUT_MAX_NAME - 17)
        input_id = struct.pack('<HHHH', 0x03, 0x1234, 0x5678, 1)
        ff_effects = struct.pack('<i', 0)
        absmax = [0] * ABS_CNT
        absmin = [0] * ABS_CNT
        absfuzz = [0] * ABS_CNT
        absflat = [0] * ABS_CNT
        absmax[ABS_X] = width - 1
        absmax[ABS_Y] = height - 1
        arrays = b''.join(struct.pack(f'<{ABS_CNT}i', *a)
                          for a in (absmax, absmin, absfuzz, absflat))
        os.write(self.fd, name + input_id + ff_effects + arrays)
        fcntl.ioctl(self.fd, UI_DEV_CREATE)
        time.sleep(0.5)  # Let libinput discover the device

    def _event(self, ev_type, code, value):
        sec = int(time.time())
        usec = int((time.time() - sec) * 1e6)
        os.write(self.fd, struct.pack('<qqHHi', sec, usec, ev_type, code, value))

    def _sync(self):
        self._event(EV_SYN, SYN_REPORT, 0)

    def move(self, x, y):
        x, y = int(x), int(y)
        # Counter-rotate: Weston applies rotate-180 to input, so invert
        # coordinates so they map back to the correct logical position
        if ROTATE_180:
            x = self.width - 1 - x
            y = self.height - 1 - y
        x = max(0, min(self.width - 1, x))
        y = max(0, min(self.height - 1, y))
        self._event(EV_ABS, ABS_X, x)
        self._event(EV_ABS, ABS_Y, y)
        self._sync()

    def click(self, x, y, button=BTN_LEFT):
        self.move(x, y)
        self._event(EV_KEY, button, 1)
        self._event(EV_KEY, BTN_TOUCH, 1)
        self._sync()
        time.sleep(0.05)
        self._event(EV_KEY, button, 0)
        self._event(EV_KEY, BTN_TOUCH, 0)
        self._sync()

    def close(self):
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd, UI_DEV_DESTROY)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None


# ─── Background frame grabber ───
class FrameGrabber:
    """Continuously captures frames in a background thread."""

    def __init__(self, capture):
        self._capture = capture
        self._jpeg = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        interval = 1.0 / TARGET_FPS
        while self._running:
            t0 = time.monotonic()
            jpeg = self._capture.grab_jpeg()
            if jpeg is not None:
                with self._lock:
                    self._jpeg = jpeg
            elapsed = time.monotonic() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def get_frame(self):
        with self._lock:
            return self._jpeg

    def stop(self):
        self._running = False


# ─── HTML page ───
HTML_PAGE = """<!DOCTYPE html>
<html><head>
<title>V2N Remote Desktop</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { background: #0a0a1a; display: flex; flex-direction: column;
       align-items: center; height: 100vh; font-family: system-ui, sans-serif;
       overflow: hidden; }
.header { color: #aab; padding: 6px 12px; font-size: 14px; display: flex;
           gap: 16px; align-items: center; width: 100%;
           background: #0f0f20; z-index: 10; flex-shrink: 0; }
.header span { color: #6b8; }
.fs-btn { background: #2a5a3a; color: #cde; border: 1px solid #3a7a4a;
          border-radius: 6px; padding: 4px 14px; cursor: pointer;
          font-size: 13px; margin-left: auto; }
.fs-btn:hover { background: #3a7a4a; }
#screen-wrap { flex: 1; display: flex; align-items: center; justify-content: center;
               width: 100%; overflow: hidden; }
#screen { cursor: crosshair; max-width: 100%; max-height: 100%;
          display: block; object-fit: contain; }
body.fullscreen .header { display: none; }
body.fullscreen #screen-wrap { width: 100vw; height: 100vh; }
body.fullscreen #status { display: none; }
#status { color: #778; font-family: monospace; font-size: 11px;
          padding: 3px 8px; background: #0f0f20; text-align: center;
          width: 100%; flex-shrink: 0; z-index: 10; }
.dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
       background: #3a3; margin-right: 6px; animation: pulse 2s infinite; }
@keyframes pulse { 50% { opacity: 0.5; } }
</style>
</head><body>
<div class="header">
  <div><span class="dot"></span>V2N Remote Desktop</div>
  <div>NATIVE_W x NATIVE_H</div>
  <div id="fps" style="color:#6b8"></div>
  <button class="fs-btn" id="fsBtn">Fullscreen</button>
</div>
<div id="screen-wrap"><img id="screen" /></div>
<div id="status">Click to interact | Right-click supported | Press F11 or button for fullscreen</div>
<script>
const img = document.getElementById('screen');
const status = document.getElementById('status');
const fpsEl = document.getElementById('fps');
const fsBtn = document.getElementById('fsBtn');
const W = NATIVE_W, H = NATIVE_H;
let frames = 0, lastFpsTime = Date.now();

img.src = '/stream';
img.onload = function() { frames++; };

setInterval(function() {
    const now = Date.now();
    const fps = (frames * 1000 / (now - lastFpsTime)).toFixed(1);
    fpsEl.textContent = fps + ' FPS';
    frames = 0;
    lastFpsTime = now;
}, 2000);

// Fullscreen toggle
function toggleFS() {
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().then(function() {
            document.body.classList.add('fullscreen');
        });
    } else {
        document.exitFullscreen().then(function() {
            document.body.classList.remove('fullscreen');
        });
    }
}
fsBtn.addEventListener('click', toggleFS);
document.addEventListener('fullscreenchange', function() {
    if (!document.fullscreenElement) document.body.classList.remove('fullscreen');
});
// Double-click image to toggle fullscreen too
img.addEventListener('dblclick', function(e) {
    e.preventDefault();
    toggleFS();
});

function getCoords(e) {
    const r = img.getBoundingClientRect();
    return [
        Math.round((e.clientX - r.left) / r.width * W),
        Math.round((e.clientY - r.top) / r.height * H)
    ];
}
function send(path, x, y, extra) {
    let url = path + '?x=' + x + '&y=' + y;
    if (extra) url += '&' + extra;
    fetch(url);
}
img.addEventListener('click', function(e) {
    const [x, y] = getCoords(e);
    send('/click', x, y, 'btn=left');
    status.textContent = 'Left click: (' + x + ', ' + y + ')';
});
img.addEventListener('contextmenu', function(e) {
    e.preventDefault();
    const [x, y] = getCoords(e);
    send('/click', x, y, 'btn=right');
    status.textContent = 'Right click: (' + x + ', ' + y + ')';
});
let moveThrottle = null;
img.addEventListener('mousemove', function(e) {
    if (moveThrottle) return;
    moveThrottle = setTimeout(function() { moveThrottle = null; }, 80);
    const [x, y] = getCoords(e);
    send('/move', x, y);
});
img.addEventListener('touchstart', function(e) {
    e.preventDefault();
    const t = e.touches[0];
    const [x, y] = getCoords(t);
    send('/click', x, y, 'btn=left');
    status.textContent = 'Touch: (' + x + ', ' + y + ')';
});
</script>
</body></html>"""


MJPEG_BOUNDARY = b'--v2nframe'


class Handler(BaseHTTPRequestHandler):
    capture = None
    mouse = None
    grabber = None
    native_w = 1024
    native_h = 600

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == '/':
            page = HTML_PAGE.replace('NATIVE_W', str(self.native_w))
            page = page.replace('NATIVE_H', str(self.native_h))
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(page.encode())

        elif parsed.path == '/stream':
            # MJPEG stream - push frames continuously over one connection
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=v2nframe')
            self.send_header('Cache-Control', 'no-cache, no-store')
            self.end_headers()
            interval = 1.0 / TARGET_FPS
            try:
                while True:
                    jpeg = self.grabber.get_frame()
                    if jpeg is None:
                        time.sleep(0.1)
                        continue
                    self.wfile.write(MJPEG_BOUNDARY + b'\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                    self.wfile.flush()
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        elif parsed.path == '/frame':
            # Single frame fallback
            jpeg = self.grabber.get_frame()
            if jpeg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-cache, no-store')
            self.end_headers()
            self.wfile.write(jpeg)

        elif parsed.path == '/click':
            params = parse_qs(parsed.query)
            x = int(params.get('x', ['0'])[0])
            y = int(params.get('y', ['0'])[0])
            btn_name = params.get('btn', ['left'])[0]
            btn = BTN_RIGHT if btn_name == 'right' else BTN_LEFT
            if self.mouse:
                self.mouse.click(x, y, btn)
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Content-Length', '2')
            self.end_headers()
            self.wfile.write(b'ok')

        elif parsed.path == '/move':
            params = parse_qs(parsed.query)
            x = int(params.get('x', ['0'])[0])
            y = int(params.get('y', ['0'])[0])
            if self.mouse:
                self.mouse.move(x, y)
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Content-Length', '2')
            self.end_headers()
            self.wfile.write(b'ok')

        else:
            self.send_response(404)
            self.end_headers()


class ThreadedHTTPServer(HTTPServer):
    """Handle each request in a new thread (needed for concurrent stream + clicks)."""
    allow_reuse_address = True
    daemon_threads = True

    def process_request(self, request, client_address):
        t = threading.Thread(target=self.process_request_thread,
                             args=(request, client_address), daemon=True)
        t.start()

    def process_request_thread(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


def main():
    print("V2N Remote Desktop Server")
    print("=" * 40)

    # Screen capture via DRM
    try:
        capture = DRMCapture()
    except Exception as e:
        print(f"ERROR: DRM capture init failed: {e}")
        sys.exit(1)

    # Background frame grabber
    grabber = FrameGrabber(capture)

    Handler.capture = capture
    Handler.grabber = grabber
    Handler.native_w = capture.width
    Handler.native_h = capture.height

    # Mouse input via uinput (uses native resolution for coordinate mapping)
    try:
        mouse = UInputMouse(capture.width, capture.height)
        Handler.mouse = mouse
        print(f"  Mouse input: enabled")
    except Exception as e:
        print(f"  Mouse input: disabled ({e})")
        mouse = None

    # HTTP server
    server = ThreadedHTTPServer(('0.0.0.0', LISTEN_PORT), Handler)
    print(f"  URL: http://192.168.50.1:{LISTEN_PORT}")
    print(f"  Target FPS: {TARGET_FPS}, JPEG quality: {JPEG_QUALITY}")
    print()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        grabber.stop()
        capture.close()
        if mouse:
            mouse.close()
        server.server_close()


if __name__ == '__main__':
    main()
