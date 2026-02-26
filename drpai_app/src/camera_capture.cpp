/*
 * Camera Capture Implementation
 *
 * Build with -DUSE_GSTREAMER to use GStreamer pipeline directly.
 * Default: OpenCV VideoCapture (uses GStreamer backend on the board).
 */

#include "camera_capture.h"
#include <stdio.h>

CameraCapture::CameraCapture() : m_width(0), m_height(0) {}

CameraCapture::~CameraCapture() {
    release();
}

bool CameraCapture::open(const std::string& device, int width, int height) {
    m_width = width;
    m_height = height;

#ifdef USE_GSTREAMER
    /* GStreamer pipeline: efficient DMA-based capture on RZ/V2N */
    char pipeline[512];
    snprintf(pipeline, sizeof(pipeline),
        "v4l2src device=%s io-mode=dmabuf ! "
        "video/x-raw,format=YUY2,width=%d,height=%d,framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink",
        device.c_str(), width, height);

    m_cap.open(pipeline, cv::CAP_GSTREAMER);
    if (!m_cap.isOpened()) {
        fprintf(stderr, "GStreamer pipeline failed, falling back to V4L2\n");
        m_cap.open(device, cv::CAP_V4L2);
    }
#else
    /* OpenCV VideoCapture: simple, portable */
    m_cap.open(device, cv::CAP_V4L2);
    if (m_cap.isOpened()) {
        m_cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        m_cap.set(cv::CAP_PROP_FPS, 30);
    }
#endif

    if (!m_cap.isOpened()) {
        fprintf(stderr, "ERROR: Cannot open camera: %s\n", device.c_str());
        return false;
    }

    printf("Camera opened: %s (%dx%d)\n", device.c_str(),
           (int)m_cap.get(cv::CAP_PROP_FRAME_WIDTH),
           (int)m_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    return true;
}

bool CameraCapture::read(cv::Mat& frame) {
    return m_cap.read(frame) && !frame.empty();
}

void CameraCapture::warmup(int num_frames) {
    cv::Mat frame;
    for (int i = 0; i < num_frames && m_cap.isOpened(); i++) {
        m_cap.read(frame);
    }
}

void CameraCapture::release() {
    if (m_cap.isOpened()) {
        m_cap.release();
    }
}

bool CameraCapture::is_opened() const {
    return m_cap.isOpened();
}
