#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

/*
 * Camera Capture
 * Supports OpenCV VideoCapture (default) and GStreamer pipeline.
 * Define USE_GSTREAMER at compile time to use GStreamer directly.
 */

#include <string>
#include <opencv2/opencv.hpp>

class CameraCapture {
public:
    CameraCapture();
    ~CameraCapture();

    bool open(const std::string& device, int width, int height);
    bool read(cv::Mat& frame);
    void warmup(int num_frames);
    void release();
    bool is_opened() const;

private:
    cv::VideoCapture m_cap;
    int m_width;
    int m_height;
};

#endif /* CAMERA_CAPTURE_H */
