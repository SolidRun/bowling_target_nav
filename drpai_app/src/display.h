#ifndef DISPLAY_H
#define DISPLAY_H

/*
 * Display Output
 * Console mode: prints detections to stdout
 * Wayland mode: draws bounding boxes on frame, displays via GStreamer waylandsink
 *               (avoids OpenCV's cv::imshow which uses GTK and fails on Weston)
 *
 * Display mode is selected at runtime via Display::init(use_wayland).
 */

#include <vector>
#include <opencv2/opencv.hpp>
#include "yolo_postprocess.h"

class Display {
public:
    Display();
    ~Display();

    void init(int width, int height, bool use_wayland);
    void show(cv::Mat& frame, const std::vector<Detection>& detections,
              float infer_ms, float camera_fps = 0, float inference_fps = 0);

private:
    int m_width;
    int m_height;
    int m_frame_count;
    bool m_use_wayland;
    cv::VideoWriter m_writer;
    bool m_writer_ok;
};

#endif /* DISPLAY_H */
