#ifndef YOLO_POSTPROCESS_H
#define YOLO_POSTPROCESS_H

/*
 * YOLOv8 Post-processing
 *
 * Two modes:
 *   1. Decoded format:   single [1, 4+NC, num_anchors] tensor (cx, cy, w, h, cls...)
 *   2. Cut heads format: 6 tensors — 3 scales × (cv2 box DFL + cv3 class scores)
 *      cv2: [1, 64, H, W] — 4 directions × 16 DFL bins (Distribution Focal Loss)
 *      cv3: [1, NC, H, W] — raw class logits (need sigmoid)
 */

#include <vector>

struct Detection {
    float x1, y1, x2, y2;  /* Bounding box (xyxy) in model input pixel space */
    float confidence;
    int class_id;
};

/*
 * Post-process decoded YOLOv8 output tensor.
 * Input: single tensor [1, 4+num_classes, num_anchors] (row-major)
 * Box values are already cx, cy, w, h in pixel space.
 */
std::vector<Detection> yolo_postprocess(
    const float* output,
    int num_classes,
    int num_anchors,
    float conf_thresh,
    float nms_thresh,
    int max_det
);

/*
 * Post-process YOLOv8 cut heads (DFL raw format).
 *
 * This is the Renesas-recommended output format where post-processing nodes
 * are cut from the ONNX graph before DRP-AI compilation.
 *
 * @param outputs       Array of 6 float* pointers (cv2_0, cv3_0, cv2_1, cv3_1, cv2_2, cv3_2)
 * @param output_sizes  Array of 6 element counts
 * @param num_classes   Number of classes (from cv3 channel count)
 * @param input_w       Model input width (e.g. 640)
 * @param input_h       Model input height (e.g. 640)
 * @param conf_thresh   Confidence threshold (after sigmoid)
 * @param nms_thresh    IoU threshold for NMS
 * @param max_det       Maximum detections to return
 */
std::vector<Detection> yolov8_cutheads_postprocess(
    float* const* outputs,
    const int* output_sizes,
    int num_classes,
    int input_w,
    int input_h,
    float conf_thresh,
    float nms_thresh,
    int max_det
);

#endif /* YOLO_POSTPROCESS_H */
