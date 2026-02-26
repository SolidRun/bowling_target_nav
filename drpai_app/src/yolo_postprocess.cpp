/*
 * YOLOv8 Post-processing Implementation
 *
 * Two modes:
 *   1. Decoded: [1, 4+NC, num_anchors] — cx, cy, w, h already computed
 *   2. Cut heads: 6 tensors with DFL box regression + raw class logits
 *
 * The cut heads format is the Renesas-recommended way to run YOLOv8 on DRP-AI.
 * Post-processing nodes are removed from the ONNX graph so the DRP-AI accelerator
 * handles the compute-heavy convolutions while the CPU does the lightweight decode.
 */

#include "yolo_postprocess.h"
#include "define.h"
#include <algorithm>
#include <cmath>

/* ═══════════════════════════════════════════════════════════════════════
 * NMS (shared between both modes)
 * ═══════════════════════════════════════════════════════════════════════ */

static float iou(const Detection& a, const Detection& b) {
    float x1 = std::max(a.x1, b.x1);
    float y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2);
    float y2 = std::min(a.y2, b.y2);

    float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    float union_area = area_a + area_b - inter;

    return (union_area > 0) ? (inter / union_area) : 0.0f;
}

static std::vector<Detection> nms(std::vector<Detection>& candidates,
                                   float nms_thresh, int max_det) {
    std::sort(candidates.begin(), candidates.end(),
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });

    std::vector<Detection> results;
    std::vector<bool> suppressed(candidates.size(), false);

    for (size_t i = 0; i < candidates.size() && (int)results.size() < max_det; i++) {
        if (suppressed[i]) continue;

        results.push_back(candidates[i]);

        for (size_t j = i + 1; j < candidates.size(); j++) {
            if (suppressed[j]) continue;
            if (candidates[i].class_id == candidates[j].class_id &&
                iou(candidates[i], candidates[j]) > nms_thresh) {
                suppressed[j] = true;
            }
        }
    }

    return results;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Mode 1: Decoded output [1, 4+NC, num_anchors]
 * ═══════════════════════════════════════════════════════════════════════ */

std::vector<Detection> yolo_postprocess(
    const float* output,
    int num_classes,
    int num_anchors,
    float conf_thresh,
    float nms_thresh,
    int max_det)
{
    std::vector<Detection> candidates;
    candidates.reserve(num_anchors / 4);

    /*
     * Output layout (row-major): [1, channels, num_anchors]
     * Element at [0, c, a] = output[c * num_anchors + a]
     */
    for (int a = 0; a < num_anchors; a++) {
        /* Find best class score */
        float max_score = -1.0f;
        int best_class = 0;
        for (int c = 0; c < num_classes; c++) {
            float score = output[(4 + c) * num_anchors + a];
            if (score > max_score) {
                max_score = score;
                best_class = c;
            }
        }

        if (max_score < conf_thresh) continue;

        /* Extract box: cx, cy, w, h */
        float cx = output[0 * num_anchors + a];
        float cy = output[1 * num_anchors + a];
        float w  = output[2 * num_anchors + a];
        float h  = output[3 * num_anchors + a];

        /* Convert xywh -> xyxy */
        Detection det;
        det.x1 = cx - w * 0.5f;
        det.y1 = cy - h * 0.5f;
        det.x2 = cx + w * 0.5f;
        det.y2 = cy + h * 0.5f;
        det.confidence = max_score;
        det.class_id = best_class;

        candidates.push_back(det);
    }

    return nms(candidates, nms_thresh, max_det);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Mode 2: Cut heads — YOLOv8 DFL decode
 *
 * 6 output tensors in order:
 *   [0] cv2_0: [1, 64, 80, 80]  — box DFL at stride 8
 *   [1] cv3_0: [1, NC, 80, 80]  — class logits at stride 8
 *   [2] cv2_1: [1, 64, 40, 40]  — box DFL at stride 16
 *   [3] cv3_1: [1, NC, 40, 40]  — class logits at stride 16
 *   [4] cv2_2: [1, 64, 20, 20]  — box DFL at stride 32
 *   [5] cv3_2: [1, NC, 20, 20]  — class logits at stride 32
 *
 * cv2 layout: 64 channels = 4 directions × 16 DFL bins
 *   ch  0-15: left distance bins
 *   ch 16-31: top distance bins
 *   ch 32-47: right distance bins
 *   ch 48-63: bottom distance bins
 *
 * DFL decode: softmax(16 bins) → weighted sum → distance in grid units
 * Box: x1 = (col + 0.5 - dist_left) × stride
 *      y1 = (row + 0.5 - dist_top)  × stride
 *      x2 = (col + 0.5 + dist_right)  × stride
 *      y2 = (row + 0.5 + dist_bottom) × stride
 * ═══════════════════════════════════════════════════════════════════════ */

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

static inline float dfl_decode(const float* data, int channel_offset, int grid_size, int idx) {
    /*
     * DFL: softmax over DFL_REG_MAX bins, then weighted sum.
     * data layout: [64, grid_h, grid_w] — channel c at spatial idx = data[c * grid_size + idx]
     */
    float bins[DFL_REG_MAX];
    float max_val = -1e9f;

    for (int i = 0; i < DFL_REG_MAX; i++) {
        bins[i] = data[(channel_offset + i) * grid_size + idx];
        if (bins[i] > max_val) max_val = bins[i];
    }

    float sum_exp = 0;
    for (int i = 0; i < DFL_REG_MAX; i++) {
        bins[i] = expf(bins[i] - max_val);
        sum_exp += bins[i];
    }

    float weighted = 0;
    for (int i = 0; i < DFL_REG_MAX; i++) {
        weighted += i * bins[i];
    }

    return weighted / sum_exp;
}

std::vector<Detection> yolov8_cutheads_postprocess(
    float* const* outputs,
    const int* output_sizes,
    int num_classes,
    int input_w,
    int input_h,
    float conf_thresh,
    float nms_thresh,
    int max_det)
{
    const int strides[] = {8, 16, 32};
    const int num_scales = 3;

    std::vector<Detection> candidates;
    candidates.reserve(1000);

    for (int s = 0; s < num_scales; s++) {
        const float* cv2 = outputs[s * 2];      /* box DFL [1, 64, H, W] */
        const float* cv3 = outputs[s * 2 + 1];  /* class scores [1, NC, H, W] */

        if (!cv2 || !cv3) continue;

        int stride = strides[s];
        int grid_h = input_h / stride;
        int grid_w = input_w / stride;
        int grid_size = grid_h * grid_w;

        for (int row = 0; row < grid_h; row++) {
            for (int col = 0; col < grid_w; col++) {
                int idx = row * grid_w + col;

                /* ── Class score (apply sigmoid to raw logits) ── */
                float max_score = -1.0f;
                int best_class = 0;
                for (int c = 0; c < num_classes; c++) {
                    float score = sigmoid(cv3[c * grid_size + idx]);
                    if (score > max_score) {
                        max_score = score;
                        best_class = c;
                    }
                }

                if (max_score < conf_thresh) continue;

                /* ── DFL decode — 4 distances (left, top, right, bottom) ── */
                float dist_left   = dfl_decode(cv2,  0 * DFL_REG_MAX, grid_size, idx);
                float dist_top    = dfl_decode(cv2, 1 * DFL_REG_MAX, grid_size, idx);
                float dist_right  = dfl_decode(cv2, 2 * DFL_REG_MAX, grid_size, idx);
                float dist_bottom = dfl_decode(cv2, 3 * DFL_REG_MAX, grid_size, idx);

                /* ── Convert to pixel coordinates ── */
                float anchor_x = (col + 0.5f);
                float anchor_y = (row + 0.5f);

                Detection det;
                det.x1 = (anchor_x - dist_left)   * stride;
                det.y1 = (anchor_y - dist_top)    * stride;
                det.x2 = (anchor_x + dist_right)  * stride;
                det.y2 = (anchor_y + dist_bottom) * stride;
                det.confidence = max_score;
                det.class_id = best_class;

                candidates.push_back(det);
            }
        }
    }

    return nms(candidates, nms_thresh, max_det);
}
