/*
 * YOLO Object Detection — Renesas RZ/V2N DRP-AI
 *
 * Runtime-configurable detection application.
 * All parameters can be overridden via config.ini without rebuilding.
 *
 * Priority: CLI args > config.ini > compile-time defaults (define.h)
 *
 * Usage:
 *   ./app_yolo_cam [options] [model_dir] [camera_device]
 *   ./app_yolo_cam --pipe [model_dir]
 *   ./app_yolo_cam --stream [model_dir]
 *
 * Options:
 *   --console    Console output only (no Wayland display)
 *   --display    Force Wayland display
 *   --pipe       Pipe mode (stdin BGR frames → stdout JSON)
 *   --stream     Stream mode (camera + DRP-AI → shm frames + stdout JSON)
 *   --help, -h   Show help
 *
 * Note: Must run as root to access /dev/drpai0
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <vector>

#include "define.h"
#include "drpai_inference.h"
#include "yolo_postprocess.h"
#include "camera_capture.h"
#include "display.h"

/* V2N OCA workaround: prevents DRP-AI / OpenCV accelerator contention */
#ifdef V2N
#include <opencv2/imgproc.hpp>
#endif

static std::atomic<bool> g_running(true);

/* ═══════════════════════════════════════════════════════════════════════
 * Runtime Configuration
 *
 * Initialized from compile-time defaults (define.h).
 * Overridden by config.ini, then by CLI arguments.
 * ═══════════════════════════════════════════════════════════════════════ */
struct AppConfig {
    /* Detection */
    float conf_threshold = CONF_THRESHOLD;
    float nms_threshold  = NMS_THRESHOLD;
    int   max_detections = MAX_DETECTIONS;

    /* Camera */
    std::string camera_device = CAMERA_DEVICE;
    int   camera_width   = CAMERA_W;
    int   camera_height  = CAMERA_H;
    int   warmup_frames  = 10;

    /* Model */
    std::string model_dir = "model";

    /* Display */
    bool  use_wayland = (DISPLAY_MODE == 1);

    /* DRP-AI frequencies (passed to drpai_inference at load time) */
    int   drp_max_freq = DRP_MAX_FREQ;
    int   drpai_freq   = DRPAI_FREQ;

    /* Thread polling interval (microseconds) — used by multi-threaded camera pipeline */
    int   wait_time = WAIT_TIME;

    /* Runtime class names (loaded from labels.txt or config.ini, overrides CLASS_NAMES) */
    std::vector<std::string> class_names;
    std::string display_label;  /* Short label for overlay (e.g., "Pin", "Target") */
};

static AppConfig g_cfg;

/* Get class name by ID — uses runtime names if loaded, else compile-time fallback */
const char* get_class_name(int class_id) {
    if (!g_cfg.class_names.empty() && class_id >= 0 &&
        class_id < (int)g_cfg.class_names.size())
        return g_cfg.class_names[class_id].c_str();
    if (class_id >= 0 && class_id < NUM_CLASSES)
        return CLASS_NAMES[class_id];
    return "unknown";
}

/* Get display label (short name for overlay, e.g., "Pin", "Target") */
static const char* get_display_label() {
    if (!g_cfg.display_label.empty())
        return g_cfg.display_label.c_str();
    /* Default: first class name */
    return get_class_name(0);
}

/* Load class names from labels.txt (one name per line) */
static int load_labels(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return 0;
    g_cfg.class_names.clear();
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        /* Trim trailing whitespace/newline */
        char* end = line + strlen(line) - 1;
        while (end >= line && (*end == '\n' || *end == '\r' || *end == ' ')) *end-- = '\0';
        if (line[0] != '\0' && line[0] != '#')
            g_cfg.class_names.emplace_back(line);
    }
    fclose(f);
    return (int)g_cfg.class_names.size();
}

static void signal_handler(int signum) {
    (void)signum;
    g_running.store(false);
}

/* ── DRP-AI frequency → human-readable string ── */
static void format_drp_freq(int idx, char* buf, size_t sz) {
    /* DRP core: freq_MHz ≈ 1260 / (idx + 1) */
    switch (idx) {
        case 2:   snprintf(buf, sz, "420 MHz (idx=%d)", idx); break;
        case 3:   snprintf(buf, sz, "315 MHz (idx=%d)", idx); break;
        case 4:   snprintf(buf, sz, "252 MHz (idx=%d)", idx); break;
        case 5:   snprintf(buf, sz, "210 MHz (idx=%d)", idx); break;
        case 127: snprintf(buf, sz, "9.8 MHz (idx=%d, minimum)", idx); break;
        default:  snprintf(buf, sz, "%.0f MHz (idx=%d)", 1260.0 / (idx + 1), idx); break;
    }
}

static void format_aimac_freq(int idx, char* buf, size_t sz) {
    /* AI-MAC: known values from Renesas datasheet */
    switch (idx) {
        case 1:   snprintf(buf, sz, "1 GHz (idx=%d)", idx); break;
        case 2:   snprintf(buf, sz, "1 GHz (idx=%d)", idx); break;
        case 3:   snprintf(buf, sz, "630 MHz (idx=%d)", idx); break;
        case 4:   snprintf(buf, sz, "420 MHz (idx=%d)", idx); break;
        case 5:   snprintf(buf, sz, "315 MHz (idx=%d)", idx); break;
        case 127: snprintf(buf, sz, "10 MHz (idx=%d, minimum)", idx); break;
        default:  snprintf(buf, sz, "%.0f MHz (idx=%d)", 1260.0 / idx, idx); break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Config File Parser
 *
 * Reads key = value pairs. Lines starting with # are comments.
 * Supports all runtime parameters — see config.ini for the full list.
 * ═══════════════════════════════════════════════════════════════════════ */
static int load_config(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return 0;

    int count = 0;
    char line[512];
    while (fgets(line, sizeof(line), f)) {
        /* Skip leading whitespace */
        char* p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\0' || *p == '\n' || *p == '\r') continue;

        char key[64] = {0};
        char val[256] = {0};
        if (sscanf(p, "%63[^= \t\n\r] %*[= \t] %255[^\n\r]", key, val) < 2) continue;

        /* Trim trailing whitespace from value */
        char* end = val + strlen(val) - 1;
        while (end > val && (*end == ' ' || *end == '\t' || *end == '\r')) *end-- = '\0';

        /* Strip inline comments (# after value) */
        char* hash = strchr(val, '#');
        if (hash) {
            *hash = '\0';
            end = hash - 1;
            while (end > val && (*end == ' ' || *end == '\t')) *end-- = '\0';
        }

        if (val[0] == '\0') continue;

        /* ── Detection ── */
        if      (strcmp(key, "conf_threshold") == 0)  { g_cfg.conf_threshold = strtof(val, NULL); count++; }
        else if (strcmp(key, "nms_threshold") == 0)    { g_cfg.nms_threshold = strtof(val, NULL); count++; }
        else if (strcmp(key, "max_detections") == 0)   { g_cfg.max_detections = atoi(val); count++; }
        /* ── Camera ── */
        else if (strcmp(key, "camera_device") == 0)    { g_cfg.camera_device = val; count++; }
        else if (strcmp(key, "camera_width") == 0)     { g_cfg.camera_width = atoi(val); count++; }
        else if (strcmp(key, "camera_height") == 0)    { g_cfg.camera_height = atoi(val); count++; }
        else if (strcmp(key, "warmup_frames") == 0)    { g_cfg.warmup_frames = atoi(val); count++; }
        /* ── Model ── */
        else if (strcmp(key, "model_dir") == 0)        { g_cfg.model_dir = val; count++; }
        /* ── Display ── */
        else if (strcmp(key, "display_mode") == 0) {
            if (strcmp(val, "wayland") == 0 || strcmp(val, "1") == 0) g_cfg.use_wayland = true;
            else g_cfg.use_wayland = false;
            count++;
        }
        /* ── DRP-AI frequencies ── */
        else if (strcmp(key, "drp_max_freq") == 0)     { g_cfg.drp_max_freq = atoi(val); count++; }
        else if (strcmp(key, "drpai_freq") == 0)       { g_cfg.drpai_freq = atoi(val); count++; }
        /* ── Threading ── */
        else if (strcmp(key, "wait_time") == 0)        { g_cfg.wait_time = atoi(val); count++; }
        /* ── Class names ── */
        else if (strcmp(key, "display_label") == 0)    { g_cfg.display_label = val; count++; }
        else if (strcmp(key, "class_names") == 0) {
            g_cfg.class_names.clear();
            char* tok = strtok(val, ",");
            while (tok) {
                while (*tok == ' ') tok++;
                char* e = tok + strlen(tok) - 1;
                while (e > tok && *e == ' ') *e-- = '\0';
                if (*tok) g_cfg.class_names.emplace_back(tok);
                tok = strtok(NULL, ",");
            }
            count++;
        }
    }
    fclose(f);
    return count;
}

/* ═══════════════════════════════════════════════════════════════════════
 * I/O Helpers
 * ═══════════════════════════════════════════════════════════════════════ */
static bool read_exact(void* buf, size_t n) {
    size_t total = 0;
    uint8_t* p = static_cast<uint8_t*>(buf);
    while (total < n) {
        size_t r = fread(p + total, 1, n - total, stdin);
        if (r == 0) return false;
        total += r;
    }
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Preprocessing
 *
 * YOLO expects: [1, 3, H, W] float32, RGB channels, normalized [0, 1]
 * Camera gives:  BGR uint8 (OpenCV default)
 *
 * Steps: resize → BGR→RGB → normalize to [0,1] → HWC→CHW
 * ═══════════════════════════════════════════════════════════════════════ */

/* ── Letterbox padding offsets (used to undo scaling in post-process) ── */
static float g_letterbox_scale = 1.0f;
static int   g_letterbox_pad_x = 0;
static int   g_letterbox_pad_y = 0;

/* Letterbox: resize preserving aspect ratio + gray padding.
 * Uses OpenCV resize (SIMD-optimized) for the heavy pixel work,
 * then does CHW conversion + padding in a single pass.
 * OCA is disabled at startup so OpenCV runs on CPU only — no DRP conflict. */
static void letterbox_to_chw(const uint8_t* src_bgr, int src_w, int src_h,
                              std::vector<float>& out) {
    out.assign(INPUT_C * INPUT_H * INPUT_W, 0.5f); /* gray pad = 128/255 */

    float scale = std::min((float)INPUT_W / src_w, (float)INPUT_H / src_h);
    int new_w = (int)(src_w * scale);
    int new_h = (int)(src_h * scale);
    int pad_x = (INPUT_W - new_w) / 2;
    int pad_y = (INPUT_H - new_h) / 2;

    g_letterbox_scale = scale;
    g_letterbox_pad_x = pad_x;
    g_letterbox_pad_y = pad_y;

    /* Use OpenCV for fast SIMD resize + color conversion */
    cv::Mat src(src_h, src_w, CV_8UC3, (void*)src_bgr);
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    /* CHW conversion with padding — single pass over resized pixels */
    const float inv255 = 1.0f / 255.0f;
    for (int dy = 0; dy < new_h; dy++) {
        const uint8_t* row = rgb.ptr<uint8_t>(dy);
        int oy = dy + pad_y;
        for (int dx = 0; dx < new_w; dx++) {
            int ox = dx + pad_x;
            int idx3 = dx * 3;
            out[0 * INPUT_H * INPUT_W + oy * INPUT_W + ox] = row[idx3]     * inv255;
            out[1 * INPUT_H * INPUT_W + oy * INPUT_W + ox] = row[idx3 + 1] * inv255;
            out[2 * INPUT_H * INPUT_W + oy * INPUT_W + ox] = row[idx3 + 2] * inv255;
        }
    }
}

/* Preprocessing for pipe mode — same letterbox as camera mode */
static void preprocess_no_opencv(const uint8_t* bgr, int src_w, int src_h,
                                  std::vector<float>& out) {
    letterbox_to_chw(bgr, src_w, src_h, out);
}


/* OpenCV preprocessing for camera mode — letterbox only, no OpenCV
 * processing to avoid any DRP/OCA contention risk. All contrast
 * enhancement is done inline in the letterbox pass (pure C++, no
 * OpenCV calls). */
static void preprocess(const cv::Mat& frame, std::vector<float>& out) {
    letterbox_to_chw(frame.data, frame.cols, frame.rows, out);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Detection Pipeline
 *
 * inference → post-process → scale & clip boxes to frame bounds
 * ═══════════════════════════════════════════════════════════════════════ */
static bool g_first_frame = true;

static void dump_output_debug_decoded(const float* data, int size) {
    int expected = (4 + NUM_CLASSES) * NUM_ANCHORS;
    fprintf(stderr, "\n[DEBUG] === Raw DRP-AI Output (decoded format) ===\n");
    fprintf(stderr, "  output_size: %d (expected %d = %d ch x %d anchors)\n",
            size, expected, 4 + NUM_CLASSES, NUM_ANCHORS);

    /* Print first 20 raw values */
    fprintf(stderr, "  First 20 values:");
    for (int i = 0; i < 20 && i < size; i++)
        fprintf(stderr, " %.4f", data[i]);
    fprintf(stderr, "\n");

    /* Per-channel stats assuming [channels, anchors] layout */
    int num_ch = 4 + NUM_CLASSES;
    int na = NUM_ANCHORS;
    if (size >= num_ch * na) {
        const char* ch_names[] = {"cx", "cy", "w", "h", "cls0"};
        for (int c = 0; c < num_ch && c < 5; c++) {
            float mn = 1e9f, mx = -1e9f, sum = 0;
            for (int a = 0; a < na; a++) {
                float v = data[c * na + a];
                if (v < mn) mn = v;
                if (v > mx) mx = v;
                sum += v;
            }
            fprintf(stderr, "  ch[%d] %-4s: min=%.4f  max=%.4f  mean=%.4f\n",
                    c, ch_names[c], mn, mx, sum / na);
        }
    } else {
        fprintf(stderr, "  WARNING: output_size %d < expected %d!\n", size, num_ch * na);
    }
    fprintf(stderr, "[DEBUG] === End Dump ===\n\n");
}

static void dump_output_debug_cutheads(DRPAIInference& drpai) {
    const char* tensor_names[] = {
        "cv2_0 [box DFL, stride 8]",  "cv3_0 [class, stride 8]",
        "cv2_1 [box DFL, stride 16]", "cv3_1 [class, stride 16]",
        "cv2_2 [box DFL, stride 32]", "cv3_2 [class, stride 32]"
    };

    fprintf(stderr, "\n[DEBUG] === Raw DRP-AI Output (cut heads, %d tensors) ===\n",
            drpai.get_num_outputs());

    for (int i = 0; i < drpai.get_num_outputs() && i < 6; i++) {
        int sz = 0;
        float* data = drpai.get_output(i, sz);
        if (!data || sz == 0) {
            fprintf(stderr, "  [%d] %s: EMPTY\n", i, tensor_names[i]);
            continue;
        }

        float mn = 1e9f, mx = -1e9f, sum = 0;
        for (int j = 0; j < sz; j++) {
            if (data[j] < mn) mn = data[j];
            if (data[j] > mx) mx = data[j];
            sum += data[j];
        }
        fprintf(stderr, "  [%d] %s: %d elems, min=%.4f max=%.4f mean=%.4f\n",
                i, tensor_names[i], sz, mn, mx, sum / sz);

        /* First 8 values */
        fprintf(stderr, "      first 8:");
        for (int j = 0; j < 8 && j < sz; j++)
            fprintf(stderr, " %.4f", data[j]);
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "[DEBUG] === End Dump ===\n\n");
}

static std::vector<Detection> detect(DRPAIInference& drpai,
                                      const std::vector<float>& input,
                                      int frame_w, int frame_h,
                                      float& infer_ms) {
    auto t0 = std::chrono::steady_clock::now();

    if (!drpai.run(input.data(), input.size())) {
        infer_ms = 0;
        return {};
    }

    std::vector<Detection> dets;

#if OUTPUT_FORMAT == 1
    /* ── Cut heads mode: retrieve all 6 output tensors and DFL decode ── */
    float* out_ptrs[NUM_OUTPUTS];
    int    out_sizes[NUM_OUTPUTS];
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        out_ptrs[i] = drpai.get_output(i, out_sizes[i]);
    }

    auto t1 = std::chrono::steady_clock::now();
    infer_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    /* Dump raw output on first real camera frame for debugging */
    if (g_first_frame) {
        dump_output_debug_cutheads(drpai);
        g_first_frame = false;
    }

    dets = yolov8_cutheads_postprocess(out_ptrs, out_sizes,
                                        NUM_CLASSES, INPUT_W, INPUT_H,
                                        g_cfg.conf_threshold, g_cfg.nms_threshold,
                                        g_cfg.max_detections);
#else
    /* ── Decoded mode: single output tensor [1, 4+NC, anchors] ── */
    int out_size = 0;
    float* out_data = drpai.get_output(0, out_size);

    auto t1 = std::chrono::steady_clock::now();
    infer_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

    if (!out_data || out_size == 0) return {};

    /* Dump raw output on first real camera frame for debugging */
    if (g_first_frame) {
        dump_output_debug_decoded(out_data, out_size);
        g_first_frame = false;
    }

    dets = yolo_postprocess(out_data, NUM_CLASSES, NUM_ANCHORS,
                             g_cfg.conf_threshold, g_cfg.nms_threshold,
                             g_cfg.max_detections);
#endif

    /* Undo letterbox transform: remove padding, then scale to camera frame.
     * Model input has gray padding around the actual image content.
     * Boxes are in model input space (640×640), need to map back to camera (640×480). */
    float lb_scale = g_letterbox_scale;
    int   lb_px    = g_letterbox_pad_x;
    int   lb_py    = g_letterbox_pad_y;
    for (auto& d : dets) {
        d.x1 = std::max(0.0f, std::min((d.x1 - lb_px) / lb_scale, (float)frame_w));
        d.y1 = std::max(0.0f, std::min((d.y1 - lb_py) / lb_scale, (float)frame_h));
        d.x2 = std::max(0.0f, std::min((d.x2 - lb_px) / lb_scale, (float)frame_w));
        d.y2 = std::max(0.0f, std::min((d.y2 - lb_py) / lb_scale, (float)frame_h));
    }
    return dets;
}

/* ═══════════════════════════════════════════════════════════════════════
 * JSON Output (Pipe Mode)
 * ═══════════════════════════════════════════════════════════════════════ */
static void write_json(const std::vector<Detection>& dets, float infer_ms) {
    printf("{\"detections\":[");
    for (size_t i = 0; i < dets.size(); i++) {
        const auto& d = dets[i];
        if (i > 0) printf(",");
        printf("{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,"
               "\"confidence\":%.4f,\"class_id\":%d,\"class_name\":\"%s\"}",
               (int)d.x1, (int)d.y1, (int)d.x2, (int)d.y2,
               d.confidence, d.class_id,
               get_class_name(d.class_id));
    }
    printf("],\"inference_ms\":%.1f}\n", infer_ms);
    fflush(stdout);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Pipe Mode
 *
 * Protocol:
 *   Input:   [uint32 width][uint32 height][width*height*3 BGR bytes]
 *   Output:  JSON line per frame
 *   Startup: writes "READY\n" after model load
 * ═══════════════════════════════════════════════════════════════════════ */
static int run_pipe_mode(DRPAIInference& drpai) {
    fprintf(stderr, "[pipe] Ready (CPU preprocessing, no OpenCV).\n");
    printf("READY\n");
    fflush(stdout);

    std::vector<float> input;
    std::vector<uint8_t> buf;

    while (g_running) {
        uint32_t w = 0, h = 0;
        if (!read_exact(&w, 4) || !read_exact(&h, 4)) break;

        if (w == 0 || h == 0 || w > 4096 || h > 4096) {
            fprintf(stderr, "[pipe] Invalid frame: %ux%u\n", w, h);
            break;
        }

        size_t sz = (size_t)w * h * 3;
        buf.resize(sz);
        if (!read_exact(buf.data(), sz)) break;

        preprocess_no_opencv(buf.data(), w, h, input);

        float ms = 0;
        auto dets = detect(drpai, input, w, h, ms);
        write_json(dets, ms);
    }

    fprintf(stderr, "[pipe] Done.\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Distance Estimation + Stream Overlay Drawing
 *
 * In stream mode, ALL image processing is done in C++ (DRP-AI accelerated).
 * Python GUI just displays the pre-rendered frames from shared memory.
 * ═══════════════════════════════════════════════════════════════════════ */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void estimate_distance(float x1, float y1, float x2, float y2,
                               int frame_w, int frame_h,
                               float ref_box_height, float ref_distance, float hfov_deg,
                               float& out_distance, float& out_angle) {
    float hfov_rad = hfov_deg * (float)M_PI / 180.0f;
    float box_h = y2 - y1;
    if (box_h < 1.0f) box_h = 1.0f;
    out_distance = ref_distance * ref_box_height / box_h;

    float cx = (x1 + x2) / 2.0f;
    out_angle = (cx - frame_w / 2.0f) / (float)frame_w * hfov_rad;
}

/* Default calibration values (used when no shm calibration available) */
static const float DEFAULT_REF_HEIGHT = 200.0f;
static const float DEFAULT_REF_DISTANCE = 1.0f;
static const float DEFAULT_FOV = 60.0f;

static void draw_stream_overlays(cv::Mat& frame,
                                  const std::vector<Detection>& dets,
                                  float infer_ms) {
    int fw = frame.cols;
    int fh = frame.rows;

    /* ── Always-on status bar (top-left) ── */
    /* Semi-transparent background bar */
    cv::Mat bar_roi = frame(cv::Rect(0, 0, fw, 32));
    bar_roi *= 0.4;

    /* DRP-AI badge */
    cv::putText(frame, "DRP-AI", cv::Point(8, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 0), 2);

    /* Inference time */
    char ai_text[64];
    snprintf(ai_text, sizeof(ai_text), "%.0fms", infer_ms);
    cv::putText(frame, ai_text, cv::Point(100, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1);

    /* Target label */
    char target_text[64];
    snprintf(target_text, sizeof(target_text), "Target: %s", get_display_label());
    cv::putText(frame, target_text, cv::Point(170, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    /* Detection count */
    char det_text[64];
    snprintf(det_text, sizeof(det_text), "Det: %d", (int)dets.size());
    cv::putText(frame, det_text, cv::Point(fw - 80, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.5,
                dets.empty() ? cv::Scalar(100, 100, 100) : cv::Scalar(0, 255, 0), 1);

    if (dets.empty()) {
        /* No detections — show "No <target> detected" at bottom */
        char no_det[128];
        snprintf(no_det, sizeof(no_det), "No %s detected", get_display_label());
        int baseline = 0;
        cv::Size tsz = cv::getTextSize(no_det, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
        cv::putText(frame, no_det, cv::Point((fw - tsz.width) / 2, fh - 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 200), 1);
        return;
    }

    cv::Scalar box_color(0, 255, 0);         /* green boxes (BGR) */
    cv::Scalar xhair_color(102, 255, 0);     /* green crosshair (BGR) */
    cv::Scalar clip_color(0, 77, 255);       /* orange CLIPPED (BGR) */

    /* Find closest detection and draw all boxes */
    int best_idx = 0;
    float best_dist = 999.0f;

    for (size_t i = 0; i < dets.size(); i++) {
        const auto& d = dets[i];
        int bx1 = std::max(0, (int)d.x1);
        int by1 = std::max(0, (int)d.y1);
        int bx2 = std::min(fw - 1, (int)d.x2);
        int by2 = std::min(fh - 1, (int)d.y2);

        cv::rectangle(frame, cv::Point(bx1, by1), cv::Point(bx2, by2), box_color, 2);

        /* Confidence label */
        char label[64];
        snprintf(label, sizeof(label), "%s %.0f%%", get_display_label(), d.confidence * 100.0f);
        cv::putText(frame, label, cv::Point(bx1, by1 - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1);

        /* Distance + angle label below box */
        float dist, angle;
        estimate_distance(d.x1, d.y1, d.x2, d.y2, fw, fh,
                          DEFAULT_REF_HEIGHT, DEFAULT_REF_DISTANCE, DEFAULT_FOV,
                          dist, angle);
        char dist_label[64];
        snprintf(dist_label, sizeof(dist_label), "%.2fm %.0fdeg",
                 dist, angle * 180.0f / (float)M_PI);
        cv::putText(frame, dist_label, cv::Point(bx1, by2 + 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1);

        if (dist < best_dist) {
            best_dist = dist;
            best_idx = (int)i;
        }
    }

    /* Crosshair on closest detection */
    const auto& best = dets[best_idx];
    float best_distance, best_angle;
    estimate_distance(best.x1, best.y1, best.x2, best.y2, fw, fh,
                      DEFAULT_REF_HEIGHT, DEFAULT_REF_DISTANCE, DEFAULT_FOV,
                      best_distance, best_angle);

    int cx = (int)((best.x1 + best.x2) / 2.0f);
    int cy = (int)((best.y1 + best.y2) / 2.0f);
    int r = 18;

    cv::circle(frame, cv::Point(cx, cy), r, xhair_color, 2);
    cv::line(frame, cv::Point(cx - r - 6, cy), cv::Point(cx - r + 6, cy), xhair_color, 2);
    cv::line(frame, cv::Point(cx + r - 6, cy), cv::Point(cx + r + 6, cy), xhair_color, 2);
    cv::line(frame, cv::Point(cx, cy - r - 6), cv::Point(cx, cy - r + 6), xhair_color, 2);
    cv::line(frame, cv::Point(cx, cy + r - 6), cv::Point(cx, cy + r + 6), xhair_color, 2);

    /* CLIPPED indicator */
    if ((int)best.y1 <= 5 || (int)best.y2 >= fh - 5) {
        cv::putText(frame, "CLIPPED", cv::Point(cx + r + 8, cy + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, clip_color, 1);
    }
}

/* Extended JSON output — includes distance and angle for each detection */
static void write_json_ext(const std::vector<Detection>& dets, float infer_ms,
                            int frame_w, int frame_h,
                            float ref_height, float ref_dist, float fov) {
    printf("{\"detections\":[");
    for (size_t i = 0; i < dets.size(); i++) {
        const auto& d = dets[i];
        float dist, angle;
        estimate_distance(d.x1, d.y1, d.x2, d.y2, frame_w, frame_h,
                          ref_height, ref_dist, fov, dist, angle);
        if (i > 0) printf(",");
        printf("{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,"
               "\"confidence\":%.4f,\"class_id\":%d,\"class_name\":\"%s\","
               "\"distance\":%.4f,\"angle\":%.4f}",
               (int)d.x1, (int)d.y1, (int)d.x2, (int)d.y2,
               d.confidence, d.class_id,
               get_class_name(d.class_id),
               dist, angle);
    }
    printf("],\"inference_ms\":%.1f}\n", infer_ms);
    fflush(stdout);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Stream Mode
 *
 * Like camera mode but for GUI integration:
 *   - C++ owns camera (multi-threaded capture + inference)
 *   - Frames with overlays → POSIX shared memory (Python just displays)
 *   - Detections with distance → stdout JSON (for navigation)
 *   - No Wayland display needed
 *
 * Shared memory layout (24-byte header + raw BGR with overlays):
 *   [uint32 width][uint32 height][uint64 sequence][uint64 timestamp_us][BGR data]
 *
 * This is ~10x faster than pipe mode because:
 *   - No 921KB frame transfer per detection cycle
 *   - Capture + inference run in parallel threads
 *   - GUI reads frames via zero-copy mmap
 *   - ALL image processing (boxes, crosshair, labels) done in C++ on DRP-AI board
 * ═══════════════════════════════════════════════════════════════════════ */
#define SHM_NAME "/v2n_camera"
#define SHM_HEADER_SIZE 24

/* Detection results shared memory — replaces JSON stdout in stream mode */
#define DET_SHM_NAME "/v2n_detections"
#define MAX_SHM_DETS 20

/* Detection shared memory layout:
 *   [uint32 n_detections]     offset 0
 *   [uint32 frame_seq]        offset 4
 *   [float  inference_ms]     offset 8
 *   [float  ref_box_height]   offset 12  (echoes calibration for verification)
 *   [float  ref_distance]     offset 16
 *   [float  camera_fov]       offset 20
 *   [24 bytes padding]        offset 24-47
 *   Detection[0..19]:         offset 48+  (each 48 bytes)
 *     [float x1, y1, x2, y2] 16 bytes
 *     [float confidence]      4 bytes
 *     [float distance]        4 bytes
 *     [float angle]           4 bytes
 *     [int32 class_id]        4 bytes
 *     [uint8 bbox_clipped]    1 byte
 *     [15 bytes padding]      align to 48 bytes
 */
#define DET_HEADER_SIZE 48
#define DET_ENTRY_SIZE  48
#define DET_SHM_SIZE    (DET_HEADER_SIZE + MAX_SHM_DETS * DET_ENTRY_SIZE)

/* Calibration input shared memory — written by Python GUI */
#define CAL_SHM_NAME "/v2n_calibration"

/* Calibration shared memory layout:
 *   [float ref_box_height]    offset 0
 *   [float ref_distance]      offset 4
 *   [float camera_fov]        offset 8
 *   [uint32 update_seq]       offset 12  (incremented by Python when changed)
 */
#define CAL_SHM_SIZE 16

static int run_stream_mode(DRPAIInference& drpai) {
    /* Open camera */
    CameraCapture camera;
    if (!camera.open(g_cfg.camera_device, g_cfg.camera_width, g_cfg.camera_height)) {
        fprintf(stderr, "[stream] ERROR: Cannot open camera %s\n",
                g_cfg.camera_device.c_str());
        return 1;
    }
    if (g_cfg.warmup_frames > 0) {
        camera.warmup(g_cfg.warmup_frames);
    }

    /* DRP-AI warmup (first inference is always slower) */
    {
        std::vector<float> dummy(INPUT_C * INPUT_H * INPUT_W, 0.0f);
        float warmup_ms = 0;
        detect(drpai, dummy, g_cfg.camera_width, g_cfg.camera_height, warmup_ms);
        fprintf(stderr, "[stream] Warmup: %.1f ms\n", warmup_ms);
        g_first_frame = true;
    }

    /* Create shared memory for camera frames */
    const size_t frame_bytes = (size_t)g_cfg.camera_width * g_cfg.camera_height * 3;
    const size_t shm_size = SHM_HEADER_SIZE + frame_bytes;

    shm_unlink(SHM_NAME);
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        fprintf(stderr, "[stream] ERROR: shm_open: %s\n", strerror(errno));
        return 1;
    }
    if (ftruncate(shm_fd, shm_size) < 0) {
        fprintf(stderr, "[stream] ERROR: ftruncate: %s\n", strerror(errno));
        close(shm_fd);
        return 1;
    }
    uint8_t* shm = (uint8_t*)mmap(nullptr, shm_size,
                                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm == MAP_FAILED) {
        fprintf(stderr, "[stream] ERROR: mmap: %s\n", strerror(errno));
        close(shm_fd);
        return 1;
    }
    memset(shm, 0, shm_size);
    close(shm_fd);

    /* Create detection results shared memory */
    shm_unlink(DET_SHM_NAME);
    int det_shm_fd = shm_open(DET_SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (det_shm_fd < 0) {
        fprintf(stderr, "[stream] ERROR: det shm_open: %s\n", strerror(errno));
        munmap(shm, shm_size);
        shm_unlink(SHM_NAME);
        return 1;
    }
    ftruncate(det_shm_fd, DET_SHM_SIZE);
    uint8_t* det_shm = (uint8_t*)mmap(nullptr, DET_SHM_SIZE,
        PROT_READ | PROT_WRITE, MAP_SHARED, det_shm_fd, 0);
    if (det_shm == MAP_FAILED) {
        fprintf(stderr, "[stream] ERROR: det mmap: %s\n", strerror(errno));
        close(det_shm_fd);
        munmap(shm, shm_size);
        shm_unlink(SHM_NAME);
        return 1;
    }
    memset(det_shm, 0, DET_SHM_SIZE);
    close(det_shm_fd);

    /* Open calibration shared memory (created by Python, may not exist yet) */
    uint8_t* cal_shm = nullptr;
    int cal_shm_fd = shm_open(CAL_SHM_NAME, O_RDONLY, 0666);
    if (cal_shm_fd >= 0) {
        cal_shm = (uint8_t*)mmap(nullptr, CAL_SHM_SIZE, PROT_READ, MAP_SHARED, cal_shm_fd, 0);
        if (cal_shm == MAP_FAILED) cal_shm = nullptr;
        close(cal_shm_fd);
        fprintf(stderr, "[stream] Calibration shm attached\n");
    } else {
        fprintf(stderr, "[stream] Calibration shm not found (using defaults)\n");
    }

    /* Live calibration values (updated from shm or defaults) */
    float cal_ref_height = DEFAULT_REF_HEIGHT;
    float cal_ref_distance = DEFAULT_REF_DISTANCE;
    float cal_fov = DEFAULT_FOV;
    uint32_t cal_last_seq = 0;

    fprintf(stderr, "[stream] Camera %dx%d, shm %zu bytes at " SHM_NAME "\n",
            g_cfg.camera_width, g_cfg.camera_height, shm_size);

    printf("READY\n");
    fflush(stdout);

    /* Mailbox: capture → inference (single-slot, latest-frame-wins) */
    std::atomic<bool> infer_flag(false);
    cv::Mat infer_mailbox;

    /* Shared detection results for JSON output */
    std::mutex det_mutex;
    std::vector<Detection> det_results;
    float det_ms = 0;
    std::atomic<bool> det_new(false);

    /* Overlay cache: inference → capture thread (for drawing on frames) */
    std::mutex overlay_mutex;
    std::vector<Detection> overlay_dets;
    float overlay_ms = 0;

    uint64_t shm_seq = 0;

    /* Capture thread: camera → overlay → shm + inference mailbox */
    std::thread capture([&]() {
        int fail_count = 0;
        while (g_running.load()) {
            cv::Mat frame;
            if (!camera.read(frame)) {
                fail_count++;
                if (fail_count > 30) {
                    fprintf(stderr, "[stream] Camera lost.\n");
                    g_running.store(false);
                    break;
                }
                usleep(g_cfg.wait_time);
                continue;
            }
            fail_count = 0;

            /* Feed inference FIRST with raw frame (no overlays) */
            if (!infer_flag.load(std::memory_order_acquire)) {
                infer_mailbox = frame.clone();
                infer_flag.store(true, std::memory_order_release);
            }

            /* No C++ overlays on SHM frames — Python draws its own overlays
             * only for detections that pass the target shape filter.
             * This prevents false positives (chair, table) from showing
             * green boxes on the GUI. */

            /* Write raw frame to shared memory */
            shm_seq++;
            uint32_t w = (uint32_t)frame.cols;
            uint32_t h = (uint32_t)frame.rows;
            auto now = std::chrono::steady_clock::now();
            uint64_t ts = (uint64_t)std::chrono::duration_cast<
                std::chrono::microseconds>(now.time_since_epoch()).count();

            memcpy(shm + 0,  &w,    4);
            memcpy(shm + 4,  &h,    4);
            memcpy(shm + 16, &ts,   8);
            memcpy(shm + SHM_HEADER_SIZE, frame.data, frame_bytes);
            __sync_synchronize();
            memcpy(shm + 8,  &shm_seq, 8);  /* Sequence last = commit */
        }
    });

    /* Inference thread: preprocess + DRP-AI + post-process */
    std::thread inference([&]() {
        std::vector<float> input;
        while (g_running.load()) {
            if (!infer_flag.load(std::memory_order_acquire)) {
                usleep(g_cfg.wait_time);
                continue;
            }

            cv::Mat frame = infer_mailbox;
            infer_flag.store(false, std::memory_order_release);

            preprocess(frame, input);

            float ms = 0;
            auto dets = detect(drpai, input, frame.cols, frame.rows, ms);

            /* Update overlay cache for capture thread drawing */
            {
                std::lock_guard<std::mutex> lock(overlay_mutex);
                overlay_dets = dets;
                overlay_ms = ms;
            }

            {
                std::lock_guard<std::mutex> lock(det_mutex);
                det_results = std::move(dets);
                det_ms = ms;
                det_new.store(true, std::memory_order_release);
            }
        }
    });

    /* Main thread: write JSON detections (with distance) to stdout */
    auto fps_log_start = std::chrono::steady_clock::now();
    int infer_count = 0;

    while (g_running.load()) {
        if (!det_new.load(std::memory_order_acquire)) {
            usleep(g_cfg.wait_time);
            continue;
        }

        std::vector<Detection> dets;
        float ms;
        {
            std::lock_guard<std::mutex> lock(det_mutex);
            dets = det_results;
            ms = det_ms;
            det_new.store(false, std::memory_order_relaxed);
        }

        /* Read calibration from shared memory (if available) */
        if (cal_shm != nullptr) {
            uint32_t cal_seq;
            memcpy(&cal_seq, cal_shm + 12, 4);
            if (cal_seq != cal_last_seq) {
                memcpy(&cal_ref_height, cal_shm + 0, 4);
                memcpy(&cal_ref_distance, cal_shm + 4, 4);
                memcpy(&cal_fov, cal_shm + 8, 4);
                cal_last_seq = cal_seq;
                /* Validate */
                if (cal_ref_height < 1.0f) cal_ref_height = DEFAULT_REF_HEIGHT;
                if (cal_ref_distance < 0.01f) cal_ref_distance = DEFAULT_REF_DISTANCE;
                if (cal_fov < 10.0f || cal_fov > 180.0f) cal_fov = DEFAULT_FOV;
                fprintf(stderr, "[stream] Calibration updated: ref=%.0fpx@%.2fm FOV=%.0f\n",
                        cal_ref_height, cal_ref_distance, cal_fov);
            }
        }

        /* Write detections to shared memory (binary struct) */
        {
            uint32_t n = (uint32_t)std::min((int)dets.size(), MAX_SHM_DETS);
            float infer_ms_f = ms;

            for (uint32_t i = 0; i < n; i++) {
                const auto& d = dets[i];
                float dist, angle;
                estimate_distance(d.x1, d.y1, d.x2, d.y2,
                                  g_cfg.camera_width, g_cfg.camera_height,
                                  cal_ref_height, cal_ref_distance, cal_fov,
                                  dist, angle);

                uint8_t* entry = det_shm + DET_HEADER_SIZE + i * DET_ENTRY_SIZE;
                float x1f = d.x1, y1f = d.y1, x2f = d.x2, y2f = d.y2;
                float conf = d.confidence;
                int32_t cls = d.class_id;
                uint8_t clipped = (d.y1 <= 5 || d.y2 >= g_cfg.camera_height - 5) ? 1 : 0;

                memcpy(entry + 0,  &x1f,    4);
                memcpy(entry + 4,  &y1f,    4);
                memcpy(entry + 8,  &x2f,    4);
                memcpy(entry + 12, &y2f,    4);
                memcpy(entry + 16, &conf,   4);
                memcpy(entry + 20, &dist,   4);
                memcpy(entry + 24, &angle,  4);
                memcpy(entry + 28, &cls,    4);
                memcpy(entry + 32, &clipped, 1);
            }

            /* Write header LAST (commit pattern — reader checks n_detections first) */
            memcpy(det_shm + 8,  &infer_ms_f, 4);
            memcpy(det_shm + 12, &cal_ref_height, 4);
            memcpy(det_shm + 16, &cal_ref_distance, 4);
            memcpy(det_shm + 20, &cal_fov, 4);
            __sync_synchronize();
            uint32_t seq32 = (uint32_t)(shm_seq & 0xFFFFFFFF);
            memcpy(det_shm + 4, &seq32, 4);
            memcpy(det_shm + 0, &n, 4);
        }

        /* JSON output (kept for pipe mode compatibility and debugging) */
        write_json_ext(dets, ms, g_cfg.camera_width, g_cfg.camera_height,
                       cal_ref_height, cal_ref_distance, cal_fov);
        infer_count++;

        /* Log stats every 2 seconds */
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - fps_log_start).count();
        if (elapsed >= 2.0f) {
            fprintf(stderr, "[stream] seq=%lu  infer=%.0f FPS (%.1f ms)  dets=%zu\n",
                    (unsigned long)shm_seq, infer_count / elapsed, ms, dets.size());
            infer_count = 0;
            fps_log_start = now;
        }
    }

    capture.join();
    inference.join();

    munmap(shm, shm_size);
    shm_unlink(SHM_NAME);
    munmap(det_shm, DET_SHM_SIZE);
    shm_unlink(DET_SHM_NAME);
    if (cal_shm) munmap((void*)cal_shm, CAL_SHM_SIZE);

    fprintf(stderr, "[stream] Done.\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Multi-Threaded Camera Pipeline (Renesas reference pattern)
 *
 * Architecture:
 *   Capture thread  → reads camera continuously at full FPS
 *   Inference thread → runs DRP-AI preprocessing + inference asynchronously
 *   Main thread      → draws detections and pushes to display
 *
 * Synchronization:
 *   - Atomic flags for single-slot mailbox (latest-frame-wins, no queue)
 *   - Mutex only for shared detection results vector
 *   - usleep(wait_time) polling in consumer threads
 * ═══════════════════════════════════════════════════════════════════════ */

/* Atomic flags — single-slot mailbox pattern (Renesas reference) */
static std::atomic<bool> g_inference_ready(false);  /* capture → inference */
static std::atomic<bool> g_display_ready(false);    /* capture → display  */

/* Shared frame buffers — written by capture thread only when flag is false */
static cv::Mat g_inference_frame;   /* Latest frame for inference thread */
static cv::Mat g_display_frame;     /* Latest frame for display thread  */

/* Shared detection results — protected by mutex */
static std::mutex g_det_mutex;
static std::vector<Detection> g_detections;
static float g_infer_ms = 0;

/* FPS counters (lock-free) */
static std::atomic<float> g_camera_fps(0);
static std::atomic<float> g_inference_fps(0);

/* ── Capture Thread ─────────────────────────────────────────────────── */
static void capture_thread_func(CameraCapture& camera) {
    int frame_count = 0;
    int fail_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (g_running.load()) {
        cv::Mat frame;
        if (!camera.read(frame)) {
            fail_count++;
            if (fail_count > 30) {
                fprintf(stderr, "ERROR: Camera read failed %d consecutive times.\n", fail_count);
                g_running.store(false);
                break;
            }
            usleep(g_cfg.wait_time);
            continue;
        }
        fail_count = 0;

        /* Feed inference thread (latest-frame-wins: drop if busy) */
        if (!g_inference_ready.load()) {
            g_inference_frame = frame.clone();
            g_inference_ready.store(true);
        }

        /* Feed display thread (latest-frame-wins: drop if busy) */
        if (!g_display_ready.load()) {
            g_display_frame = frame.clone();
            g_display_ready.store(true);
        }

        /* Camera FPS tracking */
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - fps_start).count();
        if (elapsed >= 1.0f) {
            g_camera_fps.store(frame_count / elapsed);
            frame_count = 0;
            fps_start = now;
        }
    }
}

/* ── Inference Thread ───────────────────────────────────────────────── */
static void inference_thread_func(DRPAIInference& drpai) {
    std::vector<float> input;
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (g_running.load()) {
        /* Poll for new frame */
        if (!g_inference_ready.load()) {
            usleep(g_cfg.wait_time);
            continue;
        }

        /* Grab frame and release flag immediately */
        cv::Mat frame = g_inference_frame;  /* shallow copy (fast) */
        g_inference_ready.store(false);

        /* Preprocess + inference */
        preprocess(frame, input);
        float ms = 0;
        auto dets = detect(drpai, input, frame.cols, frame.rows, ms);

        /* Publish results under mutex */
        {
            std::lock_guard<std::mutex> lock(g_det_mutex);
            g_detections = std::move(dets);
            g_infer_ms = ms;
        }

        /* Inference FPS tracking */
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - fps_start).count();
        if (elapsed >= 1.0f) {
            g_inference_fps.store(frame_count / elapsed);
            frame_count = 0;
            fps_start = now;
        }
    }
}

/* ── Camera Mode (Main Thread = Display Loop) ───────────────────────── */
static int run_camera_mode(DRPAIInference& drpai) {
    CameraCapture camera;
    if (!camera.open(g_cfg.camera_device, g_cfg.camera_width, g_cfg.camera_height)) {
        fprintf(stderr, "ERROR: Cannot open camera: %s\n", g_cfg.camera_device.c_str());
        return 1;
    }

    /* Discard first frames — USB cameras need time for auto-exposure */
    if (g_cfg.warmup_frames > 0) {
        fprintf(stderr, "Camera warmup: discarding %d frames...\n", g_cfg.warmup_frames);
        camera.warmup(g_cfg.warmup_frames);
    }

    Display display;
    display.init(g_cfg.camera_width, g_cfg.camera_height, g_cfg.use_wayland);

    /* DRP-AI warmup: first inference is always slower (DMA setup, kernel compile) */
    {
        fprintf(stderr, "Inference warmup...\n");
        std::vector<float> dummy(INPUT_C * INPUT_H * INPUT_W, 0.0f);
        float warmup_ms = 0;
        detect(drpai, dummy, g_cfg.camera_width, g_cfg.camera_height, warmup_ms);
        fprintf(stderr, "  Warmup: %.1f ms (first run, expect slower)\n", warmup_ms);
        g_first_frame = true;  /* Reset so debug dump fires on first REAL frame */
    }

    fprintf(stderr, "\nDetection running (multi-threaded, poll=%d us). Press Ctrl+C to stop.\n\n",
            g_cfg.wait_time);

    /* Launch capture + inference threads */
    std::thread cap_thread(capture_thread_func, std::ref(camera));
    std::thread inf_thread(inference_thread_func, std::ref(drpai));

    /* Main thread = display loop */
    auto fps_log_start = std::chrono::steady_clock::now();

    while (g_running.load()) {
        if (!g_display_ready.load()) {
            usleep(g_cfg.wait_time);
            continue;
        }

        cv::Mat frame = g_display_frame;  /* shallow copy */
        g_display_ready.store(false);

        /* Get latest detections (may be from a previous frame — that's fine) */
        std::vector<Detection> dets;
        float infer_ms;
        {
            std::lock_guard<std::mutex> lock(g_det_mutex);
            dets = g_detections;
            infer_ms = g_infer_ms;
        }

        /* Draw + display */
        display.show(frame, dets, infer_ms,
                     g_camera_fps.load(), g_inference_fps.load());

        /* Log FPS to stderr every 2 seconds */
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - fps_log_start).count();
        if (elapsed >= 2.0f) {
            fprintf(stderr, "Camera: %.1f FPS | Inference: %.1f FPS (%.1f ms) | Dets: %zu\n",
                    g_camera_fps.load(), g_inference_fps.load(), infer_ms, dets.size());
            fps_log_start = now;
        }
    }

    /* Graceful shutdown — signal threads and wait */
    cap_thread.join();
    inf_thread.join();

    fprintf(stderr, "\nDone.\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Help
 * ═══════════════════════════════════════════════════════════════════════ */
static void print_help(const char* prog) {
    char drp_str[48], aimac_str[48];
    format_drp_freq(g_cfg.drp_max_freq, drp_str, sizeof(drp_str));
    format_aimac_freq(g_cfg.drpai_freq, aimac_str, sizeof(aimac_str));

    fprintf(stderr,
        "%s DRP-AI Detection — Renesas RZ/V2N\n\n",
        MODEL_NAME);
    fprintf(stderr,
        "Usage: %s [options] [model_dir] [camera_device]\n\n"
        "Options:\n"
        "  --console      Console output (no Wayland)\n"
        "  --display      Force Wayland display\n"
        "  --pipe         Pipe mode (stdin BGR → stdout JSON)\n"
        "  --stream       Stream mode (camera + DRP-AI → shm frames + stdout JSON)\n"
        "  --help, -h     Show this help\n\n"
        "Current configuration:\n"
        "  Model:       %s/%s\n"
        "  Classes:     %d\n"
        "  Input:       %dx%d (%d anchors)\n"
        "  Camera:      %s (%dx%d)\n"
        "  Thresholds:  conf=%.2f  nms=%.2f  max=%d\n"
        "  DRP core:    %s\n"
        "  AI-MAC:      %s\n"
        "  Display:     %s\n"
        "  Thread poll: %d us\n\n"
        "Config file: place config.ini next to binary to override any parameter.\n",
        prog,
        g_cfg.model_dir.c_str(), MODEL_DIR,
        NUM_CLASSES,
        INPUT_W, INPUT_H, NUM_ANCHORS,
        g_cfg.camera_device.c_str(), g_cfg.camera_width, g_cfg.camera_height,
        g_cfg.conf_threshold, g_cfg.nms_threshold, g_cfg.max_detections,
        drp_str, aimac_str,
        g_cfg.use_wayland ? "wayland" : "console",
        g_cfg.wait_time);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Main
 * ═══════════════════════════════════════════════════════════════════════ */
int main(int argc, char* argv[]) {
    /* 1. Load config.ini — overrides compile-time defaults */
    int cfg_count = load_config("config.ini");

    /* 1b. Load labels.txt (if not already set by config.ini class_names) */
    if (g_cfg.class_names.empty()) {
        int n = load_labels("labels.txt");
        if (n > 0)
            fprintf(stderr, "  Loaded %d class name(s) from labels.txt\n", n);
    }

    /* 2. Parse CLI arguments — overrides config.ini */
    bool pipe_mode = false;
    bool stream_mode = false;
    bool cli_model = false, cli_camera = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--pipe") == 0) {
            pipe_mode = true;
        } else if (strcmp(argv[i], "--stream") == 0) {
            stream_mode = true;
        } else if (strcmp(argv[i], "--console") == 0) {
            g_cfg.use_wayland = false;
        } else if (strcmp(argv[i], "--display") == 0) {
            g_cfg.use_wayland = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (!cli_model) {
            g_cfg.model_dir = argv[i];
            cli_model = true;
        } else if (!cli_camera) {
            g_cfg.camera_device = argv[i];
            cli_camera = true;
        }
    }

    /* 3. Startup banner */
    char drp_str[48], aimac_str[48];
    format_drp_freq(g_cfg.drp_max_freq, drp_str, sizeof(drp_str));
    format_aimac_freq(g_cfg.drpai_freq, aimac_str, sizeof(aimac_str));

    fprintf(stderr, "════════════════════════════════════════════\n");
    fprintf(stderr, "  %s DRP-AI Detection — RZ/V2N\n", MODEL_NAME);
    fprintf(stderr, "════════════════════════════════════════════\n");
    fprintf(stderr, "Mode:       %s\n",
            pipe_mode ? "PIPE" : stream_mode ? "STREAM" :
            (g_cfg.use_wayland ? "CAMERA+WAYLAND" : "CAMERA+CONSOLE"));
    fprintf(stderr, "Model:      %s/%s (%s)\n", g_cfg.model_dir.c_str(), MODEL_DIR, MODEL_NAME);
    fprintf(stderr, "Classes:    %d", NUM_CLASSES);
    for (int i = 0; i < NUM_CLASSES && i < 10; i++)
        fprintf(stderr, "%s%s", (i == 0) ? " (" : ", ", get_class_name(i));
    if (NUM_CLASSES > 10) fprintf(stderr, ", ...");
    if (NUM_CLASSES > 0)  fprintf(stderr, ")");
    fprintf(stderr, "\n");
#if OUTPUT_FORMAT == 1
    fprintf(stderr, "Output:     cut heads (%d tensors, DFL decode on CPU)\n", NUM_OUTPUTS);
#else
    fprintf(stderr, "Output:     decoded [1, %d, %d]\n", 4 + NUM_CLASSES, NUM_ANCHORS);
#endif
    fprintf(stderr, "Input:      %dx%d (%d anchors)\n", INPUT_W, INPUT_H, NUM_ANCHORS);
    fprintf(stderr, "Thresholds: conf=%.2f  nms=%.2f  max=%d\n",
            g_cfg.conf_threshold, g_cfg.nms_threshold, g_cfg.max_detections);
    fprintf(stderr, "DRP core:   %s\n", drp_str);
    fprintf(stderr, "AI-MAC:     %s\n", aimac_str);
    if (!pipe_mode) {
        fprintf(stderr, "Camera:     %s (%dx%d)\n",
                g_cfg.camera_device.c_str(), g_cfg.camera_width, g_cfg.camera_height);
        if (!stream_mode)
            fprintf(stderr, "Display:    %s\n", g_cfg.use_wayland ? "wayland" : "console");
    }
    if (cfg_count > 0) {
        fprintf(stderr, "Config:     %d override(s) from config.ini\n", cfg_count);
    }
    fprintf(stderr, "────────────────────────────────────────────\n\n");

    /* 4. Signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 5. V2N OCA workaround (Renesas pattern: prevent DRP-AI / OpenCV contention) */
#ifdef V2N
    {
        unsigned long OCA_list[16] = {0};
        OCA_Activate(&OCA_list[0]);
        fprintf(stderr, "V2N OCA workaround applied.\n");
    }
#endif

    /* 6. Initialize DRP-AI inference with runtime frequency settings */
    DRPAIInference drpai;
    std::string model_path = g_cfg.model_dir + "/" + MODEL_DIR;

    if (!drpai.load(model_path, g_cfg.drp_max_freq, g_cfg.drpai_freq)) {
        fprintf(stderr, "ERROR: Failed to load model: %s\n", model_path.c_str());
        return 1;
    }
    fprintf(stderr, "Model loaded.\n\n");

    /* 7. Run */
    if (pipe_mode) {
        return run_pipe_mode(drpai);
    } else if (stream_mode) {
        return run_stream_mode(drpai);
    } else {
        return run_camera_mode(drpai);
    }
}
