/*
 * Display Implementation
 *
 * Console mode: prints detections to stdout.
 * Wayland mode: draws bounding boxes on frame, then pushes to
 *               GStreamer waylandsink via cv::VideoWriter.
 *               This avoids cv::imshow (GTK backend) which fails on Weston.
 *
 * Display mode is selected at runtime (not compile-time) to support
 * the --console / --display CLI flags.
 */

#include "display.h"
#include "define.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

/* Box colors per class (cycle through) */
static const cv::Scalar BOX_COLORS[] = {
    cv::Scalar(0, 255, 0),    /* green */
    cv::Scalar(255, 0, 0),    /* blue */
    cv::Scalar(0, 0, 255),    /* red */
    cv::Scalar(255, 255, 0),  /* cyan */
    cv::Scalar(0, 255, 255),  /* yellow */
    cv::Scalar(255, 0, 255),  /* magenta */
    cv::Scalar(128, 255, 0),
    cv::Scalar(0, 128, 255),
};
static const int NUM_COLORS = sizeof(BOX_COLORS) / sizeof(BOX_COLORS[0]);

/*
 * Auto-detect Wayland environment on RZ/V2N.
 *
 * The Weston socket location varies per board setup:
 *   /run/user/0/wayland-0    (root session)
 *   /run/user/996/wayland-1  (weston-launch user)
 *   /run/wayland-0           (some board configs)
 *
 * Always detects — pre-set env vars may have wrong values
 * (e.g., WAYLAND_DISPLAY="/run/wayland-0" instead of "wayland-0").
 */
static bool find_wayland_in_dir(const char* search_dir,
                                char* runtime_dir, size_t rd_size,
                                char* display_name, size_t dn_size)
{
    DIR* dir = opendir(search_dir);
    if (!dir) return false;

    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "wayland-", 8) == 0 &&
            strchr(ent->d_name + 8, '.') == NULL) {
            char sock_path[512];
            snprintf(sock_path, sizeof(sock_path), "%s/%s", search_dir, ent->d_name);
            struct stat st;
            if (stat(sock_path, &st) == 0 && S_ISSOCK(st.st_mode)) {
                snprintf(runtime_dir, rd_size, "%s", search_dir);
                snprintf(display_name, dn_size, "%s", ent->d_name);
                closedir(dir);
                return true;
            }
        }
    }
    closedir(dir);
    return false;
}

static bool find_wayland_socket(char* runtime_dir, size_t rd_size,
                                char* display_name, size_t dn_size)
{
    /* Search /run/user/* first (standard location) */
    DIR* run_user = opendir("/run/user");
    if (run_user) {
        struct dirent* uid_ent;
        while ((uid_ent = readdir(run_user)) != NULL) {
            if (uid_ent->d_name[0] == '.') continue;
            char user_dir[256];
            snprintf(user_dir, sizeof(user_dir), "/run/user/%s", uid_ent->d_name);
            if (find_wayland_in_dir(user_dir, runtime_dir, rd_size,
                                     display_name, dn_size)) {
                closedir(run_user);
                return true;
            }
        }
        closedir(run_user);
    }

    /* Fallback: search /run/ and /tmp/ (some boards put socket here) */
    if (find_wayland_in_dir("/run", runtime_dir, rd_size, display_name, dn_size))
        return true;
    if (find_wayland_in_dir("/tmp", runtime_dir, rd_size, display_name, dn_size))
        return true;

    return false;
}

static void setup_wayland_env() {
    /*
     * Always detect — pre-set values may be wrong
     * (e.g., WAYLAND_DISPLAY="/run/wayland-0" instead of "wayland-0")
     */
    char runtime_dir[256] = {0};
    char display_name[64] = {0};

    if (find_wayland_socket(runtime_dir, sizeof(runtime_dir),
                            display_name, sizeof(display_name))) {
        setenv("XDG_RUNTIME_DIR", runtime_dir, 1);  /* overwrite */
        setenv("WAYLAND_DISPLAY", display_name, 1);
        printf("Wayland: %s/%s\n", runtime_dir, display_name);
    } else {
        /* Fallback: common defaults (only if not set) */
        if (!getenv("XDG_RUNTIME_DIR"))
            setenv("XDG_RUNTIME_DIR", "/run/user/0", 0);
        if (!getenv("WAYLAND_DISPLAY"))
            setenv("WAYLAND_DISPLAY", "wayland-0", 0);
        fprintf(stderr, "WARNING: No Wayland socket found, using defaults.\n");
    }
}

Display::Display() : m_width(0), m_height(0), m_frame_count(0),
                     m_use_wayland(false), m_writer_ok(false) {}

Display::~Display() {
    if (m_writer.isOpened()) {
        m_writer.release();
    }
}

void Display::init(int width, int height, bool use_wayland) {
    m_width = width;
    m_height = height;
    m_use_wayland = use_wayland;

    if (m_use_wayland) {
        /* Auto-detect Wayland compositor socket */
        setup_wayland_env();

        /*
         * GStreamer pipeline for waylandsink:
         *  - appsrc with is-live=true: OpenCV pushes BGR frames here
         *  - Explicit caps so GStreamer knows the frame format/size
         *  - videoconvert to BGRx: waylandsink on Weston needs 4-byte pixel format
         *  - sync=false: don't block on frame timestamps (live feed)
         */
        char pipeline[512];
        snprintf(pipeline, sizeof(pipeline),
            "appsrc is-live=true ! "
            "video/x-raw,format=BGR,width=%d,height=%d,framerate=30/1 ! "
            "videoconvert ! "
            "video/x-raw,format=BGRx ! "
            "waylandsink sync=false",
            width, height);

        m_writer.open(pipeline, cv::CAP_GSTREAMER, 0, 30.0,
                      cv::Size(width, height));

        if (m_writer.isOpened()) {
            m_writer_ok = true;
            printf("Display: Wayland %dx%d\n", width, height);
        } else {
            m_writer_ok = false;
            m_use_wayland = false;
            fprintf(stderr, "WARNING: Could not open GStreamer waylandsink.\n");
            fprintf(stderr, "  Falling back to console-only output.\n");
            fprintf(stderr, "  Check: Is Weston running? Try: systemctl status weston\n");
        }
    }
}

void Display::show(cv::Mat& frame, const std::vector<Detection>& detections,
                   float infer_ms, float camera_fps, float inference_fps) {
    m_frame_count++;

    if (!m_use_wayland) {
        /* Console mode: print detections */
        if (!detections.empty()) {
            for (const auto& det : detections) {
                const char* name = (det.class_id < NUM_CLASSES) ?
                    CLASS_NAMES[det.class_id] : "unknown";
                printf("[%d] %s: %.1f%% at (%.0f,%.0f)-(%.0f,%.0f)\n",
                       m_frame_count, name, det.confidence * 100.0f,
                       det.x1, det.y1, det.x2, det.y2);
            }
        }
    } else {
        /* Wayland mode: draw bounding boxes + push to waylandsink */
        for (const auto& det : detections) {
            cv::Scalar color = BOX_COLORS[det.class_id % NUM_COLORS];
            int x1 = std::max(0, (int)det.x1);
            int y1 = std::max(0, (int)det.y1);
            int x2 = std::min(frame.cols - 1, (int)det.x2);
            int y2 = std::min(frame.rows - 1, (int)det.y2);

            cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, BOX_LINE_SIZE);

            /* Label */
            const char* name = (det.class_id < NUM_CLASSES) ?
                CLASS_NAMES[det.class_id] : "unknown";
            char label[128];
            snprintf(label, sizeof(label), "%s %.0f%%", name, det.confidence * 100.0f);

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                                  CHAR_SCALE_SMALL, CHAR_THICKNESS, &baseline);
            cv::rectangle(frame,
                          cv::Point(x1, y1 - text_size.height - 6),
                          cv::Point(x1 + text_size.width + 4, y1),
                          color, cv::FILLED);
            cv::putText(frame, label, cv::Point(x1 + 2, y1 - 4),
                        cv::FONT_HERSHEY_SIMPLEX, CHAR_SCALE_SMALL, cv::Scalar(0, 0, 0), CHAR_THICKNESS);
        }

        /* FPS overlay — dual FPS in multi-threaded mode, fallback for single-threaded */
        char fps_text[128];
        if (camera_fps > 0 || inference_fps > 0) {
            snprintf(fps_text, sizeof(fps_text),
                     "Cam: %.0f FPS | AI: %.0f FPS (%.0f ms)",
                     camera_fps, inference_fps, infer_ms);
        } else {
            snprintf(fps_text, sizeof(fps_text), "Inference: %.1f ms", infer_ms);
        }
        cv::putText(frame, fps_text, cv::Point(10, LINE_HEIGHT_PX),
                    cv::FONT_HERSHEY_SIMPLEX, CHAR_SCALE_LARGE, cv::Scalar(0, 255, 0), CHAR_THICKNESS);

        /* Push frame to Wayland via GStreamer */
        if (m_writer_ok) {
            m_writer.write(frame);
        }

        /* Also print detections to console */
        if (!detections.empty()) {
            for (const auto& det : detections) {
                const char* name = (det.class_id < NUM_CLASSES) ?
                    CLASS_NAMES[det.class_id] : "unknown";
                printf("[%d] %s: %.1f%% at (%.0f,%.0f)-(%.0f,%.0f)\n",
                       m_frame_count, name, det.confidence * 100.0f,
                       det.x1, det.y1, det.x2, det.y2);
            }
        }
    }
}
