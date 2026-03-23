#ifndef DRPAI_INFERENCE_H
#define DRPAI_INFERENCE_H

/*
 * DRP-AI Inference Wrapper
 * Uses MeraDrpRuntimeWrapper (Renesas MERA2 runtime) for RZ/V2N
 *
 * Opens /dev/drpai0, sets DRP frequency at runtime, then loads model.
 * Frequency parameters are passed in load() — not hardcoded from define.h.
 *
 * Supports both single-output (decoded) and multi-output (cut heads) models.
 */

#include <string>
#include <vector>
#include "MeraDrpRuntimeWrapper.h"

class DRPAIInference {
public:
    DRPAIInference();
    ~DRPAIInference();

    /*
     * Load model from directory (parent of sub_*__CPU_DRP_TVM).
     * drp_freq:   DRP core frequency index (2=420MHz, 127=9.8MHz)
     * drpai_freq: AI-MAC frequency index (1=1GHz, 5=315MHz, 127=10MHz)
     */
    bool load(const std::string& model_dir, int drp_freq, int drpai_freq);

    /* Run inference on preprocessed input (CHW float32, [0,1]) */
    bool run(const float* input_data, int input_size);

    /* Get output tensor by index. Returns pointer and sets element count.
     * For decoded models: idx=0 returns [1, 4+NC, anchors]
     * For cut heads: idx=0..5 returns individual cv2/cv3 tensors */
    float* get_output(int idx, int& output_size);

    /* Number of output buffers the model has */
    int get_num_outputs() const { return m_num_outputs; }

    bool is_loaded() const { return m_loaded; }

private:
    bool open_drpai_device();
    uint64_t get_drpai_start_addr();
    bool set_drp_frequency(int drp_freq, int drpai_freq);

    MeraDrpRuntimeWrapper m_runtime;
    std::vector<std::vector<float>> m_output_bufs;
    int m_num_outputs;
    bool m_loaded;
    int m_drpai_fd;
};

#endif /* DRPAI_INFERENCE_H */
