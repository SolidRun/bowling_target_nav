/*
 * DRP-AI Inference Implementation
 * Uses MeraDrpRuntimeWrapper (Renesas MERA2 runtime)
 *
 * Opens /dev/drpai0, gets DRP-AI memory start address via ioctl,
 * sets DRP frequency from runtime parameters, then loads model.
 *
 * Supports both single-output (decoded) and multi-output (cut heads) models.
 */

#include "drpai_inference.h"
#include "define.h"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/drpai.h>

DRPAIInference::DRPAIInference() : m_num_outputs(0), m_loaded(false), m_drpai_fd(-1) {}

DRPAIInference::~DRPAIInference() {
    if (m_drpai_fd >= 0) {
        close(m_drpai_fd);
        m_drpai_fd = -1;
    }
}

bool DRPAIInference::open_drpai_device() {
    m_drpai_fd = open(DRPAI_DEV, O_RDWR);
    if (m_drpai_fd < 0) {
        std::cerr << "ERROR: Cannot open " << DRPAI_DEV << std::endl;
        std::cerr << "  Run as root: sudo ./<binary>" << std::endl;
        return false;
    }
    return true;
}

uint64_t DRPAIInference::get_drpai_start_addr() {
    drpai_data_t drpai_data;
    if (ioctl(m_drpai_fd, DRPAI_GET_DRPAI_AREA, &drpai_data) != 0) {
        std::cerr << "ERROR: ioctl DRPAI_GET_DRPAI_AREA failed" << std::endl;
        return 0;
    }
    return drpai_data.address;
}

bool DRPAIInference::set_drp_frequency(int drp_freq, int drpai_freq) {
    uint32_t freq;

    /* DRP core frequency */
    freq = static_cast<uint32_t>(drp_freq);
    if (ioctl(m_drpai_fd, DRPAI_SET_DRP_MAX_FREQ, &freq) != 0) {
        std::cerr << "WARNING: Failed to set DRP frequency (idx=" << drp_freq << ")" << std::endl;
    } else {
        fprintf(stderr, "DRP core freq:  idx=%d\n", drp_freq);
    }

    /* AI-MAC frequency */
    freq = static_cast<uint32_t>(drpai_freq);
    if (ioctl(m_drpai_fd, DRPAI_SET_DRPAI_FREQ, &freq) != 0) {
        std::cerr << "WARNING: Failed to set AI-MAC frequency (idx=" << drpai_freq << ")" << std::endl;
    } else {
        fprintf(stderr, "AI-MAC freq:    idx=%d\n", drpai_freq);
    }

    return true;
}

bool DRPAIInference::load(const std::string& model_dir, int drp_freq, int drpai_freq) {
    try {
        /* Step 1: Open DRP-AI device */
        if (!open_drpai_device()) {
            return false;
        }

        /* Step 2: Get DRP-AI memory start address */
        uint64_t addr = get_drpai_start_addr();
        if (addr == 0) {
            std::cerr << "ERROR: DRP-AI memory address is 0" << std::endl;
            return false;
        }
        fprintf(stderr, "DRP-AI base:    0x%lx\n", (unsigned long)addr);

        /* Step 3: Set DRP frequency from runtime parameters */
        set_drp_frequency(drp_freq, drpai_freq);

        /* Step 4: Load model */
        if (!m_runtime.LoadModel(model_dir, addr)) {
            std::cerr << "ERROR: LoadModel failed: " << model_dir << std::endl;
            return false;
        }

        /* Step 5: Detect number of output buffers */
        m_num_outputs = NUM_OUTPUTS;
        m_output_bufs.resize(m_num_outputs);
        fprintf(stderr, "Output buffers: %d\n", m_num_outputs);

        m_loaded = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return false;
    }
}

bool DRPAIInference::run(const float* input_data, int input_size) {
    if (!m_loaded) return false;

    try {
        m_runtime.SetInput(0, input_data);
        m_runtime.Run();
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Inference error: " << e.what() << std::endl;
        return false;
    }
}

float* DRPAIInference::get_output(int idx, int& output_size) {
    if (idx < 0 || idx >= m_num_outputs) {
        output_size = 0;
        return nullptr;
    }

    try {
        auto [dtype, data_ptr, size] = m_runtime.GetOutput(idx);
        output_size = static_cast<int>(size);

        if ((int)m_output_bufs[idx].size() < output_size) {
            m_output_bufs[idx].resize(output_size);
        }

        if (dtype == InOutDataType::FLOAT32) {
            std::memcpy(m_output_bufs[idx].data(), data_ptr, output_size * sizeof(float));

        } else if (dtype == InOutDataType::FLOAT16) {
            /* Convert float16 → float32 (IEEE 754) */
            const uint16_t* fp16 = static_cast<const uint16_t*>(data_ptr);
            for (int i = 0; i < output_size; i++) {
                uint32_t h = fp16[i];
                uint32_t sign = (h & 0x8000) << 16;
                uint32_t exp  = (h >> 10) & 0x1F;
                uint32_t mant = h & 0x3FF;
                uint32_t f;

                if (exp == 0)       f = sign;                                      /* zero / subnormal */
                else if (exp == 31) f = sign | 0x7F800000 | (mant << 13);         /* inf / nan */
                else                f = sign | ((exp + 112) << 23) | (mant << 13);

                std::memcpy(&m_output_bufs[idx][i], &f, sizeof(float));
            }
        } else {
            std::cerr << "WARNING: Unsupported output dtype for buffer " << idx << std::endl;
            output_size = 0;
            return nullptr;
        }

        return m_output_bufs[idx].data();

    } catch (const std::exception& e) {
        std::cerr << "Output error (idx=" << idx << "): " << e.what() << std::endl;
        output_size = 0;
        return nullptr;
    }
}
