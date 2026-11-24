/*
 * ei_universal_mex.cpp
 * Universal MATLAB MEX Gateway for ANY Edge Impulse C++ Deployment Library
 *
 * Automatically adapts at compile time to any Edge Impulse project via
 * model-parameters/model_metadata.h:
 *   - Any number of input channels (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
 *   - Any window length (EI_CLASSIFIER_RAW_SAMPLE_COUNT)
 *   - Any sampling frequency (EI_CLASSIFIER_FREQUENCY)
 *   - Any number of output classes (EI_CLASSIFIER_LABEL_COUNT)
 *
 * Usage in MATLAB:
 *   info   = ei_universal_mex('info')      % Returns model metadata struct
 *   result = ei_universal_mex(signal_mat)  % Runs DSP + Neural Network inference
 */

#include "mex.h"
#if defined(_MSC_VER) && !defined(__attribute__)
#define __attribute__(x)
#endif

#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 1. Edge Impulse Porting Layer for Windows / MATLAB
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

EI_IMPULSE_ERROR ei_run_impulse_check_canceled() {
    return EI_IMPULSE_OK;
}

EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    return EI_IMPULSE_OK;
}

uint64_t ei_read_timer_ms() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

uint64_t ei_read_timer_us() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();
}

void ei_printf(const char *format, ...) {
    va_list myargs;
    va_start(myargs, format);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, myargs);
    mexPrintf("%s", buf);
    va_end(myargs);
}

void ei_printf_float(float f) {
    mexPrintf("%f", f);
}

void *ei_malloc(size_t size) {
    return malloc(size);
}

void *ei_calloc(size_t nitems, size_t size) {
    return calloc(nitems, size);
}

void ei_free(void *ptr) {
    free(ptr);
}

void ei_serial_set_baudrate(int baudrate) {
    (void)baudrate;
}

extern "C" void DebugLog(const char* s) {
    mexPrintf("%s", s);
}

// 2. MicroContext stubs for Edge Impulse EON & TFLite Micro runtime
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_log.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_context.h"

namespace tflite {
MicroContext::MicroContext(MicroAllocator* allocator, const Model* model, MicroGraph* graph)
    : allocator_(*allocator), graph_(*graph), model_(model) {}
MicroContext::~MicroContext() {}
void* MicroContext::AllocatePersistentBuffer(size_t bytes) { return nullptr; }
TfLiteStatus MicroContext::RequestScratchBufferInArena(size_t bytes, int* buffer_idx) { return kTfLiteOk; }
void* MicroContext::GetScratchBuffer(int buffer_idx) { return nullptr; }
TfLiteTensor* MicroContext::AllocateTempTfLiteTensor(int tensor_idx) { return nullptr; }
void MicroContext::DeallocateTempTfLiteTensor(TfLiteTensor* tensor) {}
TfLiteEvalTensor* MicroContext::GetEvalTensor(int tensor_idx) { return nullptr; }
int MicroContext::GetTensorIndex(int index, int max_size, const int* tensor_indices) { return -1; }
TfLiteTensor* MicroContext::AllocateTempInputTensor(const TfLiteNode* node, int index) {
    if (!node || !node->inputs || index < 0 || index >= node->inputs->size) return nullptr;
    int tensor_index = node->inputs->data[index];
    if (tensor_index < 0) return nullptr;
    return AllocateTempTfLiteTensor(tensor_index);
}
TfLiteTensor* MicroContext::AllocateTempOutputTensor(const TfLiteNode* node, int index) {
    if (!node || !node->outputs || index < 0 || index >= node->outputs->size) return nullptr;
    int tensor_index = node->outputs->data[index];
    if (tensor_index < 0) return nullptr;
    return AllocateTempTfLiteTensor(tensor_index);
}
TfLiteTensor* MicroContext::AllocateTempIntermediateTensor(const TfLiteNode* node, int index) {
    if (!node || !node->intermediates || index < 0 || index >= node->intermediates->size) return nullptr;
    int tensor_index = node->intermediates->data[index];
    if (tensor_index < 0) return nullptr;
    return AllocateTempTfLiteTensor(tensor_index);
}
void MicroContext::SetScratchBufferHandles(ScratchBufferHandle* scratch_buffer_handles) {}
TfLiteStatus MicroContext::set_external_context(void* external_context_payload) { return kTfLiteOk; }
void MicroContextReportOpError(struct TfLiteContext* context, const char* format, ...) {
    va_list args;
    va_start(args, format);
    Log(format, args);
    va_end(args);
}

TfLiteStatus TfLiteTypeSizeOf(TfLiteType type, size_t* size) {
    switch (type) {
        case kTfLiteFloat32: *size = sizeof(float); break;
        case kTfLiteInt16:   *size = sizeof(int16_t); break;
        case kTfLiteInt32:   *size = sizeof(int32_t); break;
        case kTfLiteUInt32:  *size = sizeof(uint32_t); break;
        case kTfLiteUInt8:   *size = sizeof(uint8_t); break;
        case kTfLiteInt8:    *size = sizeof(int8_t); break;
        case kTfLiteInt64:   *size = sizeof(int64_t); break;
        case kTfLiteUInt64:  *size = sizeof(uint64_t); break;
        case kTfLiteBool:    *size = sizeof(bool); break;
        default: return kTfLiteError;
    }
    return kTfLiteOk;
}

TfLiteStatus TfLiteEvalTensorByteLength(const TfLiteEvalTensor* eval_tensor, size_t* out_bytes) {
    if (!out_bytes) return kTfLiteError;
    int element_count = 1;
    if (eval_tensor->dims != nullptr) {
        for (int n = 0; n < eval_tensor->dims->size; ++n) {
            element_count *= eval_tensor->dims->data[n];
        }
    }
    size_t type_size = 0;
    if (TfLiteTypeSizeOf(eval_tensor->type, &type_size) != kTfLiteOk) return kTfLiteError;
    *out_bytes = element_count * type_size;
    return kTfLiteOk;
}
} // namespace tflite

// 3. Include Edge Impulse inferencing engine
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

struct SignalContext {
    const float *data;
    size_t total_length;
};

SignalContext g_sig_ctx = { nullptr, 0 };

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    if (offset + length > g_sig_ctx.total_length) {
        return EIDSP_OUT_OF_BOUNDS;
    }
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = g_sig_ctx.data[offset + i];
    }
    return EIDSP_OK;
}

static mxArray* createLabelsCell() {
    mxArray *labels_cell = mxCreateCellMatrix(1, (mwSize)EI_CLASSIFIER_LABEL_COUNT);
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        mxSetCell(labels_cell, i, mxCreateString(ei_classifier_inferencing_categories[i]));
    }
    return labels_cell;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Mode 1: Query model metadata ('info' or 0 arguments)
    if (nrhs == 0 || (nrhs >= 1 && mxIsChar(prhs[0]))) {
        const char *info_fields[] = {
            "project_name", "project_id", "num_channels",
            "window_samples", "frame_size", "frequency_hz",
            "num_classes", "labels", "has_anomaly", "is_object_detection"
        };
        plhs[0] = mxCreateStructMatrix(1, 1, 10, info_fields);
        mxSetField(plhs[0], 0, "project_name",   mxCreateString(EI_CLASSIFIER_PROJECT_NAME));
        mxSetField(plhs[0], 0, "project_id",     mxCreateDoubleScalar((double)EI_CLASSIFIER_PROJECT_ID));
        mxSetField(plhs[0], 0, "num_channels",   mxCreateDoubleScalar((double)EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME));
        mxSetField(plhs[0], 0, "window_samples", mxCreateDoubleScalar((double)EI_CLASSIFIER_RAW_SAMPLE_COUNT));
        mxSetField(plhs[0], 0, "frame_size",     mxCreateDoubleScalar((double)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE));
        mxSetField(plhs[0], 0, "frequency_hz",   mxCreateDoubleScalar((double)EI_CLASSIFIER_FREQUENCY));
        mxSetField(plhs[0], 0, "num_classes",    mxCreateDoubleScalar((double)EI_CLASSIFIER_LABEL_COUNT));
        mxSetField(plhs[0], 0, "labels",         createLabelsCell());
#if defined(EI_CLASSIFIER_HAS_ANOMALY) && (EI_CLASSIFIER_HAS_ANOMALY != 0)
        mxSetField(plhs[0], 0, "has_anomaly",    mxCreateDoubleScalar(1.0));
#else
        mxSetField(plhs[0], 0, "has_anomaly",    mxCreateDoubleScalar(0.0));
#endif
#if defined(EI_CLASSIFIER_OBJECT_DETECTION) && (EI_CLASSIFIER_OBJECT_DETECTION == 1)
        mxSetField(plhs[0], 0, "is_object_detection", mxCreateDoubleScalar(1.0));
#else
        mxSetField(plhs[0], 0, "is_object_detection", mxCreateDoubleScalar(0.0));
#endif
        return;
    }

    // Mode 2: Run classifier on Time-Series, Audio, or Image (uint8/double/single)
    const mxArray *input_mat = prhs[0];
    if (!mxIsDouble(input_mat) && !mxIsSingle(input_mat) && !mxIsUint8(input_mat)) {
        mexErrMsgIdAndTxt("EdgeImpulse:MEX:TypeError",
            "Input signal must be double, single, or uint8 (RGB/Grayscale image), or 'info' for metadata.");
    }

    std::vector<float> float_buffer(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    mwSize ndims = mxGetNumberOfDimensions(input_mat);
    const mwSize *dims = mxGetDimensions(input_mat);

    // Case A: 3D RGB Image (H x W x 3) -> pack row-major 0xRRGGBB float pixels
    if (ndims == 3 && dims[2] == 3 && (size_t)(dims[0] * dims[1]) == (size_t)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        size_t H = (size_t)dims[0];
        size_t W = (size_t)dims[1];
        size_t plane = H * W;
        if (mxIsUint8(input_mat)) {
            const uint8_t *p = (const uint8_t*)mxGetData(input_mat);
            for (size_t r = 0; r < H; r++) {
                for (size_t c = 0; c < W; c++) {
                    size_t idx = c * H + r; // MATLAB column-major
                    uint32_t R = p[idx];
                    uint32_t G = p[plane + idx];
                    uint32_t B = p[2 * plane + idx];
                    float_buffer[r * W + c] = static_cast<float>((R << 16) | (G << 8) | B);
                }
            }
        } else {
            double maxV = 0.0;
            size_t total3 = 3 * plane;
            if (mxIsDouble(input_mat)) {
                const double *p = mxGetPr(input_mat);
                for (size_t i = 0; i < total3; i++) if (p[i] > maxV) maxV = p[i];
                double scale = (maxV <= 1.0 && maxV > 0.0) ? 255.0 : 1.0;
                for (size_t r = 0; r < H; r++) {
                    for (size_t c = 0; c < W; c++) {
                        size_t idx = c * H + r;
                        uint32_t R = (uint32_t)(p[idx] * scale);
                        uint32_t G = (uint32_t)(p[plane + idx] * scale);
                        uint32_t B = (uint32_t)(p[2 * plane + idx] * scale);
                        float_buffer[r * W + c] = static_cast<float>(((R & 0xFF) << 16) | ((G & 0xFF) << 8) | (B & 0xFF));
                    }
                }
            } else {
                const float *p = (const float*)mxGetData(input_mat);
                for (size_t i = 0; i < total3; i++) if (p[i] > maxV) maxV = p[i];
                double scale = (maxV <= 1.0 && maxV > 0.0) ? 255.0 : 1.0;
                for (size_t r = 0; r < H; r++) {
                    for (size_t c = 0; c < W; c++) {
                        size_t idx = c * H + r;
                        uint32_t R = (uint32_t)(p[idx] * scale);
                        uint32_t G = (uint32_t)(p[plane + idx] * scale);
                        uint32_t B = (uint32_t)(p[2 * plane + idx] * scale);
                        float_buffer[r * W + c] = static_cast<float>(((R & 0xFF) << 16) | ((G & 0xFF) << 8) | (B & 0xFF));
                    }
                }
            }
        }
    } else {
        // Case B: 1D/2D Time-Series, Audio, or Grayscale uint8 Image
        size_t num_elements = mxGetNumberOfElements(input_mat);
        if (num_elements != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            mexErrMsgIdAndTxt("EdgeImpulse:MEX:SizeMismatch",
                "Model '%s' expects %d values (%d samples x %d channel(s) @ %g Hz) or HxWx3 RGB image. Received %d values.",
                EI_CLASSIFIER_PROJECT_NAME,
                (int)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
                (int)EI_CLASSIFIER_RAW_SAMPLE_COUNT,
                (int)EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME,
                (double)EI_CLASSIFIER_FREQUENCY,
                (int)num_elements);
        }

        size_t rows = mxGetM(input_mat);
        size_t cols = mxGetN(input_mat);

        if (mxIsUint8(input_mat)) {
            const uint8_t *p = (const uint8_t*)mxGetData(input_mat);
            // Row-major grayscale packing (0xGGGGGG) for uint8 2D images
            for (size_t r = 0; r < rows; r++) {
                for (size_t c = 0; c < cols; c++) {
                    uint32_t G = p[c * rows + r];
                    float_buffer[r * cols + c] = static_cast<float>((G << 16) | (G << 8) | G);
                }
            }
        } else if (mxIsDouble(input_mat)) {
            double *d_ptr = mxGetPr(input_mat);
            if (rows == EI_CLASSIFIER_RAW_SAMPLE_COUNT && cols == EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME && cols > 1) {
                for (size_t t = 0; t < rows; t++) {
                    for (size_t axis = 0; axis < cols; axis++) {
                        float_buffer[t * cols + axis] = static_cast<float>(d_ptr[axis * rows + t]);
                    }
                }
            } else {
                for (size_t i = 0; i < num_elements; i++) {
                    float_buffer[i] = static_cast<float>(d_ptr[i]);
                }
            }
        } else {
            float *f_ptr = (float*)mxGetData(input_mat);
            if (rows == EI_CLASSIFIER_RAW_SAMPLE_COUNT && cols == EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME && cols > 1) {
                for (size_t t = 0; t < rows; t++) {
                    for (size_t axis = 0; axis < cols; axis++) {
                        float_buffer[t * cols + axis] = f_ptr[axis * rows + t];
                    }
                }
            } else {
                for (size_t i = 0; i < num_elements; i++) {
                    float_buffer[i] = f_ptr[i];
                }
            }
        }
    }

    g_sig_ctx.data = float_buffer.data();
    g_sig_ctx.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);

    if (ei_err != EI_IMPULSE_OK) {
        mexErrMsgIdAndTxt("EdgeImpulse:MEX:InferenceError",
            "Edge Impulse inference failed with error code: %d", (int)ei_err);
    }

    const char *field_names[] = {
        "predicted_id", "predicted_label", "best_confidence",
        "scores", "labels", "anomaly", "bounding_boxes",
        "timing_dsp_ms", "timing_infer_ms"
    };
    plhs[0] = mxCreateStructMatrix(1, 1, 9, field_names);

    mxArray *scores_mat = mxCreateDoubleMatrix(1, (mwSize)EI_CLASSIFIER_LABEL_COUNT, mxREAL);
    double *scores_ptr = mxGetPr(scores_mat);
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        scores_ptr[i] = 0.0;
    }

#if defined(EI_CLASSIFIER_OBJECT_DETECTION) && (EI_CLASSIFIER_OBJECT_DETECTION == 1)
    // Object Detection (FOMO / YOLO / SSD): read result.bounding_boxes
    const char *bb_fields[] = { "label", "x", "y", "width", "height", "value" };
    std::vector<ei_impulse_result_bounding_box_t> valid_boxes;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        if (result.bounding_boxes[i].value > 0.0f) {
            valid_boxes.push_back(result.bounding_boxes[i]);
        }
    }
    mxArray *bb_arr = mxCreateStructMatrix(1, (mwSize)valid_boxes.size(), 6, bb_fields);
    float best_score = 0.0f;
    size_t best_idx = 0;
    const char *best_label = "none";
    bool found_any = false;

    for (size_t i = 0; i < valid_boxes.size(); i++) {
        const auto &bb = valid_boxes[i];
        mxSetField(bb_arr, i, "label",  mxCreateString(bb.label ? bb.label : ""));
        mxSetField(bb_arr, i, "x",      mxCreateDoubleScalar((double)bb.x));
        mxSetField(bb_arr, i, "y",      mxCreateDoubleScalar((double)bb.y));
        mxSetField(bb_arr, i, "width",  mxCreateDoubleScalar((double)bb.width));
        mxSetField(bb_arr, i, "height", mxCreateDoubleScalar((double)bb.height));
        mxSetField(bb_arr, i, "value",  mxCreateDoubleScalar((double)bb.value));

        for (size_t c = 0; c < EI_CLASSIFIER_LABEL_COUNT; c++) {
            if (bb.label && strcmp(bb.label, ei_classifier_inferencing_categories[c]) == 0) {
                if ((double)bb.value > scores_ptr[c]) scores_ptr[c] = (double)bb.value;
                if (!found_any || bb.value > best_score) {
                    best_score = bb.value;
                    best_idx = c;
                    best_label = bb.label;
                    found_any = true;
                }
                break;
            }
        }
    }
    mxSetField(plhs[0], 0, "predicted_id",    mxCreateDoubleScalar(found_any ? (double)(best_idx + 1) : 0.0));
    mxSetField(plhs[0], 0, "predicted_label", mxCreateString(best_label));
    mxSetField(plhs[0], 0, "best_confidence", mxCreateDoubleScalar((double)best_score));
    mxSetField(plhs[0], 0, "bounding_boxes",  bb_arr);
#else
    // Classification & Regression: read result.classification
    size_t best_idx = 0;
    float best_score = -1e30f;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        scores_ptr[i] = static_cast<double>(result.classification[i].value);
        if (result.classification[i].value > best_score) {
            best_score = result.classification[i].value;
            best_idx = i;
        }
    }
    mxSetField(plhs[0], 0, "predicted_id",    mxCreateDoubleScalar((double)(best_idx + 1)));
    mxSetField(plhs[0], 0, "predicted_label", mxCreateString(ei_classifier_inferencing_categories[best_idx]));
    mxSetField(plhs[0], 0, "best_confidence", mxCreateDoubleScalar((double)best_score));
    mxSetField(plhs[0], 0, "bounding_boxes",  mxCreateDoubleMatrix(0, 0, mxREAL));
#endif

#if defined(EI_CLASSIFIER_HAS_ANOMALY) && (EI_CLASSIFIER_HAS_ANOMALY != 0)
    mxSetField(plhs[0], 0, "anomaly",         mxCreateDoubleScalar((double)result.anomaly));
#else
    mxSetField(plhs[0], 0, "anomaly",         mxCreateDoubleScalar(mxGetNaN()));
#endif

    mxSetField(plhs[0], 0, "scores",          scores_mat);
    mxSetField(plhs[0], 0, "labels",          createLabelsCell());
    mxSetField(plhs[0], 0, "timing_dsp_ms",   mxCreateDoubleScalar((double)result.timing.dsp));
    mxSetField(plhs[0], 0, "timing_infer_ms", mxCreateDoubleScalar((double)result.timing.classification));
}
