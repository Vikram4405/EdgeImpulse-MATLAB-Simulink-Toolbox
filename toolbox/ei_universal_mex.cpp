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
            "num_classes", "labels"
        };
        plhs[0] = mxCreateStructMatrix(1, 1, 8, info_fields);
        mxSetField(plhs[0], 0, "project_name",   mxCreateString(EI_CLASSIFIER_PROJECT_NAME));
        mxSetField(plhs[0], 0, "project_id",     mxCreateDoubleScalar((double)EI_CLASSIFIER_PROJECT_ID));
        mxSetField(plhs[0], 0, "num_channels",   mxCreateDoubleScalar((double)EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME));
        mxSetField(plhs[0], 0, "window_samples", mxCreateDoubleScalar((double)EI_CLASSIFIER_RAW_SAMPLE_COUNT));
        mxSetField(plhs[0], 0, "frame_size",     mxCreateDoubleScalar((double)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE));
        mxSetField(plhs[0], 0, "frequency_hz",   mxCreateDoubleScalar((double)EI_CLASSIFIER_FREQUENCY));
        mxSetField(plhs[0], 0, "num_classes",    mxCreateDoubleScalar((double)EI_CLASSIFIER_LABEL_COUNT));
        mxSetField(plhs[0], 0, "labels",         createLabelsCell());
        return;
    }

    // Mode 2: Run classifier on numeric matrix
    const mxArray *input_mat = prhs[0];
    if (!mxIsDouble(input_mat) && !mxIsSingle(input_mat)) {
        mexErrMsgIdAndTxt("EdgeImpulse:MEX:TypeError",
            "Input signal must be double or single precision (or 'info' for metadata).");
    }

    size_t num_elements = mxGetNumberOfElements(input_mat);
    if (num_elements != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        mexErrMsgIdAndTxt("EdgeImpulse:MEX:SizeMismatch",
            "Model '%s' expects %d values (%d samples x %d channel(s) @ %g Hz). Received %d values.",
            EI_CLASSIFIER_PROJECT_NAME,
            (int)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
            (int)EI_CLASSIFIER_RAW_SAMPLE_COUNT,
            (int)EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME,
            (double)EI_CLASSIFIER_FREQUENCY,
            (int)num_elements);
    }

    std::vector<float> float_buffer(num_elements);
    size_t rows = mxGetM(input_mat);
    size_t cols = mxGetN(input_mat);

    if (mxIsDouble(input_mat)) {
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

    g_sig_ctx.data = float_buffer.data();
    g_sig_ctx.total_length = num_elements;

    signal_t signal;
    signal.total_length = num_elements;
    signal.get_data = &get_signal_data;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);

    if (ei_err != EI_IMPULSE_OK) {
        mexErrMsgIdAndTxt("EdgeImpulse:MEX:InferenceError",
            "Edge Impulse inference failed with error code: %d", (int)ei_err);
    }

    const char *field_names[] = {
        "predicted_id", "predicted_label", "best_confidence",
        "scores", "labels", "timing_dsp_ms", "timing_infer_ms"
    };
    plhs[0] = mxCreateStructMatrix(1, 1, 7, field_names);

    mxArray *scores_mat = mxCreateDoubleMatrix(1, (mwSize)EI_CLASSIFIER_LABEL_COUNT, mxREAL);
    double *scores_ptr = mxGetPr(scores_mat);
    size_t best_idx = 0;
    float best_score = -1.0f;

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
    mxSetField(plhs[0], 0, "scores",          scores_mat);
    mxSetField(plhs[0], 0, "labels",          createLabelsCell());
    mxSetField(plhs[0], 0, "timing_dsp_ms",   mxCreateDoubleScalar((double)result.timing.dsp));
    mxSetField(plhs[0], 0, "timing_infer_ms", mxCreateDoubleScalar((double)result.timing.classification));
}
