#include "NeuralNetwork.h"
#include "ECG_model.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

const int kArenaSize = 20000; // """""" defines the size (in bytes)

NeuralNetwork::NeuralNetwork() : 
    error_reporter(nullptr),
    model(nullptr),
    resolver(nullptr),
    interpreter(nullptr),
    input(nullptr),
    output(nullptr),
    tensor_arena(nullptr)
{
    // Initialize error reporter
    error_reporter = new tflite::MicroErrorReporter();
    if (!error_reporter) {
        return;
    }

    // Load model
    model = tflite::GetModel(ECG_model);
    if (!model) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to load model");
        return;
    }

    // Verify version
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        TF_LITE_REPORT_ERROR(error_reporter, 
            "Model version %d != supported version %d",
            model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // Initialize op resolver
    resolver = new tflite::MicroMutableOpResolver<4>();
    if (!resolver) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to create resolver");
        return;
    }

    // Register operations
    TfLiteStatus status;
    status = resolver->AddFullyConnected();
    status = resolver->AddRelu();
    status = resolver->AddLogistic();
    status = resolver->AddReshape();
    
    if (status != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to add ops");
        return;
    }

    // Allocate memory
    tensor_arena = (uint8_t*)malloc(kArenaSize);
    if (!tensor_arena) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to allocate arena");
        return;
    }

    // Create interpreter
    interpreter = new tflite::MicroInterpreter(
        model, *resolver, tensor_arena, kArenaSize, error_reporter);
    if (!interpreter) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to create interpreter");
        return;
    }

    // Allocate tensors
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to allocate tensors");
        return;
    }

    // Get input/output tensors
    input = interpreter->input(0);
    output = interpreter->output(0);

    // Verify tensors
    if (!input || !output) {
        TF_LITE_REPORT_ERROR(error_reporter, "Failed to get input/output tensors");
    }
}

NeuralNetwork::~NeuralNetwork() {
    delete interpreter;
    delete resolver;
    delete error_reporter;
    free(tensor_arena);
}

float* NeuralNetwork::getInputBuffer() {
    if (input && input->data.f) {
        return input->data.f;
    }
    return nullptr;
}

float NeuralNetwork::predict() {
    if (!interpreter) {
        return -1.0f; // Error value
    }

    TfLiteStatus status = interpreter->Invoke();
    if (status != kTfLiteOk) {
        return -1.0f; // Error value
    }

    if (output && output->data.f) {
        return output->data.f[0];
    }
    return -1.0f; // Error value
}
