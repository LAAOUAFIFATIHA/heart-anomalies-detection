#ifndef __NeuralNetwork__
#define __NeuralNetwork__

extern const unsigned char ECG_model[];
extern const int ECG_model_len;

#include <stdint.h>

namespace tflite
{
    template <unsigned int tOpCount>
    class MicroMutableOpResolver;
    class MicroErrorReporter;  
    class Model;
    class MicroInterpreter;
} // namespace tflite

struct TfLiteTensor;

class NeuralNetwork
{
private:
    tflite::MicroMutableOpResolver<4>* resolver;  
    tflite::MicroErrorReporter* error_reporter;  
    const tflite::Model* model;
    tflite::MicroInterpreter* interpreter;
    TfLiteTensor* input;
    TfLiteTensor* output;
    uint8_t* tensor_arena;

public:
    NeuralNetwork();
    ~NeuralNetwork();  
    float* getInputBuffer();
    float predict();
};

#endif