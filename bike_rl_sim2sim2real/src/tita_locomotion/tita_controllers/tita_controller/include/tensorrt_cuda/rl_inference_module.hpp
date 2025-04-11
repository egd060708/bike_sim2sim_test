// #include "tensorrt_cuda/tensor_cuda_test.hpp"

// class RL_InferenceModule{
// public:
//     RL_InferenceModule(int _numObs, int _obsBuff, int _numOutput){
//         this->input_0 = new float[_numObs];
//         this->input_1 = new float[_numObs*_obsBuff];
//         this->output = new float[_numOutput];
//         this->output_last = new float[_numOutput];
//         this->input_1_temp = new float[_numObs*(_obsBuff - 1)];
//         this->cuda = std::make_shared<CudaTest>("/home/lu/Git_Project/gitlab/bike_rl/engine/head_15model_6000.engine");
//     }
// private:
//     std::shared_ptr<CudaTest> cuda;

//     std::shared_ptr<float[]> input_0;
//     std::shared_ptr<float[]> input_1;
//     std::shared_ptr<float[]> output;

//     std::shared_ptr<float[]> output_last;
//     std::shared_ptr<float[]> input_1_temp;
// };