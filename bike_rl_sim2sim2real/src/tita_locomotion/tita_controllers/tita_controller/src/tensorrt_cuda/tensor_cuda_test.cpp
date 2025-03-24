#include "tensorrt_cuda/tensor_cuda_test.hpp"


// nvinfer1::ICudaEngine * CudaTest::get_engine(const std::string & engine_file_path)
// {
//   std::ifstream file(engine_file_path, std::ios::binary);
//   if (!file.good()) {
//     return nullptr;
//   }
//   std::vector<char> engine_data(
//     (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
//   file.close();
//   nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(gLogger);
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
//   // return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
//   return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
// #pragma GCC diagnostic pop
// }

nvinfer1::ICudaEngine * CudaTest::get_engine(const std::string & engine_file_path)
{
  std::ifstream file(engine_file_path, std::ios::binary);
  if (!file.good()) {
    return nullptr;
  }
  std::vector<char> engine_data(
    (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();
  nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(gLogger);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);
    return runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
#pragma GCC diagnostic pop
}

void CudaTest::cuda_memory_init(void)
{
  cudaStreamCreate(&stream);
  cudaMalloc(reinterpret_cast<void **>(&buffers[0]), input_size_0);
  cudaMalloc(reinterpret_cast<void **>(&buffers[1]), input_size_1);
  cudaMalloc(reinterpret_cast<void **>(&buffers[2]), output_size);
}
// Function to do inference
// void CudaTest::do_inference(
//   const float * input_0, const float * input_1, float * output)
// {
//   cudaMemcpyAsync(buffers[0], input_0, input_size_0, cudaMemcpyHostToDevice, stream);
//   cudaMemcpyAsync(buffers[1], input_1, input_size_1, cudaMemcpyHostToDevice, stream);
// // context->enqueue(1, (void**)buffers, stream, nullptr);
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
//   // context->enqueueV2(reinterpret_cast<void **>(buffers), stream, nullptr);
//   context->enqueueV3(stream);
// #pragma GCC diagnostic pop
//   cudaMemcpyAsync(output, buffers[2], output_size, cudaMemcpyDeviceToHost, stream);
//   cudaStreamSynchronize(stream);
// }

void CudaTest::do_inference(
  const float * input_0, const float * input_1, float * output)
{
    cudaMemcpyAsync(buffers[0], input_0, input_size_0, cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(buffers[1], input_1, input_size_1, cudaMemcpyHostToDevice, stream);
    context->setTensorAddress("input_tensor",buffers[0]);
    context->setTensorAddress("obs_hist",buffers[1]);
    context->setTensorAddress("output_tensor",buffers[2]);
    // context->setTensorAddress("onnx::Unsqueeze_0",buffers[0]);
    // context->setTensorAddress("onnx::Slice_1",buffers[1]);
    // context->setTensorAddress("90",buffers[2]);
    context->enqueueV3(stream);  // 使用 enqueueV3
    // printf("%.3f",*buffers[2]);
    // printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,\n",
    // buffers[2][0], buffers[2][1], buffers[2][2], buffers[2][3],  
    // buffers[2][4], buffers[2][5], buffers[2][6], buffers[2][7],
    // buffers[2][8], buffers[2][8], buffers[2][9], buffers[2][10],
    // buffers[2][12], buffers[2][13], buffers[2][14], buffers[2][15]);
    cudaStreamSynchronize(stream);
    cudaMemcpyAsync(output, buffers[2], output_size, cudaMemcpyDeviceToHost, stream);

}

bool CudaTest::get_cuda_init(void) { return cuda_init; }

CudaTest::CudaTest(const std::string & engine_file_path)
{
  engine_ = get_engine(engine_file_path);
  if (engine_ != nullptr) {
    context = engine_->createExecutionContext();
    cuda_memory_init();
    cuda_init = true;
  } else {
    cuda_init = false;
  }
}

CudaTest::~CudaTest()
{
  cudaStreamDestroy(stream);
  for (void * buf : buffers) {
    cudaFree(buf);
  }
}
