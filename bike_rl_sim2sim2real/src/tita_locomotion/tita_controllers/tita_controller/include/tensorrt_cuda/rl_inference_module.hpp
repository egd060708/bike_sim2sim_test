#pragma once
#include "tensorrt_cuda/tensor_cuda_test.hpp"
#include <functional>

class RL_InferenceModule
{
public:
    using ObsFunc = std::function<const std::vector<float> (void)>; // 包装器
    RL_InferenceModule(int _numObs, int _obsBuff, int _numOutput, const std::string &engine_file_path)
        :input_0(new float[_numObs], std::default_delete<float[]>()),
         input_1(new float[_numObs*_obsBuff], std::default_delete<float[]>()),
         output(new float[_numOutput], std::default_delete<float[]>()),
         output_last(new float[_numOutput], std::default_delete<float[]>()),
         input_1_temp(new float[_numObs*(_obsBuff - 1)], std::default_delete<float[]>())
    {
        this->_numObs = _numObs;
        this->_obsBuff = _obsBuff;
        this->_numOuput = _numOutput;
        this->cuda = std::make_shared<CudaTest>(engine_file_path);
    }
    ~RL_InferenceModule()
    {
    }
    void set_obs_func(ObsFunc _func)
    {
        this->_get_obs = _func;
    }
    // 返回 const 数据指针的引用
    std::shared_ptr<float[]> get_action () const
    {
        return this->output;
    }

    // 直接传入新的观测值
    void run_prepare(const std::vector<float> _newObs);
    void forward(const std::vector<float> _newObs);
    // 调用函数指针完成观测值内部更新
    void run_prepare();
    void forward();

private:
    int _numObs, _obsBuff, _numOuput;
    std::shared_ptr<CudaTest> cuda;

    std::shared_ptr<float[]> input_0;
    std::shared_ptr<float[]> input_1;
    std::shared_ptr<float[]> output;

    std::shared_ptr<float[]> output_last;
    std::shared_ptr<float[]> input_1_temp;

    void _update_obs(const std::vector<float> _newObs);
    // 使用函数指针调用通用外部函数
    ObsFunc _get_obs = NULL;
};