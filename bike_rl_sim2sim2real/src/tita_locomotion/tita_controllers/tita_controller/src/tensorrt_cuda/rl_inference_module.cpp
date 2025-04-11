#include "tensorrt_cuda/rl_inference_module.hpp"

void RL_InferenceModule::_update_obs(const std::vector<float> _newObs)
{
    for (int i = 0; i < _numObs; i++)
    {
        input_0.get()[i] = _newObs[i];
    }
}

void RL_InferenceModule::run_prepare()
{
    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        this->input_1.get()[i] = 0;
    for (int i = 0; i < _numOuput; i++)
        this->output_last.get()[i] = 0;

    for (int i = 0; i < _obsBuff; i++)
    {
        this->_update_obs(this->_get_obs());
        for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
            input_1_temp.get()[i] = input_1.get()[i + _numObs];

        for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
            input_1.get()[i] = input_1_temp.get()[i];

        for (int i = 0; i < _numObs; i++)
            input_1.get()[i + _numObs * (_obsBuff - 1)] = input_0.get()[i];
    }

    for (int i = 0; i < _obsBuff; i++)
    {
        this->forward();
    }
}

void RL_InferenceModule::forward()
{
    this->_update_obs(this->_get_obs());
    this->cuda->do_inference(input_0.get(), input_1.get(), output.get());

    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        input_1_temp.get()[i] = input_1.get()[i + _numObs];

    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < _numObs; i++)
        input_1.get()[i + _numObs * (_obsBuff - 1)] = input_0.get()[i];

    for (int i = 0; i < _numOuput; i++)
        output_last.get()[i] = output.get()[i];
}

void RL_InferenceModule::run_prepare(const std::vector<float> _newObs)
{
    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        this->input_1.get()[i] = 0;
    for (int i = 0; i < _numOuput; i++)
        this->output_last.get()[i] = 0;

    for (int i = 0; i < _obsBuff; i++)
    {
        this->_update_obs(_newObs);
        for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
            input_1_temp.get()[i] = input_1.get()[i + _numObs];

        for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
            input_1.get()[i] = input_1_temp.get()[i];

        for (int i = 0; i < _numObs; i++)
            input_1.get()[i + _numObs * (_obsBuff - 1)] = input_0.get()[i];
    }

    for (int i = 0; i < _obsBuff; i++)
    {
        this->forward(_newObs);
    }
}

void RL_InferenceModule::forward(const std::vector<float> _newObs)
{
    this->_update_obs(_newObs);
    this->cuda->do_inference(input_0.get(), input_1.get(), output.get());

    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        input_1_temp.get()[i] = input_1.get()[i + _numObs];

    for (int i = 0; i < _numObs * (_obsBuff - 1); i++)
        input_1.get()[i] = input_1_temp.get()[i];

    for (int i = 0; i < _numObs; i++)
        input_1.get()[i + _numObs * (_obsBuff - 1)] = input_0.get()[i];

    for (int i = 0; i < _numOuput; i++)
        output_last.get()[i] = output.get()[i];
}

