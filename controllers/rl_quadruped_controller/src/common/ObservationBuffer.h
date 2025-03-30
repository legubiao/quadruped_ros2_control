//
// Created by biao on 24-10-6.
//

#ifndef OBSERVATIONBUFFER_H
#define OBSERVATIONBUFFER_H

#include <torch/torch.h>
#include <vector>

class ObservationBuffer
{
public:
    ObservationBuffer(int num_envs, int num_obs, int include_history_steps);

    ~ObservationBuffer() = default;

    void reset(const std::vector<int>& reset_index, const torch::Tensor& new_obs);

    void clear();

    void insert(const torch::Tensor& new_obs);

    [[nodiscard]] torch::Tensor getObsVec(const std::vector<int>& obs_ids) const;

private:
    int num_envs_;
    int num_obs_;
    int include_history_steps_;
    int num_obs_total_;
    torch::Tensor obs_buffer_;
};


#endif //OBSERVATIONBUFFER_H
