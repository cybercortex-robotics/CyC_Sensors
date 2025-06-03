// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

// https://stackoverflow.com/questions/71056930/how-do-physics-engines-model-angular-velocity-and-angular-acceleration-in-3d
// https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf
// https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html

#ifndef CImuSim_H
#define CImuSim_H

#include "CyC_TYPES.h"

class CImuSim
{
public:
    CImuSim() = default;
    CImuSim(const CImuSim&) = default;
    CImuSim(CImuSim&&) = default;
    CImuSim& operator=(const CImuSim&) = default;
    CImuSim& operator=(CImuSim&&) = default;
    ~CImuSim() = default;

    void step(const float& _dt, const CPose& _pose, const Eigen::Vector3f& _linear_velocity, CycImu& _out_imu);

private:
    CPose           m_PrevPose;
    Eigen::Vector3f m_PrevLinearVelocity = Eigen::Vector3f::Zero();
};
#endif // CImuSim_H