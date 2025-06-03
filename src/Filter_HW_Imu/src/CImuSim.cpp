// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuSim.h"

void CImuSim::step(const float& _dt, const CPose& _pose, const Eigen::Vector3f& _linear_velocity, CycImu& _out_imu)
{
    // Rotation as quaternion (used to update the rotation of the IMU with ideal values)
    CQuaternion quat = _pose.rotation_quat();

    // Rotation as Euler angles (used to compute the IMU simulation)
    Eigen::Vector3f rpy = _pose.rotation_euler();

    // Compute linear cceleration [m/s^2]
    // 2nd derivative of the polynomic (quadratic) interpolation using the point in current time and two previous steps:
    // d2[i] = -2.0*(y1/(h1*h2)-y2/((h2+h1)*h2)-y0/(h1*(h2+h1)))
    Eigen::Vector3f linear_acc((_linear_velocity.x() - m_PrevLinearVelocity.x()) / _dt,
        (_linear_velocity.y() - m_PrevLinearVelocity.y()) / _dt,
        (_linear_velocity.z() - m_PrevLinearVelocity.z()) / _dt);

    // Add gravity to linear acceleration
    // TODO: Check this!!
    linear_acc.z() -= GRAVITY;

    // Compute angular velocity [deg/s]
    // TODO: fix this!!
    // this is correct only if the linear acc is 0
    //linear_acc = Eigen::Vector3f::Zero();
    const Eigen::Vector3f angular_vel(((rpy.x() - m_PrevPose.rotation_euler().x()) * RAD2DEG) / _dt,
        ((rpy.y() - m_PrevPose.rotation_euler().y()) * RAD2DEG) / _dt,
        ((rpy.z() - m_PrevPose.rotation_euler().z()) * RAD2DEG) / _dt);

    // Compute magnetometer [uT]
    const Eigen::Vector3f magnetic_field(cosf(rpy.z()), sinf(rpy.z()), 0.f);

    // Store data
    _out_imu.acc = linear_acc;
    _out_imu.gyro = angular_vel;
    _out_imu.magnet = magnetic_field;
    _out_imu.quat = quat.to_vector();

    m_PrevPose = _pose;
    m_PrevLinearVelocity = _linear_velocity;
}
