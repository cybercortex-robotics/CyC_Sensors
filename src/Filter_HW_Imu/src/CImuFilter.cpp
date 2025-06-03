// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CImuFilter.h"

#ifdef __ANDROID_API__
#include <android/looper.h>
#include <android/sensor.h>
#endif

#ifdef ENABLE_UNITREEA1
#include <CUnitreeA1.h>
#endif

#define DYNALO_EXPORT_SYMBOLS
#include <dynalo/symbol_helper.hpp>

DYNALO_EXPORT CyC_FILTER_TYPE DYNALO_CALL getFilterType()
{
    CycDatablockKey key;
    return CImuFilter(key).getFilterType();
}

DYNALO_EXPORT CCycFilterBase* DYNALO_CALL createFilter(const ConfigFilterParameters _params)
{
    return new CImuFilter(_params);
}

CImuFilter::CImuFilter(CycDatablockKey _key) : 
    CCycFilterBase(_key)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_IMU_FILTER_TYPE");
    m_OutputDataType = CyC_IMU;
}

CImuFilter::CImuFilter(const ConfigFilterParameters& _params) : 
    CCycFilterBase(_params)
{
    // Assign the filter type, input type and output type
    setFilterType("CyC_IMU_FILTER_TYPE");
    m_OutputDataType = CyC_IMU;

    // Load the sensor model
    if (!_params.CustomParameters.at("calibration").empty())
    {
        fs::path calib_file = fs::path(_params.sGlobalBasePath) / fs::path(_params.CustomParameters.at("calibration"));
        this->m_pSensorModel = new CImuSensorModel(calib_file.string());
    }

    // Check if a realsense robot is already registered
    if (_params.pSingletonRegistry != nullptr)
    {
        if (_params.pSingletonRegistry->get<CRealSense2Api>() == nullptr)
            _params.pSingletonRegistry->registerInstance<CRealSense2Api>();
        m_RealSense = _params.pSingletonRegistry->get<CRealSense2Api>();
    }

    if (!m_CustomParameters["interface"].empty())
    {
        m_ApiType = StringToEnumType(m_CustomParameters["interface"]);
    }
    else
    {
        spdlog::warn("Filter [{}-{}]: {}: No interface type defined. Switching to simulation.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
        m_ApiType = CImuUtils::Imu_API_SIM;
    }

    // Initial zero state
    m_VehicleState.x_hat = Eigen::VectorXf(NO_VEHICLE_MODEL_STATES + 1);
    m_VehicleState.x_hat << 0.f, 0.f, 0.f, 0.f, 0.f;

    m_PrevState.x_hat = Eigen::VectorXf(NO_VEHICLE_MODEL_STATES + 1);
    m_PrevState.x_hat << 0.f, 0.f, 0.f, 0.f, 0.f;
}

CImuFilter::~CImuFilter()
{
	if (m_bIsEnabled)
		disable();
}

/*
// Simulate inertial data from vehicle state
void CImuFilter::simulateImuData()
{
    // Get the vehicle state
    if (m_pVehStateFilter != nullptr)
    {
        m_VehicleState = m_pVehStateFilter->getState();
    }

    // Get the time step
    float dt = m_VehicleState.time - m_PrevState.time;

    // Get the vehicle state
    float x = m_VehicleState.x_hat(0);
    float y = m_VehicleState.x_hat(1);
    float psi = m_VehicleState.x_hat(2);
    float v = m_VehicleState.x_hat(3);
    float omega = m_VehicleState.x_hat(4);

    // Compute the velocity in the inertial frame
    float vx = v * cos(psi);
    float vy = v * sin(psi);

    // Compute the acceleration in the inertial frame
    float ax = (vx - m_PrevState.x_hat(3)) / dt;
    float ay = (vy - m_PrevState.x_hat(4)) / dt;

    // Compute the angular velocity in the inertial frame
    float omega_x = omega * cos(psi);
    float omega_y = omega * sin(psi);

    // Compute the angular acceleration in the inertial frame
    float alpha_x = (omega_x - m_PrevState.x_hat(4)) / dt;
    float alpha_y = (omega_y - m_PrevState.x_hat(5)) / dt;

    // Compute the angular acceleration in the inertial frame
    float alpha_z = (omega - m_PrevState.x_hat(6)) / dt;

    // Compute the angular velocity in the body frame
    float omega_bx = omega_x - omega_y * tan(m_pSensorModel->getPitch());
    float omega_by = omega_y * cos(m_pSensorModel->getPitch());

    // Compute the angular acceleration in the body frame
    float alpha_bx = alpha_x - alpha_y * tan(m_pSensorModel->getPitch());
    float alpha_by = alpha_y * cos(m_pSensorModel->getPitch());
    float alpha_bz = alpha_z;

    // Compute the acceleration in the
*/
bool CImuFilter::enable()
{
    if (!isNetworkFilter())
	{
        //// Check if the calibration file exists
        //fs::path path = fs::path(CConfigParameters::instance().getGlobalBasePath()) / fs::path(m_CustomParameters.at("calibration"));
        //if (!CFileUtils::FileExist(path.c_str()))
        //{
        //    spdlog::error("Filter [{}-{}]: {}: ERROR Calibration file does not exist. Disabling the CImuFilter filter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
        //    return false;
        //}
        //else
        //{
        //    fs::path calib_file = fs::path(CConfigParameters::instance().getGlobalBasePath()) / fs::path(m_CustomParameters.at("calibration"));
        //    this->m_pSensorModel = new CImuSensorModel(calib_file.string());
        //}

        // Get the pose data filter
        m_pPoseDataFilter = CFilterUtils::getStateFilter(this->getInputSources());

        if (!isReplayFilter())
        {
            //for (const CycInputSource& src : this->getInputSources())
            //    if (src.pCycFilter->getFilterType().compare("CyC_VEHICLE_SIMULATION_FILTER_TYPE") == 0 || src.pCycFilter->getFilterType().compare("CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE"))
            //            m_pVehStateFilter = src.pCycFilter;

            if (m_ApiType == CImuUtils::Imu_API_SIM)
            {
                if (m_pVehStateFilter == nullptr)
                {
                    spdlog::error("Filter [{}-{}]: {}: Expected CyC_VEHICLE_SIMULATION_FILTER_TYPE or CyC_VEHICLE_STATE_ESTIMATION_FILTER_TYPE as input filter. CImuFilter disabled.",
                        getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                    return false;
                }
            }
            else if (m_ApiType == CImuUtils::Imu_API_MPU9250)
            {
                CyC_INT port_nr = -1;
                if (m_CustomParameters.find("PortNr") != m_CustomParameters.end())
                {
                    port_nr = std::stoi(m_CustomParameters["PortNr"]);
                }
                else
                {
                    spdlog::error("Filter [{}-{}]: {}: Undefined port number in configuration file.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                    return false;
                }

                if (!m_ImuMPU9250_API.start(port_nr))
                {
                    spdlog::error("Filter [{}-{}]: {}: Can't open MPU9250 port number {}", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name(), port_nr);
                    return false;
                }

            }
            else if (m_ApiType == CImuUtils::Imu_API_BRICK)
            {
                if (!m_ImuBrick_API.setup(100))
                {
                    spdlog::error("Filter[{}-{}]: {}: Can't setup brick imu.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
                m_ImuBrick_API.start();
            }
            else if (m_ApiType == CImuUtils::Imu_API_ANDROID)
            {
#ifdef __ANDROID_API__
                /*
                // create a sensor manager
                ASensorManager *sensorManager = ASensorManager_getInstance();

                // get list of all sensors in the device
                ASensorList sensorList;
                int numSensors = ASensorManager_getSensorList(sensorManager, &sensorList);

                for( int count = 0 ; count < numSensors ; count++ )
                {
                    spdlog::info("************Sensors Available********************\n");
                    spdlog::info(ASensor_getName(sensorList[count]));
                    spdlog::info(ASensor_getVendor(sensorList[count]));
                    spdlog::info(ASensor_getStringType(sensorList[count]));
                }*/

                m_ndkSensor = new NDKSensor();
#endif //__ANDROID_API__
            }
            else if (m_ApiType == CImuUtils::Imu_API_UNITREEA1)
            {
#ifdef ENABLE_UNITREEA1
                auto rpy = CUnitreeA1::instance().getImuEuler();
                spdlog::info("Filter [{}-{}]: {}: rpy {} {} {}", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name(), rpy[0], rpy[1], rpy[2]);
#else
                spdlog::info("Filter [{}-{}]: {}: ", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                m_bIsEnabled = false;
                return false;
#endif
            }
            else if (m_ApiType == CImuUtils::Imu_API_UNITY)
            {
                if (m_CustomParameters.find("ip") == m_CustomParameters.end() || m_CustomParameters.find("port") == m_CustomParameters.end())
                {
                    spdlog::error("Filter [{}-{}]: {}: ip and/or port parameters not found in configuration file", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                    return false;
                }
                m_bIsEnabled = m_ImuUnity_API.enable(m_CustomParameters["ip"], m_CustomParameters["port"]);
            }
            else if (m_ApiType == CImuUtils::Imu_API_REALSENSE)
            {
                m_bIsEnabled = m_RealSense->start();

                if (!m_bIsEnabled)
                {
                    spdlog::error("Filter [{}-{}]: {}: RealSense2 API could not be started.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
                    return false;
                }
            }
        }
	}

    spdlog::info("Filter [{}-{}]: {}::enable() successful.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());

	m_bIsEnabled = true;
	return true;
}

bool CImuFilter::disable()
{
	if(isRunning())
		stop();

    if (!isNetworkFilter())
        m_ImuMPU9250_API.stop();

	m_bIsEnabled = false;
	return true;
}

bool CImuFilter::process()
{
    bool bReturn = false;

    // Calculate dt [s]
    const float dt = (float)(CTimer::now() - this->getTimestampStop()) * MSEC2SEC;
    
    // Update IMU pose
    //CPose pose;
    //if (getPose(pose))
    //    this->m_pSensorModel->updatePose(pose * this->m_pSensorModel->extrinsics());

    if (m_ApiType == CImuUtils::Imu_API_SIM)
    {
        const CyC_TIME_UNIT ts = m_pVehStateFilter->getTimestampStop();

        if (ts > m_lastTsState)
        {
            if (m_pVehStateFilter->getData(m_VehicleState))
            {
                // https://scicomp.stackexchange.com/questions/11353/angular-velocity-by-vector-2d

                const float steering_angle = m_VehicleState.x_hat[4];
                const float side_slip = atanf(tanf(steering_angle) / 2.f);

                // Compute roll, pitch, yaw
                const float roll = 0.F;
                const float pitch = 0.F;
                const float yaw = m_VehicleState.x_hat[3] + side_slip;

                // Convert to quaternion
                CQuaternion quat(roll, pitch, yaw);

                // Linear velocity
                const Eigen::Vector2f lin_vel_W = (m_VehicleState.x_hat.head(2) - m_PrevState.x_hat.head(2)) / dt;
                const Eigen::Vector2f lin_vel_B = Eigen::Vector2f{
                    cosf(yaw) * lin_vel_W.x() + sinf(yaw) * lin_vel_W.y(),
                    -sinf(yaw) * lin_vel_W.x() + cosf(yaw) * lin_vel_W.y()
                };

                // Linear acceleration - OLD METHOD
                const float acc = (m_VehicleState.x_hat[2] - m_PrevState.x_hat[2]) / dt;

                // Linear acceleration
                const Eigen::Vector2f lin_acc_W = (lin_vel_W - m_PrevLinVel_W) / dt;
                const Eigen::Vector2f lin_acc_B = Eigen::Vector2f{
                    cosf(yaw) * lin_acc_W.x() + sinf(yaw) * lin_acc_W.y(),
                    -sinf(yaw) * lin_acc_W.x() + cosf(yaw) * lin_acc_W.y()
                };

                // Linear acceleration - CARLA METHOD
                Eigen::Vector2f lin_acc_B_NEW(0.f, 0.f);
                if (dt < 0.5f)
                {
                    const Eigen::Vector2f Y2 = m_PrevPrevLocation;
                    const Eigen::Vector2f Y1 = m_PrevLocation;
                    const Eigen::Vector2f Y0 = m_VehicleState.x_hat.head(2);
                    const float H1 = dt;
                    const float H2 = m_PrevDt;

                    const float H1AndH2 = H2 + H1;
                    const Eigen::Vector2f A = Y1 / (H1 * H2);
                    const Eigen::Vector2f B = Y2 / (H2 * (H1AndH2));
                    const Eigen::Vector2f C = Y0 / (H1 * (H1AndH2));
                    Eigen::Vector2f lin_acc_W_NEW = -2.0f * (A - B - C);

                    lin_acc_B_NEW = Eigen::Vector2f{
                        cosf(yaw) * lin_acc_W_NEW.x() + sinf(yaw) * lin_acc_W_NEW.y(),
                        -sinf(yaw) * lin_acc_W_NEW.x() + cosf(yaw) * lin_acc_W_NEW.y()
                    };
                }
                /*spdlog::info("Acc OLD: {}", acc);
                spdlog::info("Acc    : {}, {}", lin_acc_B.x(), lin_acc_B.y());
                spdlog::info("Acc NEW: {}, {}", lin_acc_B_NEW.x(), lin_acc_B_NEW.y());
                spdlog::info("---");*/

                m_PrevPrevLocation = m_PrevLocation;
                m_PrevLocation = m_VehicleState.x_hat.head(2);
                m_PrevDt = dt;



                float ang_vel = 0;
                if (tanf(steering_angle) != 0)
                {
                    const float vehicle_length = 3.8f;

                    // Instantaneous Center of Rotation
                    const float radius_icr = vehicle_length / tanf(steering_angle);
                    const Eigen::Vector2f radius = Eigen::Vector2f{
                        radius_icr * cosf(yaw),
                        radius_icr * sinf(yaw)
                    };

                    // Angular velocity
                    ang_vel = (radius.x() * lin_vel_B.y() - radius.y() * lin_vel_B.x()) / (radius.x() * radius.x() + radius.y() * radius.y());
                }
                
                
                // Speed
                //const float dist_traveled = CGeometry::euclidean_dist(Eigen::Vector2f(m_VehicleState.x_hat.head(2)), Eigen::Vector2f(m_PrevState.x_hat.head(2)));
                //const float speed = dist_traveled / dt;



                // DEBUG
                //Eigen::Vector3f euler_quat = quat.to_euler_ZYX();
                //spdlog::info("Given heading: {}", yaw * RAD2DEG);
                //spdlog::info("Quat euler : {}, {}, {}\n", euler_quat.x() * RAD2DEG, euler_quat.y() * RAD2DEG, euler_quat.z() * RAD2DEG);

                // Compute Acceleration
                // 2nd derivative of the polynomic (quadratic) interpolation using the point in current time and two previous steps:
                // d2[i] = -2.0*(y1/(h1*h2)-y2/((h2+h1)*h2)-y0/(h1*(h2+h1)))
                //const float acc = (m_VehicleState.x_hat[2] - m_PrevState.x_hat[2]) / dt;
                
                

                


                /*
                const float lin_vel_X_W = (m_VehicleState.x_hat[0] - m_PrevState.x_hat[0]) / dt;
                const float lin_vel_Y_W = (m_VehicleState.x_hat[1] - m_PrevState.x_hat[1]) / dt;

                const float lin_vel_X_veh = cosf(yaw) * lin_vel_X_W + sinf(yaw) * lin_vel_Y_W;
                const float lin_vel_Y_veh = -sinf(yaw) * lin_vel_X_W + cosf(yaw) * lin_vel_Y_W;

                const float acc_X = (lin_vel_X_veh - m_PrevLinVel_X) / dt;
                const float acc_Y = (lin_vel_Y_veh - m_PrevLinVel_Y) / dt;
                
                spdlog::info("lin acc: {},\t{}", acc_X, acc_Y);
                */


                // TODO: fix this
                // this is correct only if the linear acc is 0
                //acc = 0.f;
                const float ang_vel_X = 0.f;
                const float ang_vel_Y = 0.f;
                //const float ang_vel_Z = ((yaw - m_PrevState.x_hat[3]) * RAD2DEG) / dt;
                //const float ang_vel_Z = (yaw - m_PrevState.x_hat[3]) / dt;
                //const float ang_vel_Z = m_VehicleState.x_hat[2] / radius_icr;
                const float ang_vel_Z = ang_vel;

                

                //const float linear_vel = ang_vel_Z * radius_icr;

                // Compute magnetometer
                const float magnetX = cosf(yaw);
                const float magnetY = sinf(yaw);
                const float magnetZ = 0.f;

                // Store data
                // TODO: redo
                //m_FilterOutputImu.acc.x() = acc; // lin_acc_B_NEW.x();
                //m_FilterOutputImu.acc.y() = 0.f; // lin_acc_B_NEW.y();
                //m_FilterOutputImu.acc.z() = -GRAVITY;
                //m_FilterOutputImu.gyro.x() = ang_vel_X;
                //m_FilterOutputImu.gyro.y() = ang_vel_Y;
                //m_FilterOutputImu.gyro.z() = ang_vel_Z;
                //m_FilterOutputImu.magnet.x() = magnetX;
                //m_FilterOutputImu.magnet.y() = magnetY;
                //m_FilterOutputImu.magnet.z() = magnetZ;
                //m_FilterOutputImu.quat = quat;

                m_PrevState = m_VehicleState;
                m_PrevLinVel_W = lin_vel_W;

                //spdlog::info("---");

                m_lastTsState = ts;
                bReturn = true;
            }
        }
    }
    else if (m_ApiType == CImuUtils::Imu_API_MPU9250)
    {
        CycImu imu;
        if (m_ImuMPU9250_API.read(imu))
        {
            m_ImuCache.emplace_back(imu);
            bReturn = true;
        }
        else
        {
            bReturn = false;
            spdlog::error("Filter [{}-{}]: {}::process(): MPU9520 unable to read IMU data.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
        }
    }
    else if (m_ApiType == CImuUtils::Imu_API_BRICK)
    {
        const auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        if ((ts - m_lastTsState) > 10)
        {
            m_lastTsState = ts;

            m_ImuBrick_API.lock();
            const auto ego_data = m_ImuBrick_API.getData();
            m_ImuBrick_API.unlock();

            CycImu imu;
            imu.acc.x() = ego_data.acceleration.x;
            imu.acc.y() = ego_data.acceleration.y;
            imu.acc.z() = ego_data.acceleration.z;
            imu.gyro.x() = ego_data.angular_velocity.x;
            imu.gyro.y() = ego_data.angular_velocity.y;
            imu.gyro.z() = ego_data.angular_velocity.z;
            imu.magnet.x() = ego_data.magnetic_field.x;
            imu.magnet.y() = ego_data.magnetic_field.y;
            imu.magnet.z() = ego_data.magnetic_field.z;
            //imu.quat.update(ego_data.quaternion.x, ego_data.quaternion.y, ego_data.quaternion.z, ego_data.quaternion.w);

            m_ImuCache.emplace_back(imu);

            bReturn = true;
        }
    }
#ifdef __ANDROID_API__
    else if (m_ApiType == Imu_API_ANDROID)
    {
        m_ndkSensor->updateAllSensorsData();

        // TODO: replace roll, pitch, yaw with quaternions

        AccelerometerData accData;
        MagnetometerData magData;
        GyroscopeData gyrData;
        RotationSensorData rotData;
        m_ndkSensor->getAccelerometerData(accData);
        m_ndkSensor->getMagnetometerData(magData);
        m_ndkSensor->getGyroscopeData(gyrData);
        m_ndkSensor->getRotationSensorData(rotData);

        m_FilterOutputImu.acc_x = accData.x;
        m_FilterOutputImu.acc_y = accData.y;
        m_FilterOutputImu.acc_z = accData.z;
        m_FilterOutputImu.gyro_x = gyrData.x;
        m_FilterOutputImu.gyro_y = gyrData.y;
        m_FilterOutputImu.gyro_z = gyrData.z;
        m_FilterOutputImu.magnet_x = magData.x;
        m_FilterOutputImu.magnet_y = magData.y;
        m_FilterOutputImu.magnet_z = magData.z;
        m_FilterOutputImu.roll_x = rotData.roll;
        m_FilterOutputImu.pitch_y = rotData.pitch;
        m_FilterOutputImu.yaw_z = rotData.yaw;

        bReturn = true;
    }
#endif
#ifdef ENABLE_UNITREEA1
    else if (m_InterfaceType == Imu_UNITREEA1_INTERFACE)
    {
        auto rpy = CUnitreeA1::instance().getImuEuler();
        auto acc = CUnitreeA1::instance().getImuAccelerometer();
        auto gyro = CUnitreeA1::instance().getImuGyroscope();
        auto magnet = CUnitreeA1::instance().getImuMagnetometer();

        m_FilterOutputImu.acc_x = acc[0];
        m_FilterOutputImu.acc_y = acc[1];
        m_FilterOutputImu.acc_z = acc[2];
        m_FilterOutputImu.gyro_x = gyro[0];
        m_FilterOutputImu.gyro_y = gyro[1];
        m_FilterOutputImu.gyro_z = gyro[2];
        m_FilterOutputImu.magnet_x = magnet[0];
        m_FilterOutputImu.magnet_y = magnet[1];
        m_FilterOutputImu.magnet_z = magnet[2];
        m_FilterOutputImu.roll = rpy[0];
        m_FilterOutputImu.pitch = rpy[1];
        m_FilterOutputImu.yaw = rpy[2];

        bReturn = true;
    }
#endif
    else if (m_ApiType == CImuUtils::Imu_API_UNITY)
    {
        while (!m_ImuUnity_API.process())
        {
            if (!isRunning())
                return true;
                
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        m_ImuCache.emplace_back(m_ImuUnity_API.get_imu());
        bReturn = true;
    }
    else if (m_ApiType == CImuUtils::Imu_API_REALSENSE)
    {
        m_RealSense->get_imu_cache(m_ImuCache);
        bReturn = true;
    }
    else
    {
        spdlog::error("Filter [{}-{}]: {}::process(): Interface type undefined. Disabling filter.", getFilterKey().nCoreID, getFilterKey().nFilterID, typeid(*this).name());
        this->disable();
        bReturn = false;
    }

    // Apply Madgwick filter for attitude estimation
    //if (bReturn && m_bUseMadgwick)
    //    m_pMadgwick->update(dt, m_FilterOutputImu);

    // DEBUG
    //spdlog::info("heading: {}", m_VehicleState.x_hat[3] * RAD2DEG);
    //const Eigen::Vector3f euler(CQuaternion(m_FilterOutputImu.quat_x, m_FilterOutputImu.quat_y, m_FilterOutputImu.quat_z, m_FilterOutputImu.quat_w).to_euler_ZYX());
    //spdlog::info("Estimated euler: {}, {}, {}\n", euler.x()* RAD2DEG, euler.y()* RAD2DEG, euler.z()* RAD2DEG);
    
    // Update filter pose
    //this->m_pSensorModel->updatePose(CPose(0.f, 0.f, 3.f, m_FilterOutputImu.quat.x(), m_FilterOutputImu.quat.y(), m_FilterOutputImu.quat.z(), m_FilterOutputImu.quat.w()));
    
    if (bReturn)
    {
        // After succesfully opening the COM-port, connect this function to a timer.
        // The timer should have an interval of approx. 100 milliSeconds.
        // https://www.teuniz.net/RS-232/
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //std::this_thread::sleep_for(std::chrono::microseconds(1));
        updateData(m_ImuCache);
    }
    
    return bReturn;
}

void CImuFilter::loadFromDatastream(const std::string& datastream_entry, const std::string& db_root_path)
{
#ifndef __ANDROID_API__
    // Set the processing flag
    m_bIsProcessing = true;

    // Update sensor pose
    CPose pose;
    if (getPose(pose))
        this->m_pSensorModel->updatePose(pose * this->m_pSensorModel->extrinsics());

    csv::reader::row row;
    row.parse_line(datastream_entry, ',');
    enum { TS_STOP, SAMPLING_TIME, IMU_PATH, NUM };
    if (row.size() < NUM)
    {
        spdlog::error("{}: Wrong number of columns. {} provided, but expected at least {}.", typeid(*this).name(), row.size(), NUM + 1);
        return;
    }

    const auto tTimestampStop = row.get<CyC_TIME_UNIT>(TS_STOP);
    const auto tSamplingTime = row.get<CyC_TIME_UNIT>(SAMPLING_TIME);
    const auto tTimestampStart = tTimestampStop - tSamplingTime;

    // IMU cache
    CycImus imu_cache;
    if (row.get<std::string>(IMU_PATH).size() != 0)
    {
        const auto imu_path = db_root_path + row.get<std::string>(IMU_PATH);
        csv::reader imu_reader;
        if (!imu_reader.open(imu_path))
        {
            spdlog::error("{}: Failed to open imu cache path '{}'.", typeid(*this).name(), imu_path);
        }
        else
        {
            const csv::reader::row& imu_row = imu_reader.get_row();
            while (imu_reader.next_row())
            {
                CycImu imu;
                imu.timestamp = imu_row.get<CyC_TIME_UNIT>(0);
                imu.acc.x() = imu_row.get<float>(1);
                imu.acc.y() = imu_row.get<float>(2);
                imu.acc.z() = imu_row.get<float>(3);
                imu.gyro.x() = imu_row.get<float>(4);
                imu.gyro.y() = imu_row.get<float>(5);
                imu.gyro.z() = imu_row.get<float>(6);
                imu_cache.emplace_back(imu);
            }
        }
    }
    m_ImuCache = imu_cache;

    updateData(m_ImuCache, std::unordered_map<CycDatablockKey, CyC_TIME_UNIT>(), tTimestampStart, tTimestampStop, tSamplingTime);

    //this->m_pSensorModel->updatePose(CPose(0.f, 0.f, 3.f, m_FilterOutputImu.quat.x(), m_FilterOutputImu.quat.y(), m_FilterOutputImu.quat.z(), m_FilterOutputImu.quat.w()));
    
    // Reset the processing flag
    m_bIsProcessing = false;
#endif
}

bool CImuFilter::getPose(CPose& _out_pose)
{
    if (m_pPoseDataFilter == nullptr)
        return false;

    bool bReturn = false;

    // Update pose from filter data, depending on the filter type (vehicle or drone)
    const CyC_TIME_UNIT readTsPose = m_pPoseDataFilter->getTimestampStop();

    if (readTsPose > m_lastReadTsPose)
    {
        m_lastReadTsPose = readTsPose;

        CycState state;
        if (m_pPoseDataFilter->getData(state))
        {
            bReturn = CFilterUtils::state2pose(state, m_pPoseDataFilter->getFilterType(), _out_pose);
        }
    }

    return bReturn;
}

CImuUtils::ImuApiType CImuFilter::StringToEnumType(const std::string& str_type)
{
    if (str_type.compare("sim") == 0)
        return CImuUtils::Imu_API_SIM;
    else if (str_type.compare("MPU9250") == 0)
        return CImuUtils::Imu_API_MPU9250;
    else if (str_type.compare("brick") == 0)
        return CImuUtils::Imu_API_BRICK;
    else if (str_type.compare("android") == 0)
        return CImuUtils::Imu_API_ANDROID;
    else if (str_type.compare("unitree_a1") == 0)
        return CImuUtils::Imu_API_UNITREEA1;
    else if (str_type.compare("unity") == 0)
        return CImuUtils::Imu_API_UNITY;
    else if (str_type.compare("realsense") == 0)
        return CImuUtils::Imu_API_REALSENSE;
    return CImuUtils::Imu_API_SIM;
}
