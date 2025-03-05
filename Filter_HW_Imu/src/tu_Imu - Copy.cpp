// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CyC_TYPES.h"
#include <vector>
#include <iostream>
#include <assert.h>
#include <iomanip>
#include "CMadgwick.h"
#include "CImuUtils.h"
#include "env/CQuaternion.h"
#include "os/CFileUtils.h"
#include "os/CDataBlockReader.h"
#include "sensors/CImuSensorModel.h"
#include "control/CStateSpaceModelVehicle.h"
#include <math/CImuPreintegrator.h>
#include <math/CImuStatePredictor.h>
#include "integration_base.h"

#include "api/MPU9250/CImuMPU9250_API.h"
#include "../ccr_filters/Filters_Hardware/Filter_HW_RGBDCamera/src/CRealSense2API.h"

std::unique_ptr<CImuSensorModel> pImuSensorModel = nullptr;

// State estimation variables
Eigen::Vector3f m_Position = Eigen::Vector3f(-71.106003f, 77.423599f, 0.f);
Eigen::Vector3f m_LinearVelocity = Eigen::Vector3f::Zero();

// Attitude estimation algorithm (Madgwick filter)
CMadgwick m_Madgwick(true);

// APIs
CImuUtils::ImuApiType m_ApiType = CImuUtils::Imu_API_SIM;
CImuMPU9250_API m_ImuMPU9250_API;
CRealSense2API m_RealSense_API;

void showUsage()
{
    printf("\nUsage:\n"
        "tu_Imu calibration_file\n\n"
        "  --sim                # Simulated data [default]\n"
        "  --mpu                # Live MPU data acquisition (mpu port_nr)\n"
        "  --rs                 # Live RealSense data acquisition (mpu port_nr)\n"
        "  --db                 # CyberCortex.AI dataset\n"
        "  --imu                # IMU filter ID in database\n"
        "  --state              # State filter ID in database\n"
        "\n"
        "eg: tu_Imu --db c:/data/nuscenes --imu 8 calibration.cal\n");
    exit(1);
}

void update(const float& _dt, CycImu& _imu);

void dummy_estimation();
void api_mpu9250_estimation(const CyC_INT& _port_nr);
void database_estimation(const std::string& _ccr_db_folder, const CyC_INT& _imu_filter_id, const CyC_INT& _state_filter_id = -1);

int main(int argc, char** argv)
{
    // Do not use scientific notation
    std::cout << std::fixed;
    std::cout << std::setprecision(6);

    CyC_INT     nApiPortNr = -1;
    bool    bUseDB = false;
    std::string  sFolderCcrDB;
    CyC_INT     nImuFilterID = -1;
    CyC_INT     nStateFilterID = -1;

    if (argc < 2)
    {
        showUsage();
        return EXIT_SUCCESS;
    }

    // Read arguments
    for (int i = 1; i < argc - 1; i++)
    {
        if (strcmp(argv[i], "--sim") == 0)
        {
            m_ApiType = CImuUtils::Imu_API_SIM;
            break;
        }
        else if (strcmp(argv[i], "--mpu") == 0)
        {
            m_ApiType = CImuUtils::Imu_API_MPU9250;
            nApiPortNr = atoi(argv[i + 1]);
            ++i;
            break;
        }
        else if (strcmp(argv[i], "--rs") == 0)
        {
            m_ApiType = CImuUtils::Imu_API_REALSENSE;
            break;
        }
        else if (strcmp(argv[i], "--db") == 0)
        {
            bUseDB = true;
            sFolderCcrDB = argv[i + 1];
            ++i;
            continue;
        }
        else if (strcmp(argv[i], "--imu") == 0)
        {
            nImuFilterID = atoi(argv[i + 1]);
            ++i;
            continue;
        }
        else if (strcmp(argv[i], "--state") == 0)
        {
            nStateFilterID = atoi(argv[i + 1]);
            ++i;
            continue;
        }

        printf("\nUnrecognized option : %s\n", argv[i]);
        showUsage();
    }

    // Load calibration data
    std::string sImuCalibFile = argv[argc - 1];
    if (!CFileUtils::FileExist(sImuCalibFile.c_str()))
    {
        std::cout << "ERROR: Calibration file not found. Exiting." << std::endl;
        return EXIT_FAILURE;
    }
    pImuSensorModel = std::make_unique<CImuSensorModel>(sImuCalibFile);

    // Check if a database has been given
    if (bUseDB)
    {
        if (!CFileUtils::FolderExist(sFolderCcrDB.c_str()))
        {
            spdlog::warn("CyberCortex.AI database not found. Exiting.");
            return EXIT_FAILURE;
        }
    }
    else
    {
        if (m_ApiType == CImuUtils::Imu_API_MPU9250)
        {
            if (!m_ImuMPU9250_API.start(nApiPortNr))
            {
                spdlog::error("Can't open MPU9250 port number {}", nApiPortNr);
                return EXIT_FAILURE;
            }
        }
        else if (m_ApiType == CImuUtils::Imu_API_REALSENSE)
        {
            if (!m_RealSense_API.start())
            {
                spdlog::error("Can't open RealSense2 camera");
                return EXIT_FAILURE;
            }
        }
    }
    
    if (bUseDB)
    {
        database_estimation(sFolderCcrDB, nImuFilterID, nStateFilterID);
    }
    else
    {
        CycImus imu_cache;
        CyC_INT counter = 0;

        bool first_imu = false;
        Eigen::Vector3f g(0.f, 0.f, GRAVITY);
        Eigen::Vector3f Bas(0.f, 0.f, 0.f), Bgs(0.f, 0.f, 0.f);
        Eigen::Vector3f acc_0(0.f, 0.f, 0.f), gyro_0(0.f, 0.f, 0.f);
        Eigen::Matrix3f Rs = Eigen::Matrix3f::Identity();
        Eigen::Vector3f Ps(0.f, 0.f, 0.f), Vs(0.f, 0.f, 0.f);

        CycImu imu_curr, imu_prev;
        while (true) // this loop should repeat each time new gyroscope data is available
        {
            imu_prev = imu_curr;
            static Bias bias{};
            bias.bax = 0;
            bias.bwx = -0;
            bias.bay = -0;
            bias.bwy = -0;
            bias.baz = -0;
            bias.bwz = 0;

            static const Sophus::SE3<float> identity;
            static const float ng = 1.3451432437037339e-01f;
            static const float na = 2.2755060933428370e-02f;
            static const float ngw = 4.7066764012620192e-03f;
            static const float naw = 5.3851103950851254e-04f;
            static Calib calib(identity, ng, na, ngw, naw);

            static CImuPreintegrator integ(bias, calib);
            static CImuStatePredictor pred(bias);

            if (m_ApiType == CImuUtils::Imu_API_SIM)
            {
                imu_curr.timestamp = CTimer::now();
                imu_curr.acc.z() = GRAVITY;

                if (counter == 0)
                    imu_curr.acc.x() = 1.0f;
                else if (counter == 60)
                    imu_curr.acc.x() = -1.0f;
                else
                    imu_curr.acc.x() = 0.0f;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            else if (m_ApiType == CImuUtils::Imu_API_MPU9250)
            {
                m_ImuMPU9250_API.read(imu_curr);
                Eigen::Vector3f euler_ccr_DEG = imu_curr.quat.to_euler_ZYX() * RAD2DEG;
            
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else if (m_ApiType == CImuUtils::Imu_API_REALSENSE)
            {
                m_RealSense_API.grab_imu(imu_curr);
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                spdlog::error("Unrecognized API '{}'", m_ApiType);
            }


            if (!first_imu)
            {
                first_imu = true;
                acc_0 = imu_curr.acc;
                gyro_0 = imu_curr.gyro;
            }
            else
            {
                float dt = (imu_curr.timestamp - imu_prev.timestamp) * MSEC2SEC;
                Eigen::Vector3f un_acc_0 = Rs * (acc_0 - Bas) - g;
                Eigen::Vector3f un_gyr = 0.5 * (gyro_0 + imu_curr.gyro) - Bgs;
                Rs *= IntegrationBase::deltaQ(un_gyr * dt).toRotationMatrix();
                Eigen::Vector3f un_acc_1 = Rs * (imu_curr.acc - Bas) - g;
                Eigen::Vector3f un_acc = 0.5 * (un_acc_0 + un_acc_1);
                Ps += dt * Vs + 0.5 * dt * dt * un_acc;
                Vs += dt * un_acc;
                acc_0 = imu_curr.acc;
                gyro_0 = imu_curr.gyro;
            }

            if (counter % 10 == 0)
                std::cout << counter << ": pos: \t" << Ps.x() << "\t" << Ps.y() << "\t" << Ps.z() << std::endl;


            //update(dt, imu);
            imu_cache.emplace_back(imu_curr);

            if (counter % 10 == 0 && imu_cache.size() > 1)
            {
                /*for (size_t i = 1; i < imu_cache.size(); i++)
                {
                    dt = (imu_cache[i].timestamp - imu_cache[i-1].timestamp) * MSEC2SEC;
                    std::cout << "dt: " << dt << std::endl;

                    const Eigen::Vector3f accLastFrame = imu_cache[i-1].acc;
                    const Eigen::Vector3f angVelLastFrame = imu_cache[i-1].gyro;

                    const Eigen::Vector3f accCurrentFrame = imu_cache[i].acc;
                    const Eigen::Vector3f angVelCurrentFrame = imu_cache[i].gyro;

                    float tstep;
                    Eigen::Vector3f acc, angVel;
                    
                    if (i < (imu_cache.size() - 1))
                    {

                        acc = (accLastFrame + accCurrentFrame) * 0.5f;
                        angVel = (angVelLastFrame + angVelCurrentFrame) * 0.5f;
                        tstep = dt;
                    }
                    else if ((i > 0) && (i == (imu_cache.size() - 1)))
                    {
                        float tab = dt;
                        float tend = dt;
                        acc = (accLastFrame + accCurrentFrame -
                            (accCurrentFrame - accLastFrame) * (tend / tab)) * 0.5f;
                        angVel = (angVelLastFrame + angVelCurrentFrame -
                            (angVelCurrentFrame - angVelLastFrame) * (tend / tab)) * 0.5f;
                        tstep = dt;
                    }
                    else if ((i == 0) && (i == (imu_cache.size() - 1)))
                    {
                        acc = accLastFrame;
                        angVel = angVelLastFrame;
                        tstep = dt;
                    }

                    integ.IntegrateNewMeasurement(acc, angVel, tstep);
                }

                pred.predict(integ);
                integ.Initialize(bias);

                const auto pos = pred.GetImuPosition();
                const auto att = CPose::R2euler(pred.GetImuRotation()) * RAD2DEG;
                const auto vel = pred.GetVelocity();

                std::cout << counter << ": pos: \t" << pos.x() << "\t" << pos.y() << "\t" << pos.z() << std::endl;
                std::cout << counter << ": euler: \t" << att.x() << "\t" << att.y() << "\t" << att.z() << std::endl;
                std::cout << counter << ": vel: \t" << vel.x() << "\t" << vel.y() << "\t" << vel.z() << std::endl;*/

                imu_cache.clear();
            }

            ++counter;

            if (counter > 100000)
                break;
        }
    }

    if (!bUseDB)
    {
        if (m_ApiType == CImuUtils::Imu_API_MPU9250)
            m_ImuMPU9250_API.stop();
        else if (m_ApiType == CImuUtils::Imu_API_REALSENSE)
            m_RealSense_API.stop();
    }

    return EXIT_SUCCESS;
}

void database_estimation(const std::string& _ccr_db_folder, const CyC_INT& _imu_filter_id, const CyC_INT& _state_filter_id)
{
    if (_imu_filter_id < 0)
    {
        spdlog::error("tu_Imu database_estimation(): imu filter ID required for attitude estimation.");
        return;
    }

    CyC_TIME_UNIT ts_curr_imu = 0;
    CyC_TIME_UNIT ts_state = 0;

    CycImu imu;
    CycState state;
    state.x_hat = Eigen::VectorXf::Zero(NO_VEHICLE_MODEL_STATES);

    std::vector<CyC_INT> filter_ids;
    std::vector<CDataBlockReader::DatablockData> row_data;

    CyC_INT filter_id_counter = 0;
    CyC_INT imu_filter_idx = -1;
    CyC_INT state_filter_idx = -1;

    // Add images datastreams
    if (_imu_filter_id != -1)
    {
        filter_ids.emplace_back(_imu_filter_id);

        CDataBlockReader::DatablockData imgs_data(_imu_filter_id);
        row_data.emplace_back(imgs_data);

        imu_filter_idx = filter_id_counter;
        ++filter_id_counter;
    }

    // Add state datastream
    if (_state_filter_id != -1)
    {
        filter_ids.emplace_back(_state_filter_id);

        CDataBlockReader::DatablockData state_data(_state_filter_id);
        row_data.emplace_back(state_data);

        state_filter_idx = filter_id_counter;
        ++filter_id_counter;
    }

    CDataBlockReader DataBlockReader(_ccr_db_folder, filter_ids);

    CyC_TIME_UNIT ts_prev_imu = 0;
    
    CyC_INT counter = 0;
    bool first_run = true;
    while (DataBlockReader.getNextRow(row_data))
    {
        std::cout << "--- counter = " << counter << " --- " << std::endl;

        if (_state_filter_id != -1 && row_data[state_filter_idx].timestamp > 0)
        {
            ts_state = row_data[state_filter_idx].timestamp;
            state = std::get<CycState>(row_data[state_filter_idx].data);

            std::cout << "Measured state:  " << state.x_hat[0] << "\t" << state.x_hat[1] << "\t" << state.x_hat[2] << "\t" << state.x_hat[3] * RAD2DEG << std::endl;
        }

        if (_imu_filter_id != -1 && row_data[imu_filter_idx].timestamp > 0)
        {
            ts_curr_imu = row_data[imu_filter_idx].timestamp;
            const float dt = (ts_curr_imu - ts_prev_imu) * MSEC2SEC;
            ts_prev_imu = ts_curr_imu;

            if (first_run)
            {
                first_run = false;
            }
            else
            {
                imu = std::get<CycImu>(row_data[imu_filter_idx].data);
                update(dt, imu);
            }

            Eigen::Vector3f imu_euler_rpy(imu.quat.to_euler_ZYX());
            std::cout << "Estimated state: " << m_Position.x() << "\t" << m_Position.y() << "\t" << m_LinearVelocity.x() << "\t" << imu_euler_rpy.z() * RAD2DEG << std::endl;
            std::cout << "dt = " << dt << std::endl;
        }

        std::cout << std::endl;

        ++counter;

        if (counter > 1000)
            break;
    }
}

void update(const float& _dt, CycImu& _imu)
{
    // R from Fusion Madgwick library
    m_Madgwick.update(_dt, _imu);

    Eigen::Matrix3f R = _imu.quat.to_rotmat();

    const Eigen::Vector3f acc(_imu.acc.x(), _imu.acc.y(), (_imu.acc.z() + GRAVITY));

    // acc and vel are ego coordinates
    m_LinearVelocity += acc * _dt;

    // position is in global coordinates
    const Eigen::Vector3f vRotated = R * m_LinearVelocity;
    m_Position += vRotated * _dt;
}
