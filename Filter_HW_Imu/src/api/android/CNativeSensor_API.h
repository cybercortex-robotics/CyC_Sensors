// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CNativeSensor_API_H_
#define CNativeSensor_API_H_

#ifdef __ANDROID_API__
#include <string>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <CyC_TYPES.h>
#include <android/log.h>
#include <android/asset_manager_jni.h>
#include <android/sensor.h>

// Rotation vector returns quaternion units
// In Android's unit quaternion, the Scalar component of the rotation vector is the last variable (of the four)
// Taken from documentation https://source.android.com/devices/sensors/sensor-types#rotation_vector
// https://developer.android.com/guide/topics/sensors/sensors_motion#sensors-motion-rotate

const int LOOPER_ID_USER = 3;
const int SENSOR_HISTORY_LENGTH = 100;
const int SENSOR_REFRESH_RATE_HZ = 100;
constexpr int32_t SENSOR_REFRESH_PERIOD_US = int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);
const float SENSOR_FILTER_ALPHA = 0.1f;

struct AccelerometerData
{
    float x;
    float y;
    float z;
};

struct MagnetometerData
{
    float x;
    float y;
    float z;
};

struct GyroscopeData
{
    float x;
    float y;
    float z;
};

struct RotationSensorData
{
    float roll;
    float pitch;
    float yaw;
};

class NDKSensor {
    ASensorManager *sensorManager;
    const ASensor *accelerometer;
    const ASensor *magnetometer;
    const ASensor *gyroscope;
    const ASensor *rotationVirtualSensor;
    ASensorEventQueue *sensorsEventQueue;

    float xPos[SENSOR_HISTORY_LENGTH];
    int accSensorDataIndex;
    int magSensorDataIndex;
    int gyrSensorDataIndex;
    int rotSensorDataIndex;
    AccelerometerData accSensorData[SENSOR_HISTORY_LENGTH*2];
    AccelerometerData accSensorDataFilter;
    MagnetometerData magSensorData[SENSOR_HISTORY_LENGTH*2];
    MagnetometerData magSensorDataFilter;
    GyroscopeData gyrSensorData[SENSOR_HISTORY_LENGTH*2];
    GyroscopeData gyrSensorDataFilter;
    RotationSensorData rotSensorData[SENSOR_HISTORY_LENGTH*2];
    RotationSensorData rotSensorDataFilter;

 public:
    NDKSensor() :
            accSensorDataIndex(0),
            magSensorDataIndex(0),
            gyrSensorDataIndex(0),
            rotSensorDataIndex(0)
    {
        init();
    }

    void init();
    void generateXPos();
    void updateAllSensorsData();
    void pause();
    void resume();

    void getAccelerometerData(AccelerometerData& accData);
    void getMagnetometerData(MagnetometerData& magData);
    void getGyroscopeData(GyroscopeData& gyrData);
    void getRotationSensorData(RotationSensorData& rotData);
};

#endif /* ANDROID_API */

#endif /* CNativeSensor_API_H_ */
