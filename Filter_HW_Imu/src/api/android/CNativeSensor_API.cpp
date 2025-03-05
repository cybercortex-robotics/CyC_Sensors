// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CNativeSensor_API.h"

#ifdef __ANDROID_API__

#define LOG_TAG "NativeSensorInterface"
#define RETURN_IF_NULLPTR(var, code) if ((var) == nullptr) return code;
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

/*
 * AcquireASensorManagerInstance(void)
 *    Workaround AsensorManager_getInstance() deprecation false alarm
 *    for Android-N and before, when compiling with NDK-r15
 */
#include <dlfcn.h>
const char*  kPackageName = "ai.cybercortex.testervisioncore_android";
ASensorManager* AcquireASensorManagerInstance(void)
{
    typedef ASensorManager *(*PF_GETINSTANCEFORPACKAGE)(const char *name);
    void* androidHandle = dlopen("libandroid.so", RTLD_NOW);

    PF_GETINSTANCEFORPACKAGE getInstanceForPackageFunc = (PF_GETINSTANCEFORPACKAGE)
        dlsym(androidHandle, "ASensorManager_getInstanceForPackage");
    if (getInstanceForPackageFunc)
    {
        return getInstanceForPackageFunc(kPackageName);
    }

    typedef ASensorManager *(*PF_GETINSTANCE)();
    PF_GETINSTANCE getInstanceFunc = (PF_GETINSTANCE)
        dlsym(androidHandle, "ASensorManager_getInstance");
    // by all means at this point, ASensorManager_getInstance should be available
    assert(getInstanceFunc);
    return getInstanceFunc();
}

static void toEulerAngle(const double qw, const double qx, const double qy, const double qz, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (qw * qx + qy * qz);
    double cosr = +1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (qw * qz + qx * qy);
    double cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny, cosy);
}

void NDKSensor::init()
{
    sensorManager = AcquireASensorManagerInstance();
    assert(sensorManager != NULL);

    accelerometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    assert(accelerometer != NULL);

    magnetometer = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);
    assert(magnetometer != NULL);

    gyroscope = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
    assert(magnetometer != NULL);

    rotationVirtualSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ROTATION_VECTOR);
    assert(magnetometer != NULL);

    sensorsEventQueue = ASensorManager_createEventQueue(sensorManager, ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS),
                                                              LOOPER_ID_USER, NULL, NULL);
    assert(sensorsEventQueue != NULL);


    auto status = ASensorEventQueue_enableSensor(sensorsEventQueue, accelerometer);
    assert(status >= 0);
    status = ASensorEventQueue_enableSensor(sensorsEventQueue, magnetometer);
    assert(status >= 0);
    status = ASensorEventQueue_enableSensor(sensorsEventQueue, gyroscope);
    assert(status >= 0);
    status = ASensorEventQueue_enableSensor(sensorsEventQueue, rotationVirtualSensor);
    assert(status >= 0);

    status = ASensorEventQueue_setEventRate(sensorsEventQueue,
                                            accelerometer,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    status = ASensorEventQueue_setEventRate(sensorsEventQueue,
                                            magnetometer,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    status = ASensorEventQueue_setEventRate(sensorsEventQueue,
                                            gyroscope,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    status = ASensorEventQueue_setEventRate(sensorsEventQueue,
                                            rotationVirtualSensor,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);


    (void)status;   //to silent unused compiler warning

    generateXPos();
}

void NDKSensor::generateXPos()
{
    for (auto i = 0; i < SENSOR_HISTORY_LENGTH; i++) {
        float t = static_cast<float>(i) / static_cast<float>(SENSOR_HISTORY_LENGTH - 1);
        xPos[i] = -1.f * (1.f - t) + 1.f * t;
    }
}

void NDKSensor::updateAllSensorsData()
{
    ALooper_pollAll(0, NULL, NULL, NULL);
    ASensorEvent event;
    float a = SENSOR_FILTER_ALPHA;

    while (ASensorEventQueue_getEvents(sensorsEventQueue, &event, 1) > 0)
    {
        if(event.type == ASENSOR_TYPE_ACCELEROMETER)
        {
            accSensorDataFilter.x = a * event.acceleration.x + (1.0f - a) * accSensorDataFilter.x;
            accSensorDataFilter.y = a * event.acceleration.y + (1.0f - a) * accSensorDataFilter.y;
            accSensorDataFilter.z = a * event.acceleration.z + (1.0f - a) * accSensorDataFilter.z;

            accSensorData[accSensorDataIndex] = accSensorDataFilter;
            accSensorData[SENSOR_HISTORY_LENGTH + accSensorDataIndex] = accSensorDataFilter;
            accSensorDataIndex = (accSensorDataIndex + 1) % SENSOR_HISTORY_LENGTH;
        }
        else if(event.type == ASENSOR_TYPE_MAGNETIC_FIELD)
        {
            magSensorDataFilter.x = a * event.magnetic.x + (1.0f - a) * magSensorDataFilter.x;
            magSensorDataFilter.y = a * event.magnetic.y + (1.0f - a) * magSensorDataFilter.y;
            magSensorDataFilter.z = a * event.magnetic.z + (1.0f - a) * magSensorDataFilter.z;

            magSensorData[magSensorDataIndex] = magSensorDataFilter;
            magSensorData[SENSOR_HISTORY_LENGTH + magSensorDataIndex] = magSensorDataFilter;
            magSensorDataIndex = (magSensorDataIndex + 1) % SENSOR_HISTORY_LENGTH;
        }
        else if(event.type == ASENSOR_TYPE_GYROSCOPE)
        {
            gyrSensorDataFilter.x = a * event.acceleration.x + (1.0f - a) * gyrSensorDataFilter.x;
            gyrSensorDataFilter.y = a * event.acceleration.y + (1.0f - a) * gyrSensorDataFilter.y;
            gyrSensorDataFilter.z = a * event.acceleration.z + (1.0f - a) * gyrSensorDataFilter.z;

            gyrSensorData[gyrSensorDataIndex] = gyrSensorDataFilter;
            gyrSensorData[SENSOR_HISTORY_LENGTH + gyrSensorDataIndex] = gyrSensorDataFilter;
            gyrSensorDataIndex = (gyrSensorDataIndex + 1) % SENSOR_HISTORY_LENGTH;

            /*gyrSensorData[gyrSensorDataIndex].x = event.acceleration.x;
            gyrSensorData[gyrSensorDataIndex].x = event.acceleration.y;
            gyrSensorData[gyrSensorDataIndex].x = event.acceleration.z;

            gyrSensorDataIndex = gyrSensorDataIndex + 1;*/
        }
        else if(event.type == ASENSOR_TYPE_ROTATION_VECTOR)
        {
            // Rotation vector returns quaternion units
            // In Android's unit quaternion, the Scalar component of the rotation vector is the last variable (of the four)
            // Taken from documentation https://source.android.com/devices/sensors/sensor-types#rotation_vector
            float qx = event.data[0];
            float qy = event.data[1];
            float qz = event.data[2];
            float qw = event.data[3];

            double roll, pitch, yaw;
            toEulerAngle(qw, qx, qy, qz, roll, pitch, yaw);

            rotSensorDataFilter.roll = a * roll * RAD2DEG + (1.0f - a) * rotSensorDataFilter.roll;
            rotSensorDataFilter.pitch = a * pitch * RAD2DEG + (1.0f - a) * rotSensorDataFilter.pitch;
            rotSensorDataFilter.yaw = a * yaw * RAD2DEG + (1.0f - a) * rotSensorDataFilter.yaw;

            rotSensorData[rotSensorDataIndex] = rotSensorDataFilter;
            rotSensorData[SENSOR_HISTORY_LENGTH + rotSensorDataIndex] = rotSensorDataFilter;
            rotSensorDataIndex = (rotSensorDataIndex + 1) % SENSOR_HISTORY_LENGTH;
        }
    }
}

void NDKSensor::pause() {
    ASensorEventQueue_disableSensor(sensorsEventQueue, accelerometer);
}

void NDKSensor::resume() {
    ASensorEventQueue_enableSensor(sensorsEventQueue, accelerometer);
    auto status = ASensorEventQueue_setEventRate(sensorsEventQueue,
                                                 accelerometer,
                                                 SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);
}

void NDKSensor::getAccelerometerData(AccelerometerData& accData)
{
    accData = accSensorData[accSensorDataIndex];
}

void NDKSensor::getMagnetometerData(MagnetometerData& magData)
{
    magData = magSensorData[magSensorDataIndex];
}

void NDKSensor::getGyroscopeData(GyroscopeData& gyrData)
{
    gyrData = gyrSensorData[gyrSensorDataIndex];
}

void NDKSensor::getRotationSensorData(RotationSensorData& rotData)
{
    rotData = rotSensorData[rotSensorDataIndex];
}


#endif /* __ANDROID_API__ */