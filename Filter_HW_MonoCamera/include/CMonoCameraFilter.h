// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CMONOCAMERA_H_
#define CMONOCAMERA_H_

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "CCR_TYPES.h"
#include "CBaseCcrFilter.h"
#include <os/CFilterUtils.h>
#include "sensors/CPinholeCameraSensorModel.h"
#include <csv_reader.h>
#include "vision/CProjectiveGeometry.h"
#include "vision/CImageDisplayUtils.h"
#include "vision/CTriangulation.h"

#if !defined(RASPBERRY_PI)
#include "interfaces/unity/CMonoCameraUnityInterface.h"
#endif

#ifdef RASPBERRY_PI
#include "interfaces/raspi/CMonoCameraRaspiInterface.h"
#endif

#ifdef __ANDROID_API__
#include <CNativeCameraInterface.h>
#endif

class CMonoCameraFilter : public CBaseCcrFilter
{
public:
    enum MonoCameraInterfaceType
    {
        MONO_CAMERA_SIM_INTERFACE = 0,
        MONO_CAMERA_USB_INTERFACE = 1,
        MONO_CAMERA_UNITY_INTERFACE = 2,
        MONO_CAMERA_RASPI_INTERFACE = 3,
        MONO_CAMERA_ARDRONE_INTERFACE = 4,
        MONO_CAMERA_ANAFI_INTERFACE = 5
    };

public:
    explicit CMonoCameraFilter(CcrDatablockKey key);
    explicit CMonoCameraFilter(ConfigFilterParameters params);
    ~CMonoCameraFilter() override;

    bool enable() override;
    bool disable() override;

private:
    void        init();
    bool	process() override;
    void        loadFromDatastream(const std::string& _datastream_entry, const std::string& _db_root_path) override;
    static int  StringToEnumType(const std::string& _str_type);

private:
    bool          m_bRectifyImages;

    // Updates pose of the camera
    CBaseCcrFilter*   m_pPoseDataFilter = nullptr;
    CCR_TIME_UNIT     m_lastReadTsPose = 0;

#ifdef __ANDROID_API__
private:
    NDKCamera* m_ndkCamera;
    CCR_UINT m_androidFrameId = 0;
    CCR_UINT m_lastProcessedAndroidFrameId = 0;
#else
private:
    cv::VideoCapture	      m_VideoCapture;
#endif

#if !defined(RASPBERRY_PI)
    CMonoCameraUnityInterface m_UnityInterface;
#endif

#ifdef RASPBERRY_PI
    CMonoCameraRaspiInterface m_RaspiInterface;
#endif

    CCR_INT		m_nDeviceID;
    CCR_INT		m_nApiID;
    CCR_INT     m_InterfaceType = 0;

    // Voxels (in world coordinates system) used for simulated camera
    CcrVoxels   m_Voxels;
};

#endif /* CMONOCAMERA_H_ */
