// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#ifndef CNATIVECAMERAINTERFACE_H_
#define CNATIVECAMERAINTERFACE_H_

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
#include <camera/NdkCameraCaptureSession.h>
#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraError.h>
#include <camera/NdkCameraManager.h>
#include <camera/NdkCameraMetadata.h>
#include <camera/NdkCameraMetadataTags.h>
#include <camera/NdkCameraWindowType.h>
#include <camera/NdkCaptureRequest.h>
#include <media/NdkImageReader.h>

/**
 * Range of Camera Exposure Time:
 *     Camera's capability range have a very long range which may be disturbing
 *     on camera. For this sample purpose, clamp to a range showing visible
 *     video on preview: 100000ns ~ 250000000ns
 */
static const uint64_t kMinExposureTime = static_cast<uint64_t>(1000000);
static const uint64_t kMaxExposureTime = static_cast<uint64_t>(250000000);

template <typename T>
class RangeValue {
public:
    T min_, max_;
    /**
     * return absolute value from relative value
     * value: in percent (50 for 50%)
     */
    T value(int percent) {
        return static_cast<T>(min_ + (max_ - min_) * percent / 100);
    }
    RangeValue() { min_ = max_ = static_cast<T>(0); }

    bool Supported(void) const { return (min_ != max_); }
};


class NDKCamera {
private:
    AImageReader* createJpegReader(int32_t width, int32_t height);
    AImageReader* createRgbReader(int32_t width, int32_t height);
    AImageReader* createYuvReader(int32_t width, int32_t height);
    ANativeWindow* createSurface();

    CyC_INT initAndroidCameraAPI();
    void disableAndroidCameraAPI();

    ACameraManager* cameraManager = nullptr;
    ACameraIdList* cameraIdList = nullptr;
    ACameraDevice* cameraDevice = nullptr;
    AImageReader* imageReader = nullptr;
    ANativeWindow* nativeWindow = nullptr;
    ACaptureSessionOutput* captureSessionOutput = nullptr;
    ACaptureSessionOutputContainer* captureSessionOutputContainer = nullptr;
    ACameraCaptureSession* session = nullptr;
    ACaptureRequest* captureRequest = nullptr;
    ACameraOutputTarget* cameraOutputTarget = nullptr;

    ACameraDevice_stateCallbacks cameraDeviceCallbacks;
    ACameraCaptureSession_captureCallbacks captureCallbacks;
    ACameraCaptureSession_stateCallbacks sessionStateCallbacks;

    cv::Mat m_androidImageFrame;
    CyC_UINT m_androidFrameId = 0;
    int32_t m_width;
    int32_t m_height;

    // set up exposure control
    int64_t exposureTime_;
    RangeValue<int64_t> exposureRange_;
    int32_t sensitivity_;
    RangeValue<int32_t> sensitivityRange_;

    CyC_INT m_nDeviceID;

public:
    NDKCamera();
    ~NDKCamera();

    /**
    * AImageReader callback handler. Called by AImageReader when a frame is
    * captured
    * (Internal function, not to be called by clients)
    */
    void imageCallback(AImageReader* reader);

    cv::Mat getCurrentFrame();
    CyC_UINT getCurrentFrameID();
};

#endif /* ANDROID_API */

#endif /* CNATIVECAMERAINTERFACE_H_ */
