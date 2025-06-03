// Copyright (c) 2025 CyberCortex Robotics SRL. All rights reserved
// Author: Sorin Mihai Grigorescu

#include "CNativeCameraInterface.h"

#ifdef __ANDROID_API__

#define LOG_TAG "NDKCamera"
#define RETURN_IF_NULLPTR(var, code) if ((var) == nullptr) return code;
#define ASSERT(cond, fmt, ...)                                \
  if (!(cond)) {                                              \
    __android_log_assert(#cond, LOG_TAG, fmt, ##__VA_ARGS__); \
  }

NDKCamera::NDKCamera()
        : cameraManager(nullptr),
          captureSessionOutputContainer(nullptr),
          exposureTime_(static_cast<int64_t>(kMinExposureTime))
{
    initAndroidCameraAPI();
}

NDKCamera::~NDKCamera()
{
    disableAndroidCameraAPI();
}

void OnImageCallback(void *ctx, AImageReader *reader)
{
    reinterpret_cast<NDKCamera *>(ctx)->imageCallback(reader);
}

static void onDisconnected(void* context, ACameraDevice* device)
{
    __android_log_print(ANDROID_LOG_WARN, "NDKCamera", "Camera disconnected.");
}

static void onError(void* context, ACameraDevice* device, int error)
{
    __android_log_print(ANDROID_LOG_ERROR, "NDKCamera", "Camera error (%d).", error);
}

void onCaptureFailed(void* context, ACameraCaptureSession* session,
                     ACaptureRequest* request, ACameraCaptureFailure* failure)
{
    __android_log_print(ANDROID_LOG_WARN, "NDKCamera", "Capture failed.");
}

void onCaptureSequenceCompleted(void* context, ACameraCaptureSession* session,
                                int sequenceId, int64_t frameNumber)
{
}

void onCaptureSequenceAborted(void* context, ACameraCaptureSession* session, int sequenceId)
{
    __android_log_print(ANDROID_LOG_WARN, "NDKCamera", "Camera aborted.");
}

void onCaptureCompleted(void* context, ACameraCaptureSession* session,
                        ACaptureRequest* request, const ACameraMetadata* result)
{
}

static void onSessionActive(void* context, ACameraCaptureSession *session)
{
}

static void onSessionReady(void* context, ACameraCaptureSession *session)
{
}

static void onSessionClosed(void* context, ACameraCaptureSession *session)
{
}

static ACameraCaptureSession_stateCallbacks sessionStateCallbacks {
        .context = nullptr,
        .onActive = onSessionActive,
        .onReady = onSessionReady,
        .onClosed = onSessionClosed
};

/**
 * Helper function for YUV_420 to RGB conversion. Courtesy of Tensorflow
 * ImageClassifier Sample:
 * https://github.com/tensorflow/tensorflow/blob/master/tensorflow/examples/android/jni/yuv2rgb.cc
 * The difference is that here we have to swap UV plane when calling it.
 */
#ifndef MAX
#define MAX(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a > _b ? _a : _b;      \
  })
#define MIN(a, b)           \
  ({                        \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a < _b ? _a : _b;      \
  })
#endif

// This value is 2 ^ 18 - 1, and is used to clamp the RGB values before their
// ranges
// are normalized to eight bits.
static const int kMaxChannelValue = 262143;

static inline uint32_t YUV2RGB(int nY, int nU, int nV) {
    nY -= 16;
    nU -= 128;
    nV -= 128;
    if (nY < 0) nY = 0;

    // This is the floating point equivalent. We do the conversion in integer
    // because some Android devices do not have floating point in hardware.
    // nR = (int)(1.164 * nY + 1.596 * nV);
    // nG = (int)(1.164 * nY - 0.813 * nV - 0.391 * nU);
    // nB = (int)(1.164 * nY + 2.018 * nU);

    int nR = (int)(1192 * nY + 1634 * nV);
    int nG = (int)(1192 * nY - 833 * nV - 400 * nU);
    int nB = (int)(1192 * nY + 2066 * nU);

    nR = MIN(kMaxChannelValue, MAX(0, nR));
    nG = MIN(kMaxChannelValue, MAX(0, nG));
    nB = MIN(kMaxChannelValue, MAX(0, nB));

    nR = (nR >> 10) & 0xff;
    nG = (nG >> 10) & 0xff;
    nB = (nB >> 10) & 0xff;

    return 0xff000000 | (nR << 16) | (nG << 8) | nB;
}

void NDKCamera::imageCallback(AImageReader* reader)
{
    int32_t format;
    media_status_t status = AImageReader_getFormat(reader, &format);
    ASSERT(status == AMEDIA_OK, "Failed to get the media format");

    if (format == AIMAGE_FORMAT_JPEG)
    {
        AImage *image = nullptr;
        media_status_t status = AImageReader_acquireLatestImage(reader, &image);
        ASSERT(status == AMEDIA_OK && image, "Image is not available");


        spdlog::info("NDKCamera::imageCallback() format AIMAGE_FORMAT_JPEG");

        AImage_delete(image);
    }

    if (format == AIMAGE_FORMAT_YUV_420_888)
    {
        AImage *image = nullptr;
        media_status_t status = AImageReader_acquireNextImage(reader, &image);
        ASSERT(status == AMEDIA_OK && image, "Image is not available");

        // Get raw data into YUV format
        uint8_t *yPixel, *uPixel, *vPixel;
        int32_t yLen, uLen, vLen;
        AImage_getPlaneData(image, 0, &yPixel, &yLen);
        AImage_getPlaneData(image, 1, &uPixel, &uLen);
        AImage_getPlaneData(image, 2, &vPixel, &vLen);

        // Create contiguous data buffer for creating the opencv Mat format
        uint8_t * data = new uint8_t[yLen + vLen + uLen];
        memcpy(data, yPixel, yLen);
        memcpy(data+yLen, vPixel, vLen);
        memcpy(data+yLen+vLen, uPixel, uLen);

        // Create openCV mat and convert to RGB
        cv::Mat mYUV = cv::Mat(m_height * 1.5, m_width, CV_8UC1, data);

        // YUV_NV21 to RGB conversion works on samsung phones with native camera
        // YUV_420_888 to RGB conversion works on emulator
        //cv::cvtColor(mYUV, m_androidImageFrame, cv::COLOR_YUV2RGB_I420, 3);
        cv::cvtColor(mYUV, m_androidImageFrame, cv::COLOR_YUV2BGR_NV21, 3);

        // Count the frame ID for the filter callback
        m_androidFrameId++;

        // clean-up to prevent memory leak
        delete[] data;

        AImage_delete(image);
    }

    if (format == AIMAGE_FORMAT_RGB_888)
    {
        spdlog::info("NDKCamera::imageCallback() format is AIMAGE_FORMAT_RGB_888");

        AImage *image = nullptr;
        media_status_t status = AImageReader_acquireLatestImage(reader, &image);
        ASSERT(status == AMEDIA_OK && image, "Image is not available");

        uint8_t *data = nullptr;
        int len = 0;
        AImage_getPlaneData(image, 0, &data, &len);

        cv::Mat mRGB = cv::Mat(m_height, m_width, CV_8UC3, data);

        cv::cvtColor(mRGB, m_androidImageFrame, cv::COLOR_RGB2BGR, 3);

        // Count the frame ID for the filter callback
        m_androidFrameId++;

        // clean-up to prevent memory leak
        delete[] data;
        AImage_delete(image);

    }
}

AImageReader* NDKCamera::createYuvReader(int32_t width, int32_t height)
{
    AImageReader* reader = nullptr;
    media_status_t status = AImageReader_new(width, height, AIMAGE_FORMAT_YUV_420_888, 4, &reader);

    if (status != AMEDIA_OK)
    {
        AImageReader_delete(reader);
        return nullptr;
    }

    AImageReader_ImageListener listener{
            .context = this,
            .onImageAvailable = OnImageCallback,
    };

    AImageReader_setImageListener(reader, &listener);
    return reader;
}

AImageReader* NDKCamera::createRgbReader(int32_t width, int32_t height)
{
    AImageReader* reader = nullptr;
    media_status_t status = AImageReader_new(width, height, AIMAGE_FORMAT_RGB_888, 4, &reader);

    if (status != AMEDIA_OK)
    {
        AImageReader_delete(reader);
        return nullptr;
    }

    AImageReader_ImageListener listener{
            .context = this,
            .onImageAvailable = OnImageCallback,
    };

    AImageReader_setImageListener(reader, &listener);
    return reader;
}

AImageReader* NDKCamera::createJpegReader(int32_t width, int32_t height)
{
    AImageReader* reader = nullptr;
    media_status_t status = AImageReader_new(width, height, AIMAGE_FORMAT_JPEG, 4, &reader);

    if (status != AMEDIA_OK)
    {
        AImageReader_delete(reader);
        return nullptr;
    }

    AImageReader_ImageListener listener{
            .context = this,
            .onImageAvailable = OnImageCallback,
    };

    AImageReader_setImageListener(reader, &listener);
    return reader;
}

ANativeWindow* NDKCamera::createSurface()
{
    ANativeWindow *nativeWindow;
    AImageReader_getWindow(imageReader, &nativeWindow);

    return nativeWindow;
}

cv::Mat NDKCamera::getCurrentFrame()
{
    return m_androidImageFrame;
}

CyC_UINT NDKCamera::getCurrentFrameID()
{
    return m_androidFrameId;
}


CyC_INT NDKCamera::initAndroidCameraAPI()
{
    cameraDeviceCallbacks.context = this;
    cameraDeviceCallbacks.onDisconnected = onDisconnected;
    cameraDeviceCallbacks.onError = onError;

    captureCallbacks.context = this;
    captureCallbacks.onCaptureStarted = nullptr;
    captureCallbacks.onCaptureProgressed = nullptr;
    captureCallbacks.onCaptureCompleted = onCaptureCompleted;
    captureCallbacks.onCaptureFailed = onCaptureFailed;
    captureCallbacks.onCaptureSequenceCompleted = onCaptureSequenceCompleted;
    captureCallbacks.onCaptureSequenceAborted = onCaptureSequenceAborted;
    captureCallbacks.onCaptureBufferLost = nullptr;

    sessionStateCallbacks.context = this;
    sessionStateCallbacks.onActive = onSessionActive;
    sessionStateCallbacks.onReady = onSessionReady;
    sessionStateCallbacks.onClosed = onSessionClosed;

    cameraManager = ACameraManager_create();
    RETURN_IF_NULLPTR(cameraManager, 1);

    camera_status_t cameraStatus = ACAMERA_OK;

    cameraStatus = ACameraManager_getCameraIdList(cameraManager, &cameraIdList);
    RETURN_IF_NULLPTR(cameraIdList, -1);
    ASSERT(cameraStatus == ACAMERA_OK,
           "Failed to get camera id list (reason: %d)", cameraStatus);

    ASSERT(cameraIdList->numCameras > 0, "No camera device detected");

    //cameraDeviceCallbacks.context = pFilter;
    ACameraManager_openCamera(cameraManager, cameraIdList->cameraIds[0], &cameraDeviceCallbacks, &cameraDevice);
    RETURN_IF_NULLPTR(cameraDevice, -3);

    ACameraMetadata* metadataObj = nullptr;
    ACameraManager_getCameraCharacteristics(cameraManager, cameraIdList->cameraIds[0], &metadataObj);
    RETURN_IF_NULLPTR(metadataObj, -4);

    ACameraMetadata_const_entry lens_facing_entry;
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_LENS_FACING_BACK, &lens_facing_entry);

    ACameraMetadata_const_entry entry;
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);

    //int32_t width = 0;
    //int32_t height = 0;
    for (int i = 0; i < entry.count; i += 4)
    {
        // We are only interested in output streams, so skip input stream
        int32_t input = entry.data.i32[i + 3];
        int32_t format = entry.data.i32[i + 0];
        if (input)
            continue;

        if (format == AIMAGE_FORMAT_YUV_420_888 || format == AIMAGE_FORMAT_JPEG)
        {
            m_width = entry.data.i32[i + 1];
            m_height = entry.data.i32[i + 2];
        }
    }

    m_width = 640;
    m_height = 480;

    //imageReader = createJpegReader(m_width, m_height);
    imageReader = createYuvReader(m_width, m_height);
    //imageReader = createRgbReader(m_width, m_height);

    RETURN_IF_NULLPTR(imageReader, -5);
    nativeWindow = createSurface();
    RETURN_IF_NULLPTR(nativeWindow, -6);

    ACaptureSessionOutput_create(nativeWindow, &captureSessionOutput);
    RETURN_IF_NULLPTR(captureSessionOutput, -6);

    ACaptureSessionOutputContainer_create(&captureSessionOutputContainer);
    RETURN_IF_NULLPTR(captureSessionOutputContainer, -7);

    ACaptureSessionOutputContainer_add(captureSessionOutputContainer, captureSessionOutput);

    ACameraDevice_createCaptureRequest(cameraDevice, TEMPLATE_PREVIEW, &captureRequest);
    RETURN_IF_NULLPTR(captureRequest, -9);

    ACameraDevice_createCaptureSession(cameraDevice, captureSessionOutputContainer, &sessionStateCallbacks, &session);
    RETURN_IF_NULLPTR(session, -8);

    ACameraOutputTarget_create(nativeWindow, &cameraOutputTarget);
    RETURN_IF_NULLPTR(cameraOutputTarget, -10);

    ACaptureRequest_addTarget(captureRequest, cameraOutputTarget);

    ACameraMetadata_const_entry val = {0,};
    camera_status_t status = ACameraMetadata_getConstEntry(
            metadataObj, ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE, &val);

    if (status == ACAMERA_OK)
    {
        exposureRange_.min_ = val.data.i64[0];
        if (exposureRange_.min_ < kMinExposureTime) {
            exposureRange_.min_ = kMinExposureTime;
        }
        exposureRange_.max_ = val.data.i64[1];
        if (exposureRange_.max_ > kMaxExposureTime) {
            exposureRange_.max_ = kMaxExposureTime;
        }
        exposureTime_ = exposureRange_.value(2);
    }
    status = ACameraMetadata_getConstEntry(
            metadataObj, ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE, &val);

    if (status == ACAMERA_OK){
        sensitivityRange_.min_ = val.data.i32[0];
        sensitivityRange_.max_ = val.data.i32[1];

        sensitivity_ = sensitivityRange_.value(2);
    }

    // Set auto exposure on
    uint8_t aeModeOn = ACAMERA_CONTROL_AE_MODE_ON;
    ACaptureRequest_setEntry_u8(captureRequest,ACAMERA_CONTROL_AE_MODE, 1, &aeModeOn);

    ACameraCaptureSession_setRepeatingRequest(session, &captureCallbacks, 1, &captureRequest, nullptr);

    // check auto exposure
    /*ACameraMetadata_const_entry ae_mode_entry;
    ACameraMetadata_getConstEntry(metadataObj,
                                  ACAMERA_CONTROL_AE_MODE, &ae_mode_entry);

    uint8_t ae_mode = ae_mode_entry.data.u8[0];

    __android_log_print(ANDROID_LOG_INFO, "NDKCamera", "AE mode: %d ", static_cast<acamera_metadata_enum_acamera_control_ae_mode>(ae_mode));
    __android_log_print(ANDROID_LOG_INFO, "NDKCamera", "AE mode static : %d ", ACAMERA_CONTROL_AE_MODE_ON);*/

    /*ACameraMetadata_const_entry sensor_orientation;
    ACameraMetadata_getConstEntry(metadataObj, ACAMERA_SENSOR_ORIENTATION, &sensor_orientation);

    __android_log_print(ANDROID_LOG_INFO, "NDKCamera", "====Current SENSOR_ORIENTATION: %8d", sensor_orientation.data.i32[0]);*/

   ACameraMetadata_free(metadataObj);

   spdlog::info("NDKCamera::initAndroidCameraAPI() Set repeating request successful");
   return 0;
}

void NDKCamera::disableAndroidCameraAPI()
{
   ACameraCaptureSession_stopRepeating(session);
   ACameraCaptureSession_close(session);

   ACaptureRequest_removeTarget(captureRequest, cameraOutputTarget);
   ACaptureRequest_free(captureRequest);

   ACameraOutputTarget_free(cameraOutputTarget);

   ACaptureSessionOutputContainer_remove(captureSessionOutputContainer, captureSessionOutput);
   ACaptureSessionOutputContainer_free(captureSessionOutputContainer);

   ACaptureSessionOutput_free(captureSessionOutput);

   AImageReader_delete(imageReader);

   ACameraDevice_close(cameraDevice);
   ACameraManager_deleteCameraIdList(cameraIdList);
   ACameraManager_delete(cameraManager);

   __android_log_print(ANDROID_LOG_INFO, "NDKCamera", "Camera disabled.");
}

#endif /* __ANDROID_API__ */