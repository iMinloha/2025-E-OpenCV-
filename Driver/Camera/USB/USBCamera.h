#ifndef HLJU_RM_USBCAMERA_H
#define HLJU_RM_USBCAMERA_H

#include "Camera.h"

class USBCamera : public Camera {
public:
    explicit USBCamera(int cameraId = 0, int width = 1280, int height = 720, double exposure=0.25, double rotation=0); ;

    ~USBCamera();

    void open() override;

    void close() override;

    cv::Mat capture() override;

    bool capture(cv::Mat & mat) override;

    void release() override;

    bool isOpen() override;

private:
    cv::VideoCapture capture_;
    int cameraId;
    double rotation;
};

extern double fx, fy;

#endif //HLJU_RM_USBCAMERA_H
