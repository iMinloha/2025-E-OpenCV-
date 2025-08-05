#include "USBCamera.h"

USBCamera::~USBCamera() {
    capture_.release();
}

USBCamera::USBCamera(int cameraId, int width, int height, double exposure, double rotation) {

#ifdef _WIN32
    capture_ = cv::VideoCapture(0);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, exposure);
#endif

#ifdef __linux__
    capture_ = cv::VideoCapture(0, cv::CAP_V4L2);
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    // 设置相机曝光
    capture_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    capture_.set(cv::CAP_PROP_EXPOSURE, 5000);
#endif
    capture_.set(cv::CAP_PROP_FPS, 30);
    this->cameraId = cameraId;
    this->rotation = rotation;
}

void USBCamera::open() {
    if (!capture_.isOpened())
        capture_.open(this->cameraId);
}

void USBCamera::close() {
    capture_.release();
}

cv::Mat USBCamera::capture() {
    cv::Mat frame;
    capture_ >> frame;
    // 图像旋转rotation
    if (rotation != 0) {
        cv::Point2f center(frame.cols / 2, frame.rows / 2);
        cv::Mat M = cv::getRotationMatrix2D(center, rotation, 1);
        cv::warpAffine(frame, frame, M, cv::Size(frame.cols, frame.rows));
    }
    return frame;
}

void USBCamera::release() {
    capture_.release();
}

bool USBCamera::isOpen() {
    return capture_.isOpened();
}

bool USBCamera::capture(cv::Mat & mat) {
    capture_ >> mat;
    return true;
}
