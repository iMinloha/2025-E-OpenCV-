#include "Detector.h"
#include <vector>
#include "algorithm"
#include "cmath"

using namespace std;

Point2f Detector::purple_point(const Mat &input) {
    Mat img = input.clone();
    const Scalar purple_lower(130, 60, 60);     // HSV紫色下限（根据激光颜色可调整）
    const Scalar purple_upper(170, 255, 255);   // HSV紫色上限
    const int min_area = 10;                                // 最小轮廓面积
    const Size kernel_size(5, 5);             // 形态学核大小

    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);

    // 1. 紫色区域分割（激光通常为紫边白心）
    Mat mask;
    inRange(hsv, purple_lower, purple_upper, mask);

    // 2. 闭运算连接紫色边缘与白心（关键）
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, kernel_size);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);  // 连接激光边缘与中心
    // 膨胀
    morphologyEx(mask, mask, MORPH_DILATE, kernel);  // 扩大连接区域

    // 3. 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Point2f best_center(-1, -1);
    float best_score = 0;

    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area < min_area) continue;

        // 4. 圆形度检测（激光点应接近圆）
        double perimeter = arcLength(contour, true);
        if (perimeter == 0) continue;
        double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < 0.3) continue;  // 可视情况调节（建议范围 0.3~1.0）

        // 5. 拟合椭圆
        if (contour.size() < 5) continue;
        RotatedRect ellipse = fitEllipse(contour);
        Point center = ellipse.center;

        // 6. 检查中心是否在图像内
        if (center.x < 2 || center.y < 2 ||
            center.x + 2 >= input.cols || center.y + 2 >= input.rows) {
            continue;
        }

        // 7. 检查中心亮度（V通道是否过曝）
        Vec3b hsv_center = hsv.at<Vec3b>(center);
        int v = hsv_center[2];
        if (v < 240) continue;  // 激光中心通常非常亮

        // 8. 选择最优圆形度作为激光点
        if (circularity > best_score) {
            best_score = (float) circularity;
            best_center = ellipse.center;
        }
    }

    return best_center;
}

pair<bool, vector<Point>> is_approx_rect(const vector<Point>& contour, double epsilon_factor = 0.02) {
    double peri = arcLength(contour, true);
    vector<Point> approx;
    approxPolyDP(contour, approx, epsilon_factor * peri, true);
    bool is_convex = isContourConvex(approx);
    bool is_rect = (approx.size() >= 4 && approx.size() <= 5) && is_convex;
    return make_pair(is_rect, approx);
}

// 计算轮廓中心
Point calc_center(const vector<Point>& approx) {
    Moments M = moments(approx);
    if (M.m00 == 0) return Point(-1, -1);
    return Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
}

// 计算两点距离
double distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}


Point prev_center(-1, -1);

Point2f Detector::run(const Mat &input) {
    Mat frame, gray, binary, closed, contour_img;
    frame = input.clone();

    cvtColor(frame, gray, COLOR_BGR2GRAY);
    threshold(gray, binary, 120, 255, THRESH_BINARY_INV);
    erode(binary, binary, getStructuringElement(MORPH_RECT, Size(5, 5)));
    morphologyEx(binary, closed, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(70, 70)));


    // imshow("closed", closed);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(closed, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 候选矩形处理
    vector<tuple<vector<Point>, Point, double>> candidates;
    for (const auto& cnt : contours) {
        auto result = is_approx_rect(cnt);
        if (result.first) {
            Point center = calc_center(result.second);
            if (center.x >= 0) {
                double area = contourArea(result.second);
                candidates.emplace_back(result.second, center, area);
            }
        }
    }

    // 选择最佳矩形
    tuple<vector<Point>, Point, double> selected;
    if (candidates.empty()) {
        selected = make_tuple(vector<Point>(), Point(), 0);
    } else if (prev_center == Point(-1, -1)) {
        auto max_it = max_element(candidates.begin(), candidates.end(),
                                  [](const auto& a, const auto& b) { return get<2>(a) < get<2>(b); });
        selected = *max_it;
    } else {
        sort(candidates.begin(), candidates.end(),
             [&](const auto& a, const auto& b) {
                 return distance(get<1>(a), prev_center) < distance(get<1>(b), prev_center);
             });

        vector<tuple<vector<Point>, Point, double>> top_n;
        for (const auto& c : candidates) {
            if (distance(get<1>(c), prev_center) - distance(get<1>(candidates[0]), prev_center) < 50) {
                top_n.push_back(c);
            } else {
                break;
            }
        }
        auto max_it = max_element(top_n.begin(), top_n.end(),
                                  [](const auto& a, const auto& b) { return get<2>(a) < get<2>(b); });
        selected = *max_it;
    }

    // 绘制结果
    Mat display_frame = frame.clone();
    contour_img = Mat::zeros(frame.size(), frame.type());

    if (!get<0>(selected).empty()) {
        auto& approx = get<0>(selected);
        auto& center = get<1>(selected);
        drawContours(display_frame, vector<vector<Point>>{approx}, -1, Scalar(0, 0, 255), 5);
        circle(display_frame, center, 7, Scalar(0, 0, 255), -1);
        drawContours(contour_img, vector<vector<Point>>{approx}, -1, Scalar(0, 255, 0), 3);
        prev_center = center;
    } else {
        prev_center = Point(-1, -1);
    }

    return prev_center;
}


void Detector::_imgshow(const string &name, const Mat &img) {
    // img缩小一半显示
    Mat half;
    resize(img, half, Size(), 0.5, 0.5);
    imshow(name, half);
    waitKey(1);
}

