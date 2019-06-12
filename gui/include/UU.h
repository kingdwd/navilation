//
// Created by sigi on 28.05.19.
//

#ifndef EPIPHANY_UU_H
#define EPIPHANY_UU_H


#include <opencv2/core/mat.hpp>

namespace UU {
    using Pixel = cv::Point3_<uint8_t>;
    void overlayImage(const cv::Mat &background, const cv::Mat &foreground, const cv::Mat &foreground_alpha,
                      cv::Mat &output, cv::Point2i location);

    void rotateSimple(const cv::Mat& src, cv::Mat& dst, double angle, double scale = 1);

    void pad2square(const cv::Mat& src, cv::Mat& dst);

    void pad2square(cv::Mat& src);

    void rotate(const cv::Mat& src, cv::Mat& dst, double degrees, double scale = 1);

    cv::Mat imreadAlpha(const std::string& path);
}

#endif //EPIPHANY_UU_H
