//
// Created by sigi on 28.05.19.
//

#include <iostream>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include "UU.h"
#include "U.h"

namespace UU {

    cv::Mat imreadAlpha(const std::string& path){
        cv::Mat im = cv::imread(path, cv::IMREAD_UNCHANGED);
        cv::Mat out = cv::Mat::zeros(im.rows, im.cols, CV_8UC1);
        for(int r = 0; r < im.rows; r++){
            for(int c = 0; c < im.cols; c++){
                uchar v = im.at<cv::Vec4b>(r,c)[3];
                out.data[r*out.step+c] = v;
            }
        }
        return out;
    }

    std::tuple<int, int> calcDiff(int max, int min){
        int diff = max-min;
        int a = diff/2;
        int b = diff%2 == 0 ? a : a+1;
        return {a, b};
    }

    void pad2square(const cv::Mat& src, cv::Mat& dst){
        if(src.cols == src.rows) src.copyTo(dst);
        else if(src.cols > src.rows)
        {
            auto [top, bottom] = calcDiff(src.cols, src.rows);
            cv::copyMakeBorder(src, dst, top, bottom, 0, 0, cv::BORDER_CONSTANT);
        }
        else if(src.cols < src.rows)
        {
            auto [left, right] = calcDiff(src.rows, src.cols);
            cv::copyMakeBorder(src, dst, 0, 0, left, right, cv::BORDER_CONSTANT);
        }
    }

    void pad2square(cv::Mat& src){
        if(src.cols == src.rows) return;
        pad2square(src, src);
    }

    void rotateSimple(const cv::Mat& src, cv::Mat& dst, double angle, double scale){
        cv::Mat rot = cv::getRotationMatrix2D(cv::Point2f((src.cols-1)/2.0, (src.rows-1)/2.0), angle, scale);
        cv::warpAffine(src, dst, rot, src.size());
    }

    void rotate(const cv::Mat& src, cv::Mat& dst, double angle, double scale){
        // get rotation matrix for rotating the image around its center in pixel coordinates
        cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
        cv::Mat rot = cv::getRotationMatrix2D(center, angle, scale);
        // determine bounding rectangle, center not relevant
        cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
        // adjust transformation matrix
        rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
        rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

        cv::warpAffine(src, dst, rot, bbox.size());
    }
    void overlayImage(const cv::Mat &background, const cv::Mat &foreground, const cv::Mat &foreground_alpha,
                      cv::Mat &output, cv::Point2i location) {
        background.copyTo(output);

        for(int row = 0; row < foreground.rows; row++){
            auto row_background = row + location.y;
            for(int col = 0; col < foreground.cols; col++){
                auto col_background = col + location.x;
                auto opacity = foreground_alpha.ptr(row)[col];

                if(opacity > 127) {
                    auto xy = row_background*background.step + col_background*output.channels();
                    output.data[xy] =  foreground.at<Pixel>(row,col).x;
                    output.data[xy + 1] = foreground.at<Pixel>(row,col).y;
                    output.data[xy + 2] = foreground.at<Pixel>(row,col).z;
                }
            }
        }

        // start at the row indicated by location, or at row 0 if location.y is negative.
        /*
        for (int y = std::max(location.y, 0); y < background.rows; ++y) {
            int fY = y - location.y; // because of the translation

            // we are done of we have processed all rows of the foreground image.
            if (fY >= foreground.rows)
                break;

            // start at the column indicated by location,

            // or at column 0 if location.x is negative.
            for (int x = std::max(location.x, 0); x < background.cols; ++x) {
                int fX = x - location.x; // because of the translation.

                // we are done with this row if the column is outside of the foreground image.
                if (fX >= foreground.cols)
                    break;

                // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
                double opacity =
                        ((double) foreground.data[fY * foreground.step + fX * foreground.channels() + 3])

                        / 255.;


                // and now combine the background and foreground pixel, using the opacity,

                // but only if opacity > 0.
                for (int c = 0; opacity > 0 && c < output.channels(); ++c) {
                    unsigned char foregroundPx =
                            foreground.data[fY * foreground.step + fX * foreground.channels() + c];
                    unsigned char backgroundPx =
                            background.data[y * background.step + x * background.channels() + c];
                    output.data[y * output.step + output.channels() * x + c] =
                            backgroundPx * (1. - opacity) + foregroundPx * opacity;
                }
            }
        }

         */
    }
}
