//
// Created by sigi on 11.06.19.
//

#include <opencv4/opencv2/imgproc.hpp>
#include "viewControl.hpp"
#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>

#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <view.hpp>
#include "UU.h"

using namespace std;
using namespace epi;
using namespace cv;

struct epi::ViewBuilder::Impl {
    Mutex _mutex;
    Mat car_img, workMap, map, dst, alpha, car_r, alpha_r;
    shared_ptr<MouseClickHandler> _mouseClickHandler;
    shared_ptr<ViewModel> _viewModel;

    View view{_viewModel};

    Impl(shared_ptr<MouseClickHandler> mouseHandler
            , const std::shared_ptr<epi::System> sys
            , ViewBuilder* vb)
    : _mouseClickHandler{mouseHandler}
    , _viewModel{make_shared<ViewModel>(_mouseClickHandler, sys, vb)}
    {}

    void init() {
        std::string imageName("../../gui/data/car.png");
        std::string mapName("../../gui/data/map.png");

        map = imread(mapName); // Read the file
        car_img = imread(imageName); // Read the file
        alpha = UU::imreadAlpha(imageName);

        if (car_img.empty())                      // Check for invalid input
        {
            cout << "Could not open or find the image\n";
            return;
        }

        int pos = 0;
        constexpr int INTER_LINEAR = 1;
        resize(map, map, Size(), 2, 2, INTER_LINEAR);
        resize(alpha, alpha, Size(), 0.05, 0.05, INTER_AREA);
        resize(car_img, car_img, Size(), 0.05, 0.05, INTER_AREA);
        workMap = map;

        UU::pad2square(car_img);
        UU::pad2square(alpha);

        /* get car into intial position, to match image and initial pose */
        UU::rotateSimple(car_img, car_img, 180);
        UU::rotateSimple(alpha, alpha, 180);
    }

    void drawCarAt(const Pose &pose) {
        double phi = U::Math::r2d(pose.phi);
        UU::rotateSimple(car_img, car_r, phi);
        UU::rotateSimple(alpha, alpha_r, phi);
        /* opencv uses a slightly different coordinate system:
         * y-axis points into the opposite direction. Hence the
         * sign reversal.
         * */

        UU::overlayImage(workMap, car_r, alpha_r, dst, cv::Point(pose.x, -pose.y));
        imshow(MAIN_WINDOW_TITLE, dst);
    }

    void onPoseUpdate(const Pose &pose) {
        std::unique_lock<mutex>(_mutex);
        cout << "Pose changed to " << pose << "\n";
        drawCarAt(pose);
    }
};

epi::ViewBuilder::ViewBuilder(const shared_ptr<System> sys)
: car{sys->vehicle}
, pImpl{make_unique<Impl>(make_shared<MouseClickHandler>(), sys, this)}
{
    pImpl->init();
    car->pose.onUpdate(pImpl.get(), &Impl::onPoseUpdate);
}

const static Scalar RED(0,0,255);
void epi::ViewBuilder::drawSpline(const spline::Points& spline){
    pImpl->workMap = pImpl->dst;
    int pointThickNess = 2;
    auto pointToDraw = Mat(pointThickNess, pointThickNess, pImpl->car_r.type(), RED);
    auto opacity = 255*Mat::ones(pointThickNess, pointThickNess, pImpl->alpha_r.type());
    for(auto& point : spline){
        std::cout<< "point to vis as path: " << point << "\n";
        UU::overlayImage(pImpl->workMap, pointToDraw, opacity
                , pImpl->dst, cv::Point(point.x, point.y));
    }
    imshow(MAIN_WINDOW_TITLE, pImpl->dst);
}

void epi::ViewBuilder::show(){
    namedWindow( MAIN_WINDOW_TITLE, WINDOW_FREERATIO ); // Create a window for display.
    setMouseCallback(MAIN_WINDOW_TITLE, &MouseClickHandler::onClick , pImpl->_mouseClickHandler.get());
    pImpl->drawCarAt(car->pose.get());
}

void ViewBuilder::resetSpline() {
    pImpl->workMap = pImpl->map;
    pImpl->drawCarAt(car->pose.get());
}

epi::ViewBuilder::~ViewBuilder() = default;
