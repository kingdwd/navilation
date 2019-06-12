//
// Created by sigi on 11.06.19.
//

#include <iostream>
#include <string>
#include <opencv2/core.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "UU.h"
#include "data.h"
#include "U.h"

using namespace std;
using namespace cv;

class ViewBuilder{
    shared_ptr<epi::Vehicle> car;
    Mat car_img, map, dst, alpha, car_r, alpha_r;

    void init(){
        std::string imageName("../../gui/data/car.png");
        std::string mapName("../../gui/data/map.png");

        map = imread( mapName ); // Read the file
        car_img = imread( imageName ); // Read the file
        alpha = UU::imreadAlpha(imageName);

        if( car_img.empty() )                      // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return;
        }

        int pos = 0;
        resize(map, map, Size() , 2, 2, INTER_LINEAR);
        resize(alpha, alpha, Size() , 0.05, 0.05, INTER_AREA);
        resize(car_img, car_img, Size() , 0.05, 0.05, INTER_AREA);

        UU::pad2square(car_img);
        UU::pad2square(alpha);
    }

    void drawCarAt(epi::Pose pose){
        UU::rotateSimple(car_img, car_r, pose.phi);
        UU::rotateSimple(alpha, alpha_r, pose.phi);
        UU::overlayImage(map, car_r, alpha_r, dst, Point(pose.x,pose.y));
        imshow( "Display window", dst );                // Show our image inside it.
    }
public:
    ViewBuilder(shared_ptr<epi::Vehicle> car) : car{car} {
        init();
        car->pose.onUpdate(&ViewBuilder::onPoseUpdate, this);
    }

    void onPoseUpdate(epi::Pose pose){
        cout<<"Pose changed to " << pose << "\n";
        drawCarAt(pose);
    }

    void show(){
        namedWindow( "Display window", WINDOW_FREERATIO ); // Create a window for display.
        car->pose.set(epi::Pose{0,0,180});
        drawCarAt(car->pose.get());
    }
};
