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

        /* get car into intial position, to match image and initial pose */
        UU::rotateSimple(car_img, car_img, 180);
        UU::rotateSimple(alpha, alpha, 180);
    }

    void drawCarAt(epi::Pose pose){
        UU::rotateSimple(car_img, car_r, pose.phi);
        UU::rotateSimple(alpha, alpha_r, pose.phi);
        /* opencv uses a slightly different coordinate system:
         * y-axis points into the opposite direction. Hence the
         * sign reversal.
         * */

        UU::overlayImage(map, car_r, alpha_r, dst, Point(pose.x,-pose.y));
        imshow( "Display window", dst );                // Show our image inside it.
    }

    void onPoseUpdate(epi::Pose pose){
        cout<<"Pose changed to " << pose << "\n";
        drawCarAt(pose);
    }
public:
    ViewBuilder(shared_ptr<epi::Vehicle> car) : car{car} {
        init();
        car->pose.onUpdate(&ViewBuilder::onPoseUpdate, this);
    }

    void show(){
        namedWindow( "Display window", WINDOW_FREERATIO ); // Create a window for display.
        drawCarAt(car->pose.get());
    }
};
