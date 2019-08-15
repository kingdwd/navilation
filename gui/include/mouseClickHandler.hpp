//
// Created by sigi on 13.08.19.
//

#ifndef EPIPHANY_MOUSEPOSCLICKPROVIDER_HPP
#define EPIPHANY_MOUSEPOSCLICKPROVIDER_HPP

#include <data.h>
#include <opencv2/highgui/highgui.hpp>

namespace epi {

    class MouseClickHandler {
    public:

        U::Var<cv::Point> clickPos{{0,0}};

        static void onClick(int event, int x, int y, int flags, void* userdata){
            if(cv::EVENT_LBUTTONUP != event) return;

            auto *pThis = (MouseClickHandler*) userdata;
            pThis->clickPos.set({x,y});
            std::cout<<"this are points clicked "<< x << "; " << y <<"\n";
        }
    };

}
#endif //EPIPHANY_MOUSEPOSCLICKPROVIDER_HPP
