//
// Created by sigi on 14.08.19.
//

#ifndef EPIPHANY_VIEWCONTROL_HPP
#define EPIPHANY_VIEWCONTROL_HPP

#include "spline.hpp"
#include "mouse_click_handler.hpp"
#include "system.hpp"

using namespace std;

namespace epi {
    class ViewControl {

    public:
        inline static const string MAIN_WINDOW_TITLE = "hello";

        ViewControl(const shared_ptr<System> car);

        ~ViewControl();

        void drawSpline(const spline::Points &spline);

        void show();

        void resetSpline();

    private:
        shared_ptr<Vehicle> car;

        struct Impl;
        unique_ptr<Impl> pImpl;
    };
}
#endif //EPIPHANY_VIEWCONTROL_HPP
