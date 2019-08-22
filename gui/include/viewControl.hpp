//
// Created by sigi on 14.08.19.
//

#ifndef EPIPHANY_VIEWCONTROL_HPP
#define EPIPHANY_VIEWCONTROL_HPP

#include "Spline.hpp"
#include "mouseClickHandler.hpp"
#include "System.hpp"

using namespace std;

namespace epi {
    class ViewBuilder {

    public:
        inline static const string MAIN_WINDOW_TITLE = "hello";

        ViewBuilder(const shared_ptr<System> car);
        ~ViewBuilder();

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
