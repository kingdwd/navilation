//
// Created by sigi on 12.06.19.
//

#include "math_util.h"
#include <math.h>

double U::Math::d2r(double d) {
    return (d / 180.0) * ((double) M_PI);
}

double U::Math::r2d(double radiant) {
    return radiant/((double) M_PI)*180;
}

double U::Math::sind(double x) {
    if (!isfinite(x)) {
        return sin(x);
    }
    if (x < 0.0) {
        return -sind(-x);
    }
    int quo;
    double x90 = remquo(fabs(x), 90.0, &quo);
    switch (quo % 4) {
        case 0:
            // Use * 1.0 to avoid -0.0
            return sin(d2r(x90)* 1.0);
        case 1:
            return cos(d2r(x90));
        case 2:
            return sin(d2r(-x90) * 1.0);
        case 3:
            return -cos(d2r(x90));
    }
    return 0.0;
}

double U::Math::cosd(double x) {
    if (!isfinite(x)) {
        return cos(x);
    }
    if (x < 0.0) {
        return cosd(-x);
    }
    int quo;
    double x90 = remquo(fabs(x), 90.0, &quo);
    switch (quo % 4) {
        case 0:
            // Use * 1.0 to avoid -0.0
            return cos(d2r(x90));
        case 1:
            return -sin(d2r(x90)* 1.0);
        case 2:
            return -cos(d2r(x90));
        case 3:
            return sin(d2r(x90) * 1.0);
    }
    return 0.0;
}

double U::Math::tand(double degrees) {
    return tan(d2r(degrees));
}

double U::Math::atand(double relation) {
    return r2d(atan(relation));
}