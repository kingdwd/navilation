//
// Created by sigi on 03.08.19.
//

#include <opencv4/opencv2/core/mat.hpp>
#include "spline.hpp"

using Punkt = cv::Point;

inline double tj(double ti, const Punkt& p_i, const Punkt& p_j){
    return ti + cv::norm(p_j - p_i);
}

inline Punkt iter(double t, double ti, double tj, const Punkt& left, const Punkt& right){
    return ((tj - t)*left + (t - ti)*right)/(tj - ti);
}

inline Punkt catMullRomSpline(double t, const Punkt& p0, const Punkt& p1, const Punkt& p2, const Punkt& p3
                                                , const double t1, const double t2, const double t3){
    double t0 = 0;

    Punkt a1 = iter(t, t0, t1, p0, p1);
    Punkt a2 = iter(t, t1, t2, p1, p2);
    Punkt a3 = iter(t, t2, t3, p2, p3);

    Punkt b1 = iter(t, t0, t2, a1, a2);
    Punkt b2 = iter(t, t1, t3, a2, a3);

    return iter(t, t1, t2, b1, b2);
}

epi::spline::Points epi::spline::interpol(const Points &points
                                            , const int nPoints
                                            , const Solver& solver) {
    Points spline;

    if(solver == Solver::CATMULL_ROM) {
        for(int i=0; i<points.size()-3; i++){
            auto& p0 = points[i];
            auto& p1 = points[i+1];
            auto& p2 = points[i+2];
            auto& p3 = points[i+3];
            double t0 = 0;
            double t1 = tj(t0, p0, p1);
            double t2 = tj(t1, p1, p2);
            double t3 = tj(t2, p2, p3);

            double tStep = (t2-t1)/(nPoints-1);
            for(int i=0; i<nPoints; i++){
                spline.push_back(catMullRomSpline(t1 + i*tStep, p0, p1, p2, p3, t1, t2, t3));
            }
        }
    }
    return spline;
}
