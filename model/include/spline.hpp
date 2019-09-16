//
//
//

#ifndef EPIPHANY_CATMULLROMSPLINE_HPP
#define EPIPHANY_CATMULLROMSPLINE_HPP

#include <opencv4/opencv2/core/matx.hpp>
#include <opencv4/opencv2/core.hpp>
#include <vector>

namespace epi {
    namespace spline {
        using Points = std::vector<cv::Point>;

        enum class Solver{
            CATMULL_ROM
        };

        /**
         * Calculates a set of interpolated points inbetween of the given predefined points (knots).
         * The total length of the returned vector is defined nPoints*(len(points)-2) in case of Catmull-Rom.
         *
         * @param points that shall be used for interpolation
         * @param nPoints number of interpolation points between each two given point pairs
         * @param solver to be used for interpolation.
         * @return
         */
        Points interpol(Points const & points
                , const int nPoints = 100
                , const Solver& solver = Solver::CATMULL_ROM);

    }
}
#endif //EPIPHANY_CATMULLROMSPLINE_HPP
