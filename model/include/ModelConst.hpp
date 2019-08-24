//
// Created by sigi on 19.08.19.
//

#ifndef EPIPHANY_MODELCONST_HPP
#define EPIPHANY_MODELCONST_HPP
#include <chrono>

namespace epi{
    namespace con{

        using namespace std::literals;
        /**
         * step size of simulation in s
         */
        constexpr std::chrono::duration<long double> STEP_SIZE = 0.01s;
        static constexpr double g = 9.81;
        static constexpr double m = 1700;    /* Vehicle mass.                    */
        static constexpr double a = 1.5;     /* Distance from front axle to COG. */
        static constexpr double b = 1.5;     /* Distance from rear axle to COG.  */
        static constexpr double Cx = 1.5E5;  /* Longitudinal tire stiffness.     */
        static constexpr double Cy = 4E5;    /* Lateral tire stiffness.          */
        static constexpr double CA = 1.5;    /* Air resistance coefficient.      */
        static constexpr double P = 3E5;     /* Power */
        static constexpr double Cr = 1.0;    /* Rolling resistance coefficient   */
        static constexpr double Fr = m * g * Cr; /* normal force of friction         */
    }
}
#endif //EPIPHANY_MODELCONST_HPP
