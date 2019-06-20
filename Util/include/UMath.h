//
// Created by sigi on 12.06.19.
//

#ifndef EPIPHANY_UMATH_H
#define EPIPHANY_UMATH_H


namespace U {
    namespace Math{

        /**
         * converts degrees to radiant
         * @param degrees unit as degrees
         * @return converted value
         */
        double d2r(double degrees);

        double r2d(double radiant);

        /**
         * sinus as a function of degrees
         * @param degrees
         * @return sinus value of the given parameter
         */
        double sind(double degrees);

        /**
         * cosinus as a function of degrees
         * @param degrees
         * @return cosinus value of the given parameter
         */
        double cosd(double degrees);

        double tand(double degrees);

        double atand(double relation);

        template <typename T> int sgn(T val) {
            return (T(0) < val) - (val < T(0));
        }
    }
}
#endif //EPIPHANY_UMATH_H
