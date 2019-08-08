//
// Created by sigi on 27.06.19.
//

#ifndef EPIPHANY_DEADBAND_HPP
#define EPIPHANY_DEADBAND_HPP

#include "ControlBlock.hpp"

namespace epi{
    /**
     * Realising a deadband:
     *
     *
     *              ^(y)
     *              |
     *              |      /
     *              |     /
     *              |___ /_____offset
     *              |   |
     *              |   |
     *           --------    ->(x)
     *          |   |
     *          |   |
     *         /    |
     *        /     |
     *       /      |
     *
     * with following behavior:
     * y=0 inbetween the negative and positive deadzone.
     * y=g*(x-dz)+sign(x)*offset
     *
     * where y being the output and x the input of the #apply function.
     * g and gz are the given constructor parameters representing
     * the gain and the negative or positive deadzone respectively.
     *
     */
class Deadband : public epi::ControlBlock<double> {
        double _deadzonePos;
        double _deadzoneNeg;
        double _gain;
        double _offset;
    public:
        explicit Deadband(double deadzonePos, double gain = 1);
        Deadband(double deadzoneNeg, double  deadzonePos, double gain = 1, double offset = 0);

        double apply(double) override;
    };

}
#endif //EPIPHANY_DEADBAND_HPP
