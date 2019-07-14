//
// Created by sigi on 27.06.19.
//

#ifndef EPIPHANY_CONTROLBLOCK_HPP
#define EPIPHANY_CONTROLBLOCK_HPP

namespace epi{
    /**
     * Realising a deadband:
     *
     *
     *              ^(y)
     *              |
     *              |      /
     *              |     /
     *              |    /
     *           --------    ->(x)
     *         /    |
     *        /     |
     *       /      |
     *
     * with following behavior:
     * y=0 inbetween the negative and positive deadzone.
     * y=g*(x-dz)
     *
     * where y being the output and x the input of the #apply function.
     * g and gz are the given constructor parameters representing
     * the gain and the negative or positive deadzone respectively.
     *
     */
    class Deadband{
        double _deadzonePos;
        double _deadzoneNeg;
        double _gain;
    public:
        explicit Deadband(double deadzonePos, double gain = 1);
        Deadband(double deadzoneNeg, double  deadzonePos, double gain = 1);

        double apply(double);
    };


    template<class T>
    class PrevValue{
        T _initValue;
    public:
        explicit PrevValue(T initValue);

        T apply(T nextValue);
    };
}
#endif //EPIPHANY_CONTROLBLOCK_HPP
