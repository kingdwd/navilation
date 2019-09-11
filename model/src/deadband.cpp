//
// Created by sigi on 27.06.19.
//

#include "deadband.hpp"

epi::Deadband::Deadband(double deadzone, double gain) : Deadband(-deadzone, deadzone, gain) {}

epi::Deadband::Deadband(double deadzoneNeg, double deadzonePos, double gain, double offset) :
        _deadzoneNeg{deadzoneNeg},
        _deadzonePos{deadzonePos},
        _gain{gain},
        _offset{offset}
{}

double epi::Deadband::apply(double x) const {
    if(_deadzoneNeg > x) return _gain*(x-_deadzoneNeg) + _offset;
    if(_deadzonePos < x) return _gain*(x-_deadzonePos) - _offset;
    return 0;
}


