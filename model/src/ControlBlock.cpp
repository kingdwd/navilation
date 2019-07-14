//
// Created by sigi on 27.06.19.
//

#include "ControlBlock.hpp"

epi::Deadband::Deadband(double deadzone, double gain) : Deadband(-deadzone, deadzone, gain) {}

double epi::Deadband::apply(double x) {
    if(_deadzoneNeg > x) return _gain*(x-_deadzoneNeg);
    if(_deadzonePos < x) return _gain*(x-_deadzonePos);
    return 0;
}

epi::Deadband::Deadband(double deadzoneNeg, double deadzonePos, double gain) :
                _deadzoneNeg{deadzoneNeg},
                _deadzonePos{deadzonePos},
                _gain{gain}
                {}

template <class T>
epi::PrevValue<T>::PrevValue(T initValue) : _initValue{initValue} {}

template <class T>
T epi::PrevValue<T>::apply(T nextValue) {
    auto temp = _initValue;
    _initValue = nextValue;
    return temp;
}
