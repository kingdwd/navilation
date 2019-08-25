//
// Created by sigi on 27.07.19.
//

#ifndef EPIPHANY_LIMITBLOCK_HPP
#define EPIPHANY_LIMITBLOCK_HPP

#include "ControlBlock.hpp"
#include <math.h>

namespace epi {
    /**
     * Realising a value limition:
     *
     *
     *                  ^(y)
     *                  |
     *                  |
     *                  |   ________limit
     *                  |  /
     *                  | /
     *                  |/
     *        ----------/-----------0
     *                 /|
     *                / |
     * -limit _______/  |
     *                  |
     *                  |
     *
     * with following behavior:
     * @tparam T
     */
    template<typename T>
    class LimitBlock : public epi::ControlBlock<T> {
        T _limit;
    public:
        LimitBlock(T limit) : _limit{limit} {}

        T apply(const T value) override {
            if(std::isnan(value)) return 0;
            if (value > _limit ) return _limit;
            if(-value > _limit) return -_limit;
            return value;
        }

    };
}
#endif //EPIPHANY_LIMITBLOCK_HPP
