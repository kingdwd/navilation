//
// Created by sigi on 26.07.19.
//

#ifndef EPIPHANY_CONTROLBLOCK_HPP
#define EPIPHANY_CONTROLBLOCK_HPP

namespace epi{
    template <typename T>
    class ControlBlock{
    public:
        virtual T apply(const T) = 0;
        T operator>>(const T& a){
            return apply(a);
        }
    };
}

#endif //EPIPHANY_CONTROLBLOCK_HPP
