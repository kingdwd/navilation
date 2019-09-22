//
// Created by sigi on 11.09.19.
//

#ifndef EPIPHANY_MODEL_HPP
#define EPIPHANY_MODEL_HPP

#include "data.hpp"

namespace epi{

    class Model{
    public:
        virtual ~Model() = default;
        virtual State dx(const State& state, const double longitudinal, const double lateral) const = 0;
    };

}

#endif //EPIPHANY_MODEL_HPP
