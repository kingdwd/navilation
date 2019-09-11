//
// Created by sigi on 15.08.19.
//

#ifndef EPIPHANY_DYNAMICMODEL_HPP
#define EPIPHANY_DYNAMICMODEL_HPP

#include "data.hpp"

namespace epi{

    class dynamic_model{
    public:
        virtual ~dynamic_model() = default;
        virtual State dx(const State& state, const double longitudinal, const double lateral) = 0;
    };


    struct DynamicCarModel : public dynamic_model {

        DynamicCarModel();
        ~DynamicCarModel();

        State dx(const State& state, const double longitudinal, const double lateral) const override;

    private:
        limit_block<double> _limitBlock{0.5};
        limit_block<double> _limitU_F{1.0};
        class ModelImpl;
        std::unique_ptr<ModelImpl> _model;
    };

}
#endif //EPIPHANY_DYNAMICMODEL_HPP
