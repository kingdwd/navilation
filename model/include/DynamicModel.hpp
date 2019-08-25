//
// Created by sigi on 15.08.19.
//

#ifndef EPIPHANY_DYNAMICMODEL_HPP
#define EPIPHANY_DYNAMICMODEL_HPP

#include "data.h"

namespace epi{

    class DynamicModel{
    public:
        virtual ~DynamicModel() = default;
        virtual State dx(const State& state, const double longitudinal, const double lateral) = 0;
    };


    struct DynamicCarModel : public DynamicModel {

        DynamicCarModel();
        ~DynamicCarModel();

        State dx(const State& state, const double longitudinal, const double lateral) override;

    private:
        LimitBlock<double> _limitBlock{0.5};
        LimitBlock<double> _limitU_F{1.0};
        class ModelImpl;
        std::unique_ptr<ModelImpl> _model;
    };

}
#endif //EPIPHANY_DYNAMICMODEL_HPP
