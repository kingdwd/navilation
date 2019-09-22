//
// Created by sigi on 11.09.19.
//

#ifndef EPIPHANY_KINEMATICMODEL_HPP
#define EPIPHANY_KINEMATICMODEL_HPP

#include "model.hpp"

namespace epi {

    struct KinematicCarModel : public Model {

        KinematicCarModel();

        ~KinematicCarModel();

        State dx(const State &state, const double longitudinal, const double lateral) const override;

    private:
        LimitBlock<double> _limitBlock{0.5};
        LimitBlock<double> _limitU_F{1.0};

        class ModelImpl;

        std::unique_ptr<ModelImpl> _model;
    };

}
#endif //EPIPHANY_KINEMATICMODEL_HPP
