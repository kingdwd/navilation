//
// Created by sigi on 11.09.19.
//

#include "KinematicModel.hpp"
#include "UMath.h"
#include "ModelConst.hpp"

struct epi::KinematicCarModel::ModelImpl {
    State dx(const State& x, const double u_F, const double u_phi) {

        using namespace U::Math;
        using namespace con;
        double phi = x[2];
        double v = x[3];
        double sgn_v = v/sqrt(0.1 + v*v);
        double beta =atan(b*tan(u_phi)/l);
        double dv = u_F*P/m-sgn_v*(v*v*CA+Fr)/m;
        double airRes = CA*v*v;
        State dx{v * cos(x[2] + beta) // dx
                , v * sin(x[2] + beta) // dy
                , v/b*sin(beta)
                , dv
        };
        return dx;
    }
};

namespace epi {

    KinematicCarModel::KinematicCarModel() :
            _model{std::make_unique<ModelImpl>()}
    {}

    State KinematicCarModel::dx(const State& state, const double longitudinal, const double lateral) const {
        return _model->dx(state, _limitU_F.apply(longitudinal), _limitBlock.apply(lateral));
    }

    KinematicCarModel::~KinematicCarModel() = default;

}
