//
// Created by sigi on 07.06.19.
//


#include "dynamic_model.hpp"
#include "math_util.h"
#include "model_const.hpp"

struct epi::DynamicCarModel::ModelImpl {
    Deadband _deadband{800};

    State dx(const State& x, const double u_F, const double u_phi) {

        using namespace U::Math;
        using namespace con;
        //double beta =atan(b*tan(phi)/(a+b));
        double v_x = x[3];
        double v_y = x[4];
        double sign_v = sgn(v_x);
        double v = sign_v * sqrt(v_x * v_x + v_y * v_y);
        double d_phi = v / (a + b) * u_phi;
        double dd_phi = _deadband.apply(-v_x * d_phi);
        //double threshold_vx = 2;
        //double Fw_x = abs(v_x) > threshold_vx ? -2 * Cy * (phi - (v_y + a * d_phi) / abs(v_x) * sin_Uphi) : 0;
        //double Fw_y = abs(v_x) > threshold_vx ?
        //              +2 * Cy * (phi - (v_y + a * d_phi) / abs(v_x)) * cos_Uphi
        //              + 2 * Cy * (b * d_phi - v_y) / abs(v_x) : 0;
        //double Fw_phi = abs(v_x) > threshold_vx ? a * (2 * Cy * (phi - (v_y + a * d_phi) / abs(v_x)))
        //                                          - 2 * b * Cy * (b * d_phi - v_y) / abs(v_x)
        //                                        : 0;
        double sign_y = sgn(v_y);
        double airRes = CA*v*v;
        State dx{v * cos(x[2]) // dx
                , v * sin(x[2]) // dy
                , d_phi
                , v_y * d_phi + (P*u_F - sign_v*(CA*v_x*v_x+Fr))/m,
                 dd_phi - sign_y * (CA*v_y*v_y + Fr*5)/m
        };
        return dx;
    }
};

namespace epi {

    DynamicCarModel::DynamicCarModel() :
            _model{std::make_unique<ModelImpl>()}
    {}

    State DynamicCarModel::dx(const State& state, const double longitudinal, const double lateral) const {
        return _model->dx(state, _limitU_F.apply(longitudinal), _limitBlock.apply(lateral));
    }

    DynamicCarModel::~DynamicCarModel() = default;

}
