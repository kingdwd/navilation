//
// Created by sigi on 11.06.19.
//

#include "vehicle.hpp"

epi::Vehicle::Vehicle(std::unique_ptr<epi::dynamic_model> model
                        , const epi::Pose& pose
                        , const epi::Shape& shape
                        , const std::string& type)
    : shape{shape}
    , pose{U::Var{pose}}
    , state{pose.x, pose.y, pose.phi, 0, 0, 0}
    , type{type}
    , _dynamicModel{std::move(model)}
    {}

void epi::Vehicle::drive(double longitudinal, double lateral) {
    state = _solver->next(state, longitudinal, lateral);
    pose.set(Pose{state[0], state[1], state[2]});
}

epi::State epi::Vehicle::getState() const {
    return state;
}
