//
// Created by sigi on 11.06.19.
//

#include "Vehicule.hpp"

epi::Vehicle::Vehicle(std::shared_ptr<epi::Model> model
                        , const epi::Pose& pose
                        , const epi::Shape& shape
                        , const std::string& type)
    : shape{shape}
    , pose{U::Var{pose}}
    , state{pose.x, pose.y, pose.phi, 0, 0, 0}
    , type{type}
    , _dynamicModel{std::move(model)}
    {}

void epi::Vehicle::drive(double uF, double uPhi) {
    std::cout<<"got move command : [" <<uF <<";" <<uPhi << "] \n";
    state = _solver->next(state, uF, uPhi);
    std::cout<<"state: "<< state << std::endl;
    pose.set(Pose{state[0], state[1], state[2]});
}

epi::State epi::Vehicle::getState() const {
    return state;
}
