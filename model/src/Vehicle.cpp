//
// Created by sigi on 11.06.19.
//

#include "data.h"
#include <memory>


void epi::Vehicle::drive(double longitudinal, double lateral) {
    state = _controller->convert(longitudinal, lateral, state);
    pose.set(Pose{state[0], state[1], state[2]});
}
