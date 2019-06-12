//
// Created by sigi on 11.06.19.
//

#include "data.h"
#include <memory>


void epi::Vehicle::drive(double longitudinal, double lateral) {
    pose.set(_controller->convert(longitudinal, lateral, pose.get()));
}
