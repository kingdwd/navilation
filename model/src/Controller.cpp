//
// Created by sigi on 07.06.19.
//

#include "data.h"
#include <math.h>
namespace epi{
    Pose SimpleController::convert (const double longitudinal, const double lateral, const Pose state) const {
        double rad = d2r(state.phi);
        return Pose{state.x+longitudinal*cosd(rad)
        , state.y+longitudinal*sind(state.phi)
        , state.phi+lateral*longitudinal};
    };

}
