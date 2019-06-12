//
// Created by sigi on 07.06.19.
//

#include "data.h"
#include "UMath.h"
namespace epi{
    Pose SimpleController::convert (const double longitudinal, const double lateral, const Pose state) const {
        using namespace U::Math;
        return Pose{state.x+longitudinal*cosd(state.phi)
        , state.y+longitudinal*sind(state.phi)
        , state.phi+lateral*longitudinal/1.0};
    };

}
