//
// Created by sigi on 07.06.19.
//

#include "data.h"
#include "UMath.h"
#include "StateBlock.cpp"
namespace epi{
    State SimpleController::convert (const double longitudinal, const double lateral, const State state) {
        using namespace U::Math;

        return State{state[0]+longitudinal*cosd(state[2])
        , state[1]+longitudinal*sind(state[2])
        , state[2]+lateral*longitudinal/1.0};
    };

    State DynamicModel::convert (const double longitudinal, const double lateral, const State state) {
        return sys.next(longitudinal, lateral);
    }

}
