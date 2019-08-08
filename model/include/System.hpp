//
// Created by sigi on 19.06.19.
//

#ifndef EPIPHANY_SYSTEM_HPP
#define EPIPHANY_SYSTEM_HPP

namespace epi {
    template <typename State, typename Input>
    class System{
        const State _initialState;
    public:
        System(State initialState) : _initialState{initialState} {};
        virtual ~System() = default;

        virtual State apply(State state, Input u) = 0;
    };
}
#endif //EPIPHANY_SYSTEM_HPP
