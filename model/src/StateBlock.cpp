//
// Created by sigi on 15.06.19.
//

#include "StateBlock.hpp"
#include <functional>

template <class V>
class StateBlock{
    V _lastValue;
    const std::function<V(V, V)> _f;
public:
    StateBlock(V initialValue, const std::function<V(V, V)>& func) :
    _lastValue {initialValue}
    , _f{func}{};

    V apply(V v){
        _lastValue = _f(_lastValue, v);
        return _lastValue;
    }
};

template <class V>
struct IntergratorBlock : public StateBlock<V> {
    IntergratorBlock(V initialValue) : StateBlock<V>(initialValue, [](V a, V b){a+b;}){};
};

template <class V>
struct DifferentialBlock: public StateBlock<V> {
    DifferentialBlock(V initialValue) : StateBlock<V>(initialValue, [](V a, V b){b-a;}){};
};
