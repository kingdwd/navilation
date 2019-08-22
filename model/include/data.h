//
// Created by sigi on 06.06.19.
//

#ifndef EPIPHANY_POSE_H
#define EPIPHANY_POSE_H

#include "U.h"
#include "ControlBlock.hpp"
#include "Deadband.hpp"
#include "LimitBlock.hpp"
#include <memory>
#include <ostream>
#include <opencv4/opencv2/core.hpp>
#include <UMath.h>

namespace epi {

    namespace con{
    }

    struct Point {
        double x, y;

        bool operator==(const Point &rhs) const {
            return x == rhs.x &&
                   y == rhs.y;
        }

        bool operator!=(const Point &rhs) const {
            return !(rhs == *this);
        }

        friend std::ostream &operator<<(std::ostream &os, const Point &point) {
            os << "x: " << point.x << " y: " << point.y;
            return os;
        }
    };


    struct Pose : public Point {
        double phi;

        friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
            os << static_cast<const Point &>(pose) << " phi: " << pose.phi;
            return os;
        }

        bool operator==(const Pose &rhs) const {
            return static_cast<const epi::Point &>(*this) == static_cast<const epi::Point &>(rhs) &&
                   phi == rhs.phi;
        }

        bool operator!=(const Pose &rhs) const {
            return !(rhs == *this);
        }
    };

    struct Shape {
        double length, width;
    };

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
    struct IntegratorBlock : public StateBlock<V> {
        IntegratorBlock(V initialValue) : StateBlock<V>(initialValue, [](V a, V b){return a+b;}){};
    };

    template <class V>
    struct DifferentialBlock: public StateBlock<V> {
        DifferentialBlock(V initialValue) : StateBlock<V>(initialValue, [](V a, V b){return b-a;}){};
    };
    using State = cv::Vec<double, 5>;


}

#endif //EPIPHANY_POSE_H
