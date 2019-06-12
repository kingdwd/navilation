//
// Created by sigi on 06.06.19.
//

#ifndef EPIPHANY_POSE_H
#define EPIPHANY_POSE_H

#include "U.h"
#include <memory>
#include <ostream>

namespace epi {

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

    class Controller{
    public:
        //Controller() = default;
        //virtual ~Controller();
        virtual Pose convert(const double longitudinal, const double lateral, const Pose state) const = 0;
        //virtual Pose convert(double longitudinal, double lateral, State state) = 0;
    };

    class SimpleController : public Controller {
    public:
        //SimpleController() = default;
        //~SimpleController() {}
        virtual Pose convert(const double longitudinal, const double lateral, const Pose state) const override;
    };

    class Vehicle {
        const std::unique_ptr<Controller> _controller;
    public:
        Vehicle(std::unique_ptr<Controller> controller
                , const Shape shape = Shape{100, 20}
                , const U::Var<Pose> = U::Var(Pose{0,0,0})
                , const std::string& type = "car")
            : _controller{std::move(controller)}
            , shape{shape}
            , pose{pose}
            , type{type}
            {};
        const Shape shape;
        U::Var<Pose> pose;
        const std::string type;
        void drive(double longitudinal, double lateral);
    };

}

#endif //EPIPHANY_POSE_H
