//
// Created by sigi on 06.06.19.
//

#ifndef EPIPHANY_POSE_H
#define EPIPHANY_POSE_H

#include "U.h"
#include <memory>
#include <ostream>
#include <opencv2/core.hpp>
#include <UMath.h>

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
    using State = cv::Vec6d;

    class System{
        double stepSize;
        double m = 1700;    /* Vehicle mass.                    */
        double a = 1.5;     /* Distance from front axle to COG.  */
        double b = 1.5;     /* Distance from rear axle to COG.  */
        double Cx = 1.5E5;  /* Longitudinal tire stiffness.     */
        double Cy = 4E4;    /* Lateral tire stiffness.          */
        double CA = 1.5;   /* Air resistance coefficient.      */
        double g = 9.81;
        double Cr = 1.0;   /* Rolling resistance coefficient  */
        double Fr = m*g*Cr;
        State x;
    public:
        System(const State initialState, const double stepSize) :
                x{initialState}
                , stepSize{stepSize}
                {}

        State fxu(State x, const double u_F, const double u_phi) {

            using namespace U::Math;
            std::cout<<"u phi: "<< u_phi<<"\n";
            double beta =atand(b*tand(u_phi)/(a+b));
            double cos_Uphi = cosd(u_phi);
            double sin_Uphi = sind(u_phi);
            double v_x = x[3];
            double v_y = x[4];
            double threshold_vx = 1009;
            std::cout<<"beta: "<< beta <<"\n";
            double Fw_x = v_x > threshold_vx ? 2*Cy*(u_phi - (v_y+a*x[5])/v_x)*sin_Uphi : 0;
            double Fw_y = v_x > threshold_vx ? 2*Cx*u_F*sin_Uphi
                                               + 2*Cy*(u_phi - (v_y +a*x[5])/v_x)*cos_Uphi
                                               + 2*Cy*(b*x[5] - v_y)/v_x  : 0;
            double Fw_phi = v_x > threshold_vx ? a*(2*Cx*u_F*sin_Uphi + 2*Cy*(u_phi - (v_y+a*x[5])/v_x)*cos_Uphi)
                                                 -2*b*Cy*(b*x[5] -v_y)/v_x : 0;
            double sign_v = sgn(v_x);
            double v = sign_v*sqrt(v_x*v_x + v_y*v_y);
            State dx{v_x*cosd(x[2]) - v_y*sind(x[2])// dx
                    , v_x*sind(x[2]) + v_y*cosd(x[2]) // dy
                    , v/(b)*sind(beta) //d_phi
                    , v_y*x[5] + 1/m* (2*Cx*u_F*cos_Uphi
                                       - Fw_x
                                       - sign_v*(CA*v*v + Fr))
                    , -v_x*x[5] + 1/m* (Fw_y)
                    ,  4/(m*pow(a+b,2))*(Fw_phi)
            };
            return dx;
        }
        State next(double u_F, double u_phi){
            x = x + stepSize*runge4(x, u_F/2, 75* u_phi, stepSize);
            std::cout<<"State: " << x << "\n";
            return x;
        }

        State runge4(State xk, double u_F, double u_phi, double stepSize){
            State k1 = fxu(xk, u_F, u_phi);
            State k2 = fxu(xk +stepSize/2*k1, u_F, u_phi);
            State k3 = fxu(xk +stepSize/2*k2, u_F, u_phi);
            State k4 = fxu(xk +stepSize*k3, u_F, u_phi);
            return (k1+k4)/6 +(k2+k3)/3;
        }

        State rungeMerson(State xk, double u_F, double u_phi, double stepSize){
            State k1 = fxu(xk, u_F, u_phi);
            State k2 = fxu(xk +stepSize/3*k1, u_F, u_phi);
            State k3 = fxu(xk +stepSize/6*(k1+k2), u_F, u_phi);
            State k4 = fxu(xk +stepSize/8*(k1+3*k2), u_F, u_phi);
            State k5 = fxu(xk +0.5*stepSize*(k1-3*k3+4*k4), u_F, u_phi);

            return (k1 + 4*k4 + k5)/6 + (-2*k1 + 9*k3 - 8*k4 + k5)/30;
        }
    };
    class Controller{
    public:
        virtual ~Controller() = default;
        virtual State convert(const double longitudinal, const double lateral, const State state) = 0;
        //virtual Pose convert(double longitudinal, double lateral, State state) = 0;
    };

    class SimpleController : public Controller {
    public:
        ~SimpleController() = default;
        virtual State convert(const double longitudinal, const double lateral, const State state) override;
    };

    class DynamicModel : public Controller {
    private:
        System sys;
    public:
        DynamicModel(System sys) :
        sys(std::move(sys)) {}

        ~DynamicModel() = default;
        virtual State convert(const double longitudinal, const double lateral, const State state) override;
    };

    class Vehicle {
        const std::unique_ptr<Controller> _controller;
    public:
        Vehicle(std::unique_ptr<Controller> controller
                , const Pose pose = Pose{0,0,0}
                , const Shape shape = Shape{100, 20}
                , const std::string& type = "car")
            : _controller{std::move(controller)}
            , shape{shape}
            , pose{U::Var{pose}}
            , type{type}
            , state{pose.x, pose.y, pose.phi, 0,0,0}
            {};
        const Shape shape;
        U::Var<Pose> pose;
        State state;
        const std::string type;
        void drive(double longitudinal, double lateral);
    };

}

#endif //EPIPHANY_POSE_H
