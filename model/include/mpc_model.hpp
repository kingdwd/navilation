//
// Created by sigi on 20.07.19.
//

#ifndef EPIPHANY_MPCMODEL_HPP
#define EPIPHANY_MPCMODEL_HPP
/* square macro */
#define POW2(a) ((a)*(a))

#include "problem_description.hpp"

#undef EPS
#include <math.h>
#include <math_util.h>
#include "model_const.hpp"
#include "dynamic_model.hpp"

namespace epi {

    using namespace con;
    class MpcModel : public grampc::ProblemDescription {

        DynamicCarModel _model;
    /**
     * final state cost parameters
     */
    typeRNum* FSC;

    /**
     * state cost parameters
     */
    typeRNum* SC;

    /**
     * input cost parameters
     */
    typeRNum* IC;

public:
    static constexpr typeInt X_DIM = 5;
    static constexpr typeInt U_DIM = 2;
    MpcModel(double finalStateCost[5], double stateCost[5], double inputCost[2])
    : FSC(finalStateCost), SC(stateCost), IC(inputCost) {}

    /** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng),
        inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
    void ocp_dim(typeInt *Nx, typeInt *Nu
                , typeInt *Np, typeInt *Ng
                , typeInt *Nh, typeInt *NgT
                , typeInt *NhT) {
        *Nx = X_DIM;
        *Nu = U_DIM;
        *Np = 0;
        *Ng = 0;
        *Nh = 0;
        *NgT =2;
        *NhT =0;
    }


    /** System function f(t,x,u,p,userparam)
    ------------------------------------ **/
    void ffct(typeRNum *out, ctypeRNum t
                    , ctypeRNum *x, ctypeRNum *u
                    , ctypeRNum *p) override {
        using namespace U::Math;
        using namespace con;
        //double beta =atan(b*tan(phi)/(a+b));

        State state = _model.dx(State{x}, u[0], u[1]);
        out[0] = state[0];
        out[1] = state[1];
        out[2] = state[2];
        out[3] = state[3];
        out[4] = state[4];
    }
    /** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
    void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *adj, ctypeRNum *u, ctypeRNum *p) override {
        double l = (a + b); // car length
        double k = 2*CA/m;

        typeRNum v = sqrt(x[3]*x[3] + x[4]*x[4]);
        typeRNum sign_v = U::Math::sgn(x[3]);

        typeRNum dv_x = v== 0.0 ? 1 : x[3]/v;
        typeRNum dv_y = v== 0.0 ? 1 : x[4]/v;
        out[0] = 0;
        out[1] = 0;
        out[2] = (x[3]*cos(x[2])-x[4]*sin(x[2]))*adj[1]-(x[3]*sin(x[2]) + x[4]*cos(x[2]))*adj[0];
        out[3] = cos(x[2])*adj[0] + sin(x[2])*adj[1] + dv_x*u[1]/l*adj[2]
                - k*abs(x[3])*adj[3];
        out[4] = -sin(x[2])*adj[0] + cos(x[2])*adj[1] + dv_y*u[1]/l*adj[2] - k*abs(x[4])*adj[4];
    }
    /** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
    void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *adj, ctypeRNum *u, ctypeRNum *p) override {
        double l = (a + b); // car length
        typeRNum sign_v = U::Math::sgn(x[3]);
        typeRNum v = sign_v*sqrt(x[3]*x[3] + x[4]*x[4]);

        out[0] = P/m*adj[3];
        out[1] = x[3]/(a+b)*adj[2];
    }
    /** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
    void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *adj, ctypeRNum *u, ctypeRNum *p) override {}


    /** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam)
    -------------------------------------------------- **/
    void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override {
        out[0] =  SC[0] * POW2(x[0] - xdes[0])
            + SC[1] * POW2(x[1] - xdes[1])
            + SC[2] * POW2(x[2] - xdes[2])
            + SC[3] * POW2(x[3] - xdes[3])
            + SC[4] * POW2(x[4] - xdes[4])
            + IC[0] * POW2(u[0] - udes[0])
            + IC[1] * POW2(u[1] - udes[1]);
    }
    /** Gradient dl/dx **/
    void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override {
        out[0] = 2 * SC[0] * (x[0] - xdes[0]);
        out[1] = 2 * SC[1] * (x[1] - xdes[1]);
        out[2] = 2 * SC[2] * (x[2] - xdes[2]);
        out[3] = 2 * SC[3] * (x[3] - xdes[3]);
        out[4] = 2 * SC[4] * (x[4] - xdes[4]);
    }
    /** Gradient dl/du **/
    void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override {
        out[0] = 2 * IC[0] * (u[0] - udes[0]);
        out[1] = 2 * IC[1] * (u[1] - udes[1]);
    }
    /** Gradient dl/dp **/
    void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes) override {}

    /** Terminal cost V(T,x,p) */
    void Vfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override {
	out[0] = FSC[0] * POW2(x[0] - xdes[0])
		+ FSC[1] * POW2(x[1] - xdes[1])
		+ FSC[2] * POW2(x[2] - xdes[2])
		+ FSC[3] * POW2(x[3] - xdes[3])
		+ FSC[4] * POW2(x[4] - xdes[4]);
    }
    /** Gradient dV/dx **/
    void dVdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) override {
	out[0] = 2 * FSC[0] * (x[0] - xdes[0]);
	out[1] = 2 * FSC[1] * (x[1] - xdes[1]);
	out[2] = 2 * FSC[2] * (x[2] - xdes[2]);
	out[3] = 2 * FSC[3] * (x[3] - xdes[3]);
	out[4] = 2 * FSC[4] * (x[4] - xdes[4]);
}
    /** Gradient dV/dp **/
    virtual void dVdp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) {}
    /** Gradient dV/dT **/
    virtual void dVdT(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes) {
        out[0] = 0;
    }


    /** Equality constraints g(t,x,u,p) = 0 */
    virtual void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) {}
    /** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
    virtual void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
    virtual void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
    virtual void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}


    /** Inequality constraints h(t,x,u,p) < 0 */
    virtual void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p) {}
    /** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
    virtual void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
    virtual void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
    virtual void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec) {}


    /** Terminal equality constraints gT(T,x,p) = 0 */
    virtual void gTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p) {
        out[0] = x[3];
        out[1] = x[4];
    }
    /** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
    virtual void dgTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {
        out[0] = vec[3];
        out[1] = vec[4];
    }
    /** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
    virtual void dgTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
    virtual void dgTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {}


    /** Terminal inequality constraints hT(T,x,p) < 0 */
    virtual void hTfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p) {}
    /** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
    virtual void dhTdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
    virtual void dhTdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {}
    /** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
    virtual void dhTdT_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec) {}
};

}
#endif //EPIPHANY_MPCMODEL_HPP
