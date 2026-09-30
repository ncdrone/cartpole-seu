/** @file Plant.cpp */
#include "fsw/Plant.hpp"
#include <cmath>

namespace fsw {

void plant_params_default(PlantParams& pp) {
    pp.cart_mass = cfg::CART_MASS_KG;
    pp.pole_mass = cfg::POLE_MASS_KG;
    pp.half_len  = cfg::POLE_HALF_LEN_M;
    pp.gravity   = cfg::GRAVITY_MS2;
}

/** Derivative of the state vector (Florian eqs., theta from upright). */
static PlantState deriv(const PlantParams& pp, const PlantState& s, F64 u) {
    const F64 tm = pp.cart_mass + pp.pole_mass;
    const F64 c = std::cos(s.theta), sn = std::sin(s.theta);
    const F64 tmp = (u + pp.pole_mass * pp.half_len * s.thetadot * s.thetadot * sn) / tm;
    const F64 thdd = (pp.gravity * sn - c * tmp) /
                     (pp.half_len * (4.0 / 3.0 - pp.pole_mass * c * c / tm));
    const F64 xdd = tmp - pp.pole_mass * pp.half_len * thdd * c / tm;
    PlantState d = { s.xdot, xdd, s.thetadot, thdd };
    return d;
}

static PlantState axpy(const PlantState& s, const PlantState& d, F64 h) {
    PlantState r = { s.x + h * d.x, s.xdot + h * d.xdot,
                     s.theta + h * d.theta, s.thetadot + h * d.thetadot };
    return r;
}

void plant_step(const PlantParams& pp, PlantState& s, F64 u, F64 dt) {
    const PlantState k1 = deriv(pp, s, u);
    const PlantState k2 = deriv(pp, axpy(s, k1, dt * 0.5), u);
    const PlantState k3 = deriv(pp, axpy(s, k2, dt * 0.5), u);
    const PlantState k4 = deriv(pp, axpy(s, k3, dt), u);
    const F64 h = dt / 6.0;
    s.x        += h * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    s.xdot     += h * (k1.xdot + 2.0 * k2.xdot + 2.0 * k3.xdot + k4.xdot);
    s.theta    += h * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
    s.thetadot += h * (k1.thetadot + 2.0 * k2.thetadot + 2.0 * k3.thetadot + k4.thetadot);
}

} // namespace fsw
