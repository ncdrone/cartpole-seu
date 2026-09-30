/** @file Plant.hpp  @brief Cart-pole simulation (POD state + step function). */
#ifndef FSW_PLANT_HPP
#define FSW_PLANT_HPP
#include "fsw/FswTypes.hpp"
#include "fsw/Config.hpp"

namespace fsw {

/** Plant parameters (POD). */
struct PlantParams {
    F64 cart_mass, pole_mass, half_len, gravity;
};

/** Plant state; theta = 0 is upright. */
struct PlantState {
    F64 x, xdot, theta, thetadot;
};

/** @brief Standard plant parameters from Config.hpp. */
void plant_params_default(PlantParams& pp);

/**
 * @brief Advance plant by dt with force u using RK4 (force held constant).
 * Dynamics: Florian, "Correct equations for the dynamics of the cart-pole
 * system" (2007), frictionless; same form as Barto/Sutton/Anderson (1983).
 */
void plant_step(const PlantParams& pp, PlantState& s, F64 u, F64 dt);

} // namespace fsw
#endif
