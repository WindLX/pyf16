#include "leading_edge_flap.h"
#include "fr_plugin.h"
#include "utils.h"
#include <stdlib.h>

AtmosFunc frplugin_atmos = NULL;

IntegratorNew frplugin_integrator_new = NULL;
IntegratorDrop frplugin_integrator_drop = NULL;
IntegratorUpdate frplugin_integrator_update = NULL;
IntegratorReset frplugin_integrator_reset = NULL;
IntegratorPast frplugin_integrator_past = NULL;

ActuatorNew frplugin_actuator_new = NULL;
ActuatorDrop frplugin_actuator_drop = NULL;
ActuatorUpdate frplugin_actuator_update = NULL;
ActuatorReset frplugin_actuator_reset = NULL;
ActuatorPast frplugin_actuator_past = NULL;

void frplugin_register_atmos(AtmosFunc atmos) { frplugin_atmos = atmos; }

void frplugin_register_integrator_new(IntegratorNew integrator_new) {
  frplugin_integrator_new = integrator_new;
}

void frplugin_register_integrator_drop(IntegratorDrop integrator_drop) {
  frplugin_integrator_drop = integrator_drop;
}

void frplugin_register_integrator_update(IntegratorUpdate integrator_update) {
  frplugin_integrator_update = integrator_update;
}

void frplugin_register_integrator_reset(IntegratorReset integrator_reset) {
  frplugin_integrator_reset = integrator_reset;
}

void frplugin_register_integrator_past(IntegratorPast integrator_past) {
  frplugin_integrator_past = integrator_past;
}

void frplugin_register_actuator_new(ActuatorNew actuator_new) {
  frplugin_actuator_new = actuator_new;
}

void frplugin_register_actuator_drop(ActuatorDrop actuator_drop) {
  frplugin_actuator_drop = actuator_drop;
}

void frplugin_register_actuator_update(ActuatorUpdate actuator_update) {
  frplugin_actuator_update = actuator_update;
}

void frplugin_register_actuator_reset(ActuatorReset actuator_reset) {
  frplugin_actuator_reset = actuator_reset;
}

void frplugin_register_actuator_past(ActuatorPast actuator_past) {
  frplugin_actuator_past = actuator_past;
}

double get_lef(const State *state) {
  Atmos atmos = frplugin_atmos(state->altitude, state->velocity);
  double lef =
      1.38 * state->alpha * r2d() - 9.05 * atmos.qbar / atmos.ps + 1.45;

  lef = clamp(lef, 0.0, 25.0);
  return lef;
}

void lef_new(LeadingEdgeFlapBlock *lef, const State *state) {
  double d_lef = get_lef(state);
  trace("LEFBlock: alpha_init: %.2f, d_lef: %.2f", state->alpha, d_lef);
  lef->lef_actuator =
      frplugin_actuator_new(d_lef, 25.0, 0.0, 25.0, 1.0 / 0.136);
  lef->integrator = frplugin_integrator_new(-state->alpha * r2d());
  lef->feedback = 0.0;
}

int lef_update(LeadingEdgeFlapBlock *lef, const State *state, double t,
               double *result) {
  int r = 0;
  trace("[t: %f] LEFBlock: alpha: %f, altitude: %f, velocity: %f", t,
        state->alpha, state->altitude, state->velocity);
  Atmos atmos = frplugin_atmos(state->altitude, state->velocity);
  double r_1 = atmos.qbar / atmos.ps * 9.05;
  double alpha = state->alpha * r2d();
  double r_2 = (alpha - lef->feedback) * 7.25;
  double r_3 = 0.0;
  r = frplugin_integrator_update(lef->integrator, r_2, t, &r_3);

  if (r < 0) {
    debug("[t: %f] LEFBlock: integrator error", t);
    return r;
  }

  double r_4 = r_3 + 2.0 * alpha;
  lef->feedback = r_4;
  double r_5 = r_4 * 1.38;
  double r_6 = 0.0;
  r = frplugin_actuator_update(lef->lef_actuator, (1.45 + r_5 - r_1), t, &r_6);

  if (r < 0) {
    debug("[t: %f] LEFBlock: actuator error", t);
    return r;
  }

  trace("[t: %f] LEFBlock: lef: %f", t, r_6);
  *result = r_6;
  return r;
}

int lef_past(LeadingEdgeFlapBlock *lef, const State *state, double *result) {
  int r = 0;
  r = frplugin_actuator_past(lef->lef_actuator, result);
  return r;
}

int lef_reset(LeadingEdgeFlapBlock *lef, const State *state) {
  int r = 0;
  r = frplugin_actuator_reset(lef->lef_actuator);
  if (r < 0) {
    debug("LEFBlock: actuator reset error");
    return r;
  }
  r = frplugin_integrator_reset(lef->integrator);
  if (r < 0) {
    debug("LEFBlock: integrator reset error");
    return r;
  }
  lef->feedback = 0.0;
  trace("LEFBlock reset finished");
  return r;
}

void lef_drop(LeadingEdgeFlapBlock *lef) {
  frplugin_actuator_drop(lef->lef_actuator);
  frplugin_integrator_drop(lef->integrator);
  trace("LEFBlock drop finished");
}