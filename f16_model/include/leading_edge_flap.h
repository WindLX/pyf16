#ifndef LEADING_EDGE_FLAP_H
#define LEADING_EDGE_FLAP_H

#include "fr_model.h"

double get_lef(const State *state);

typedef struct {
  void *lef_actuator;
  void *integrator;
  double feedback;
} LeadingEdgeFlapBlock;

void lef_new(LeadingEdgeFlapBlock *lef, const State *state);
int lef_update(LeadingEdgeFlapBlock *lef, const State *state, double t,
               double *result);
int lef_past(LeadingEdgeFlapBlock *lef, const State *state, double *result);
int lef_reset(LeadingEdgeFlapBlock *lef, const State *state);
void lef_drop(LeadingEdgeFlapBlock *lef);

#endif // LEADING_EDGE_FLAP_H