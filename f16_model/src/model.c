#include "fr_model.h"
#include "fr_plugin.h"
#include "hashmap.h"
#include "hifi_F16_AeroData.h"
#include "leading_edge_flap.h"
#include "lofi_F16_AeroData.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK_LOAD(res, name)                                                  \
  if (res < 0) {                                                               \
    error_("failed to load: %s", name);                                        \
    free(temp);                                                                \
    return res;                                                                \
  }

static PlantConstants consts = {.m = 636.94,
                                .b = 30.0,
                                .s = 300.0,
                                .c_bar = 11.32,
                                .x_cg_r = 0.35,
                                .x_cg = 0.30,
                                .h_eng = 0.0,
                                .j_y = 55814.0,
                                .j_xz = 982.0,
                                .j_z = 63100.0,
                                .j_x = 9496.0};

static HashMap *LEFMap = NULL;

static int fi_flag = 1;
Logger frplugin_log = NULL;

void frplugin_register_logger(Logger cb) { frplugin_log = cb; }

int frplugin_install_hook(int argc, char **argv) {
  int r = 0;

  if (argc < 1) {
    warn("argc is %d, should be at least 1", argc);
  }

  char *data_dir = argv[0];

  if (data_dir == NULL) {
    warn("data_dir is NULL and it's set to current");
    data_dir = "";
  } else if (strlen(data_dir) == 0) {
    info("data_dir is set to current");
    data_dir = "";
  } else {
    info("data_dir is set to %s", data_dir);
  }

  set_data_dir(data_dir);

  if (argc == 2) {
    char *fi_flag_str = argv[1];
    debug("fi_flag pointer address: %p", fi_flag_str);
    if (fi_flag_str == NULL) {
      warn("fi_flag is NULL, it's set to default 1");
    } else if (strlen(fi_flag_str) == 0) {
      warn("fi_flag is empty, it's set to default 1");
    } else if (strcmp(fi_flag_str, "1") == 0) {
      info("fi_flag is set to hifi");
    } else if (strcmp(fi_flag_str, "0") == 0) {
      info("fi_flag is set to lofi");
    }
  }

  r = init_hifi_data();
  if (r < 0) {
    return r;
  }

  r = init_axis_data();

  LEFMap = create_hashmap(100);

  return r;
}

int frplugin_uninstall_hook(int argc, char **argv) {
  free_hifi_data();
  trace("free hifi data successfully");

  free_axis_data();
  trace("free axis data successfully");

  free_hashmap(LEFMap);

  return 0;
}

int frmodel_load_constants(PlantConstants *constants) {
  constants->m = consts.m;
  constants->b = consts.b;
  constants->s = consts.s;
  constants->c_bar = consts.c_bar;
  constants->x_cg_r = consts.x_cg_r;
  constants->x_cg = consts.x_cg;
  constants->h_eng = consts.h_eng;
  constants->j_y = consts.j_y;
  constants->j_z = consts.j_z;
  constants->j_xz = consts.j_xz;
  constants->j_x = consts.j_x;

  trace("f16 consts load successfully");

  return 0;
};

int frmodel_load_ctrl_limits(ControlLimit *ctrl_limits) {
  ControlLimit limits = {.thrust_cmd_limit_top = 19000.0,
                         .thrust_cmd_limit_bottom = 1000.0,
                         .thrust_rate_limit = 10000.0,
                         .ele_cmd_limit_top = 25.0,
                         .ele_cmd_limit_bottom = -25.0,
                         .ele_rate_limit = 60.0,
                         .ail_cmd_limit_top = 21.5,
                         .ail_cmd_limit_bottom = -21.5,
                         .ail_rate_limit = 80.0,
                         .rud_cmd_limit_top = 30.0,
                         .rud_cmd_limit_bottom = -30.0,
                         .rud_rate_limit = 120.0,
                         .alpha_limit_top = 45.0,
                         .alpha_limit_bottom = -20.0,
                         .beta_limit_top = 30.0,
                         .beta_limit_bottom = -30.0};
  *ctrl_limits = limits;

  return 0;
}

static int frmodel_step_helper(const State *state, const Control *control,
                               double d_lef, C *c) {
  double m = consts.m;
  double B = consts.b;
  double S = consts.s;
  double cbar = consts.c_bar;
  double xcgr = consts.x_cg_r;
  double xcg = consts.x_cg;

  double Heng = consts.h_eng;

  double Jy = consts.j_y;
  double Jxz = consts.j_xz;
  double Jz = consts.j_z;
  double Jx = consts.j_x;

  double *temp;
  int res = 0;

  double vt, alpha, beta, P, Q, R;
  double el, ail, rud, dail, drud, lef, dlef;

  double Cx_tot, Cx, delta_Cx_lef, dXdQ, Cxq, delta_Cxq_lef;
  double Cz_tot, Cz, delta_Cz_lef, dZdQ, Czq, delta_Czq_lef;
  double Cm_tot, Cm, eta_el, delta_Cm_lef, dMdQ, Cmq, delta_Cmq_lef, delta_Cm,
      delta_Cm_ds;
  double Cy_tot, Cy, delta_Cy_lef, dYdail, delta_Cy_r30, dYdR, dYdP;
  double delta_Cy_a20, delta_Cy_a20_lef, Cyr, delta_Cyr_lef, Cyp, delta_Cyp_lef;
  double Cn_tot, Cn, delta_Cn_lef, dNdail, delta_Cn_r30, dNdR, dNdP,
      delta_Cnbeta;
  double delta_Cn_a20, delta_Cn_a20_lef, Cnr, delta_Cnr_lef, Cnp, delta_Cnp_lef;
  double Cl_tot, Cl, delta_Cl_lef, dLdail, delta_Cl_r30, dLdR, dLdP,
      delta_Clbeta;
  double delta_Cl_a20, delta_Cl_a20_lef, Clr, delta_Clr_lef, Clp, delta_Clp_lef;

  temp = (double *)malloc(9 * sizeof(double));

  vt = state->velocity;         /* total velocity */
  alpha = state->alpha * r2d(); /* angle of attack in degrees */
  beta = state->beta * r2d();   /* sideslip angle in degrees */
  P = state->p;                 /* Roll Rate --- rolling  moment is Lbar */
  Q = state->q;                 /* Pitch Rate--- pitching moment is M */
  R = state->r;                 /* Yaw Rate  --- yawing   moment is N */

  if (vt <= 0.01) {
    vt = 0.01;
  }

  /// Control Input

  el = control->elevator; /* Elevator setting in degrees. */
  ail = control->aileron; /* Ailerons mex setting in degrees. */
  rud = control->rudder;  /* Rudder setting in degrees. */
  lef = d_lef;            /* Leading edge flap setting in degrees */

  /* dail  = ail/20.0;   aileron normalized against max angle */
  /* The aileron was normalized using 20.0 but the NASA report and
     S&L both have 21.5 deg. as maximum deflection. */
  /* As a result... */

  dail = ail / 21.5;
  drud = rud / 30.0;
  dlef = (1 - lef / 25.0);

  if (fi_flag == 1) {
    /// Hifi Table Look-Up
    res = hifi_C(alpha, beta, el, temp);
    CHECK_LOAD(res, "hifi_C")
    Cx = temp[0];
    Cz = temp[1];
    Cm = temp[2];
    Cy = temp[3];
    Cn = temp[4];
    Cl = temp[5];

    res = hifi_damping(alpha, temp);
    CHECK_LOAD(res, "hifi_damping")
    Cxq = temp[0];
    Cyr = temp[1];
    Cyp = temp[2];
    Czq = temp[3];
    Clr = temp[4];
    Clp = temp[5];
    Cmq = temp[6];
    Cnr = temp[7];
    Cnp = temp[8];

    res = hifi_C_lef(alpha, beta, temp);
    CHECK_LOAD(res, "hifi_C_lef")
    delta_Cx_lef = temp[0];
    delta_Cz_lef = temp[1];
    delta_Cm_lef = temp[2];
    delta_Cy_lef = temp[3];
    delta_Cn_lef = temp[4];
    delta_Cl_lef = temp[5];

    res = hifi_damping_lef(alpha, temp);
    CHECK_LOAD(res, "hifi_damping_lef")
    delta_Cxq_lef = temp[0];
    delta_Cyr_lef = temp[1];
    delta_Cyp_lef = temp[2];
    delta_Czq_lef = temp[3];
    delta_Clr_lef = temp[4];
    delta_Clp_lef = temp[5];
    delta_Cmq_lef = temp[6];
    delta_Cnr_lef = temp[7];
    delta_Cnp_lef = temp[8];

    res = hifi_rudder(alpha, beta, temp);
    CHECK_LOAD(res, "hifi_rudder")
    delta_Cy_r30 = temp[0];
    delta_Cn_r30 = temp[1];
    delta_Cl_r30 = temp[2];

    res = hifi_ailerons(alpha, beta, temp);
    CHECK_LOAD(res, "hifi_ailerons")
    delta_Cy_a20 = temp[0];
    delta_Cy_a20_lef = temp[1];
    delta_Cn_a20 = temp[2];
    delta_Cn_a20_lef = temp[3];
    delta_Cl_a20 = temp[4];
    delta_Cl_a20_lef = temp[5];

    res = hifi_other_coeffs(alpha, el, temp);
    CHECK_LOAD(res, "hifi_other_coeffs")
    delta_Cnbeta = temp[0];
    delta_Clbeta = temp[1];
    delta_Cm = temp[2];
    eta_el = temp[3];
    delta_Cm_ds = 0; // ignore deep-stall effect
  } else if (fi_flag == 0) {
    /// Lofi Table Look-Up

    /*
       The lofi model does not include the
       leading edge flap.  All terms multiplied
       dlef have been set to zero but just to
       be sure we will set it to zero.
    */

    dlef = 0.0;

    damping(alpha, temp);
    Cxq = temp[0];
    Cyr = temp[1];
    Cyp = temp[2];
    Czq = temp[3];
    Clr = temp[4];
    Clp = temp[5];
    Cmq = temp[6];
    Cnr = temp[7];
    Cnp = temp[8];

    dmomdcon(alpha, beta, temp);
    delta_Cl_a20 = temp[0];
    delta_Cl_r30 = temp[1];
    delta_Cn_a20 = temp[2];
    delta_Cn_r30 = temp[3];

    clcn(alpha, beta, temp);
    Cl = temp[0];
    Cn = temp[1];

    cxcm(alpha, el, temp);
    Cx = temp[0];
    Cm = temp[1];

    Cy = -.02 * beta + .021 * dail + .086 * drud;

    cz(alpha, beta, el, temp);
    Cz = temp[0];

    /*
       Set all higher order terms of hifi that are
       not applicable to lofi equal to zero.
    */

    delta_Cx_lef = 0.0;
    delta_Cz_lef = 0.0;
    delta_Cm_lef = 0.0;
    delta_Cy_lef = 0.0;
    delta_Cn_lef = 0.0;
    delta_Cl_lef = 0.0;
    delta_Cxq_lef = 0.0;
    delta_Cyr_lef = 0.0;
    delta_Cyp_lef = 0.0;
    delta_Czq_lef = 0.0;
    delta_Clr_lef = 0.0;
    delta_Clp_lef = 0.0;
    delta_Cmq_lef = 0.0;
    delta_Cnr_lef = 0.0;
    delta_Cnp_lef = 0.0;
    delta_Cy_r30 = 0.0;
    delta_Cy_a20 = 0.0;
    delta_Cy_a20_lef = 0.0;
    delta_Cn_a20_lef = 0.0;
    delta_Cl_a20_lef = 0.0;
    delta_Cnbeta = 0.0;
    delta_Clbeta = 0.0;
    delta_Cm = 0.0;
    eta_el = 1.0; /* Needs to be one. See equation for Cm_tot*/
    delta_Cm_ds = 0.0;
  }

  /// compute Cx_tot, Cz_tot, Cm_tot, Cy_tot, Cn_tot, and Cl_tot
  /// (as on NASA report p37-40)

#pragma region Cx_tot

  dXdQ = (cbar / (2 * vt)) * (Cxq + delta_Cxq_lef * dlef);

  Cx_tot = Cx + delta_Cx_lef * dlef + dXdQ * Q;

#pragma endregion

#pragma region Cz_tot

  dZdQ = (cbar / (2 * vt)) * (Czq + delta_Cz_lef * dlef);

  Cz_tot = Cz + delta_Cz_lef * dlef + dZdQ * Q;

#pragma endregion

#pragma region Cm_tot

  dMdQ = (cbar / (2 * vt)) * (Cmq + delta_Cmq_lef * dlef);

  Cm_tot = Cm * eta_el + Cz_tot * (xcgr - xcg) + delta_Cm_lef * dlef +
           dMdQ * Q + delta_Cm + delta_Cm_ds;

#pragma endregion

#pragma region Cy_tot

  dYdail = delta_Cy_a20 + delta_Cy_a20_lef * dlef;

  dYdR = (B / (2 * vt)) * (Cyr + delta_Cyr_lef * dlef);

  dYdP = (B / (2 * vt)) * (Cyp + delta_Cyp_lef * dlef);

  Cy_tot = Cy + delta_Cy_lef * dlef + dYdail * dail + delta_Cy_r30 * drud +
           dYdR * R + dYdP * P;

#pragma endregion

#pragma region Cn_tot

  dNdail = delta_Cn_a20 + delta_Cn_a20_lef * dlef;

  dNdR = (B / (2 * vt)) * (Cnr + delta_Cnr_lef * dlef);

  dNdP = (B / (2 * vt)) * (Cnp + delta_Cnp_lef * dlef);

  Cn_tot = Cn + delta_Cn_lef * dlef - Cy_tot * (xcgr - xcg) * (cbar / B) +
           dNdail * dail + delta_Cn_r30 * drud + dNdR * R + dNdP * P +
           delta_Cnbeta * beta;

#pragma endregion

#pragma region Cl_tot

  dLdail = delta_Cl_a20 + delta_Cl_a20_lef * dlef;

  dLdR = (B / (2 * vt)) * (Clr + delta_Clr_lef * dlef);

  dLdP = (B / (2 * vt)) * (Clp + delta_Clp_lef * dlef);

  Cl_tot = Cl + delta_Cl_lef * dlef + dLdail * dail + delta_Cl_r30 * drud +
           dLdR * R + dLdP * P + delta_Clbeta * beta;

#pragma endregion

  trace("f16 coeff:Cl=%f, Cm=%f, Cn=%f,Cx=%f, Cy=%f, Cz=%f", Cl_tot, Cm_tot,
        Cn_tot, Cx_tot, Cy_tot, Cz_tot);

  c->c_l = Cl_tot;
  c->c_m = Cm_tot;
  c->c_n = Cn_tot;
  c->c_x = Cx_tot;
  c->c_y = Cy_tot;
  c->c_z = Cz_tot;

  free(temp);

  return 0;
};

int frmodel_trim(const State *state, const Control *control, C *c) {
  trace("f16 trim start");
  int r = 0;

  double lef = get_lef(state);

  r = frmodel_step_helper(state, control, lef, c);

  trace("f16 trim finished");
  return r;
}

int frmodel_init(const char *id, const State *state, const Control *control) {
  trace("f16 %s init start", id);
  int r = 0;

  LeadingEdgeFlapBlock *lef_block_ptr = malloc(sizeof(LeadingEdgeFlapBlock));
  lef_new(lef_block_ptr, state);
  hashmap_insert(LEFMap, id, lef_block_ptr);

  trace("f16 %s init finished", id);
  return r;
}

int frmodel_step(const char *id, const State *state, const Control *control,
                 double t, C *c) {
  trace("[t: %f] f16 step start", t);

  int r = 0;
  double lef = 0.0;
  LeadingEdgeFlapBlock *lef_block_ptr = hashmap_get(LEFMap, id);
  if (lef_block_ptr == NULL) {
    error_("[t: %f] f16 %s step failed to get lef", t, id);
    return -1;
  }

  r = lef_update(lef_block_ptr, state, t, &lef);
  if (r < 0) {
    error_("[t: %f] f16 %s step failed to update lef", t, id);
    return r;
  }

  r = frmodel_step_helper(state, control, lef, c);

  trace("[t: %f] f16 %s step finished", t, id);
  return r;
}

int frmodel_delete(const char *id) {
  int r = 0;

  LeadingEdgeFlapBlock *lef_block_ptr = hashmap_remove(LEFMap, id);

  if (lef_block_ptr != NULL) {
    lef_drop(lef_block_ptr);
    free(lef_block_ptr);
  } else {
    error_("failed to find lef %s", id);
    return -1;
  }

  return r;
}