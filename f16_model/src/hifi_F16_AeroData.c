#include "hifi_F16_AeroData.h"
#include "mexndinterp.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DATA_LEN 44
#define GET_BIT(num, pos) ((num >> pos) & 1)

#define GET_COEFF_ALPHA(axisIndex, hifiIndex)                                  \
  double targetData[1] = {alpha};                                              \
  double **axis = get_axis_data(axisIndex);                                    \
  double r = interpn(axis, hifiData[hifiIndex], targetData);                   \
  free(axis);                                                                  \
  return r

#define GET_COEFF2(axisIndex, hifiIndex)                                       \
  double targetData[2] = {alpha, beta};                                        \
  double **axis = get_axis_data(axisIndex);                                    \
  double r = interpn(axis, hifiData[hifiIndex], targetData);                   \
  free(axis);                                                                  \
  return r

#define GET_COEFF3(axisIndex, hifiIndex)                                       \
  double targetData[3] = {alpha, beta, dele};                                  \
  double **axis = get_axis_data(axisIndex);                                    \
  double r = interpn(axis, hifiData[hifiIndex], targetData);                   \
  free(axis);                                                                  \
  return r

#define CHECK_NAN(len)                                                         \
  for (int i = 0; i < len; i++) {                                              \
    if (isnan(retVal[i])) {                                                    \
      trace("check NAN fail at: (%d, %d)", len, i);                            \
      return -1;                                                               \
    }                                                                          \
  }                                                                            \
  return 0

static Tensor **hifiData;
static double **axisData;
static char *dataDir;

typedef enum {
  ALPHA1 = 0,
  ALPHA2,
  DH1,
  ALPHA1_BETA1,
  ALPHA2_BETA1,
  ALPHA1_BETA1_DH1,
  ALPHA1_BETA1_DH2
} AxisDataIndex;

typedef enum {
  CL0120 = 0,
  CL0620,
  CL0720,
  CL0820,
  CL0920,
  CL1220,
  CL1320,
  CL1520,
  CL1620,
  CL9999,
  CM0120,
  CM0820,
  CM1020,
  CM1120,
  CM1420,
  CM9999,
  CN0120,
  CN0620,
  CN0720,
  CN0820,
  CN0920,
  CN1220,
  CN1320,
  CN1520,
  CN1620,
  CN9999,
  CX0120,
  CX0820,
  CX1120,
  CX1420,
  CY0320,
  CY0620,
  CY0720,
  CY0820,
  CY0920,
  CY1220,
  CY1320,
  CY1520,
  CY1620,
  CZ0120,
  CZ0820,
  CZ1120,
  CZ1420
} HifiDataIndex;

/// @brief 加载轴向数据 alpha beta dh
/// @param fileName 数据文件名
/// @param len 数据长度
/// @return 数据
static double *load_axis_data(char *fileName, int len) {
  char filePath[256];
  if (!strcmp(dataDir, "")) {
    sprintf(filePath, "%s", fileName);
  } else {
    sprintf(filePath, "%s/%s", dataDir, fileName);
  }
  FILE *fp = fopen(filePath, "r");
  int r = 0;
  double buffer;

  if (fp == NULL) {
    error_("can't find file %s", filePath);
    trace("file_len: %d", len);
    return NULL;
  }

  double *data = create_dvector(len);

  for (int i = 0; i < len; i++) {
    r = fscanf(fp, "%lf", &buffer);
    if (r < 0) {
      fclose(fp);
      free(data);
      error_("file %s read failed", fileName);
      return NULL;
    }
    data[i] = buffer;
  }
  fclose(fp);
  info("load %s successfully", filePath);
  return data;
}

/// @brief 加载气动数据
/// @param fileName 数据文件名
/// @param n_dimension 数据维度长度
/// @param dataNameIndex 数据名索引 由四位二进制数据构成
///			0b1000: ETA_DH1
///			从第三位至第一位分别代表 ALPHA BETA DH
///			0 代表 1, 1 代表 2
///			例如: 0b100 代表 ALPHA2 BETA1 DH1
/// @return 数据
static Tensor *load_aerodynamic_data(char *fileName, int n_dimension,
                                     char dataNameIndex) {
  /**
   * dataNameIndex:
   * 	000: ALPHA1 BETA1 DH1
   * 	special 0b1000 for ETA_DH1_brett
   */
  int r = 0;
  double buffer = 0.0;
  char filePath[256];
  int fileSize = 0;
  int *n_points = (int *)malloc(n_dimension * sizeof(int));

  if (n_dimension > 0) {
    if (n_dimension == 1 && GET_BIT(dataNameIndex, 3) == 1) {
      n_points[0] = 5;
    } else if (GET_BIT(dataNameIndex, 2) == 0) {
      n_points[0] = 20;
    } else if (GET_BIT(dataNameIndex, 2) == 1) {
      n_points[0] = 14;
    } else {
      free(n_points);
      error_("invalid dataNameIndex: %d", dataNameIndex);
      return NULL;
    }
    fileSize = n_points[0];

    if (n_dimension > 1) {
      n_points[1] = 19;
      fileSize *= n_points[1];

      if (n_dimension == 3) {
        if (GET_BIT(dataNameIndex, 0) == 0) {
          n_points[2] = 5;
        } else if (GET_BIT(dataNameIndex, 0) == 1) {
          n_points[2] = 3;
        } else {
          free(n_points);
          error_("invalid dataNameIndex: %d", dataNameIndex);
          return NULL;
        }
        fileSize *= n_points[2];
      }
    }
  }

  Tensor *tensor = create_tensor(n_dimension, n_points);
  free(n_points);

  if (!strcmp(dataDir, "")) {
    sprintf(filePath, "%s", fileName);
  } else {
    sprintf(filePath, "%s/%s", dataDir, fileName);
  }
  debug("filePath: %s", filePath);

  FILE *fp = fopen(filePath, "r");
  if (fp == (FILE *)NULL) {
    free_tensor(tensor);
    error_("can't find file %s", filePath);
    return NULL;
  }

  for (int i = 0; i < fileSize; i++) {
    r = fscanf(fp, "%lf", &buffer);
    if (r < 0) {
      fclose(fp);
      free_tensor(tensor);
      error_("file %s read failed", filePath);
      return NULL;
    }
    tensor->data[i] = buffer;
  }
  fclose(fp);

  info("load %s successfully", filePath);

  return tensor;
}

static double **get_axis_data(AxisDataIndex axisIndex) {
  double **axis;
  switch (axisIndex) {
  case ALPHA1:
    axis = (double **)malloc(sizeof(double *) * 1);
    axis[0] = axisData[0];
    break;
  case ALPHA2:
    axis = (double **)malloc(sizeof(double *) * 1);
    axis[0] = axisData[1];
    break;
  case DH1:
    axis = (double **)malloc(sizeof(double *) * 1);
    axis[0] = axisData[3];
    break;
  case ALPHA1_BETA1:
    axis = (double **)malloc(sizeof(double *) * 2);
    axis[0] = axisData[0];
    axis[1] = axisData[2];
    break;
  case ALPHA2_BETA1:
    axis = (double **)malloc(sizeof(double *) * 2);
    axis[0] = axisData[1];
    axis[1] = axisData[2];
    break;
  case ALPHA1_BETA1_DH1:
    axis = (double **)malloc(sizeof(double *) * 3);
    axis[0] = axisData[0];
    axis[1] = axisData[2];
    axis[2] = axisData[3];
    break;
  case ALPHA1_BETA1_DH2:
    axis = (double **)malloc(sizeof(double *) * 3);
    axis[0] = axisData[0];
    axis[1] = axisData[2];
    axis[2] = axisData[4];
    break;
  default:
    axis = NULL;
    break;
  }
  return axis;
}

int init_hifi_data() {
  hifiData = (Tensor **)malloc(sizeof(Tensor *) * DATA_LEN);
  hifiData[0] =
      load_aerodynamic_data("CL0120_ALPHA1_BETA1_DH2_601.dat", 3, 0b001);
  hifiData[1] = load_aerodynamic_data("CL0620_ALPHA1_BETA1_604.dat", 2, 0b000);
  hifiData[2] = load_aerodynamic_data("CL0720_ALPHA1_BETA1_603.dat", 2, 0b000);
  hifiData[3] = load_aerodynamic_data("CL0820_ALPHA2_BETA1_602.dat", 2, 0b100);
  hifiData[4] = load_aerodynamic_data("CL0920_ALPHA2_BETA1_605.dat", 2, 0b100);
  hifiData[5] = load_aerodynamic_data("CL1220_ALPHA1_608.dat", 1, 0b000);
  hifiData[6] = load_aerodynamic_data("CL1320_ALPHA1_606.dat", 1, 0b000);
  hifiData[7] = load_aerodynamic_data("CL1520_ALPHA2_609.dat", 1, 0b100);
  hifiData[8] = load_aerodynamic_data("CL1620_ALPHA2_607.dat", 1, 0b100);
  hifiData[9] = load_aerodynamic_data("CL9999_ALPHA1_brett.dat", 1, 0b000);
  hifiData[10] =
      load_aerodynamic_data("CM0120_ALPHA1_BETA1_DH1_101.dat", 3, 0b000);
  hifiData[11] = load_aerodynamic_data("CM0820_ALPHA2_BETA1_102.dat", 2, 0b100);
  hifiData[12] = load_aerodynamic_data("CM1020_ALPHA1_103.dat", 1, 0b000);
  hifiData[13] = load_aerodynamic_data("CM1120_ALPHA1_104.dat", 1, 0b000);
  hifiData[14] = load_aerodynamic_data("CM1420_ALPHA2_105.dat", 1, 0b100);
  hifiData[15] = load_aerodynamic_data("CM9999_ALPHA1_brett.dat", 1, 0b000);
  hifiData[16] =
      load_aerodynamic_data("CN0120_ALPHA1_BETA1_DH2_501.dat", 3, 0b001);
  hifiData[17] = load_aerodynamic_data("CN0620_ALPHA1_BETA1_504.dat", 2, 0b000);
  hifiData[18] = load_aerodynamic_data("CN0720_ALPHA1_BETA1_503.dat", 2, 0b000);
  hifiData[19] = load_aerodynamic_data("CN0820_ALPHA2_BETA1_502.dat", 2, 0b100);
  hifiData[20] = load_aerodynamic_data("CN0920_ALPHA2_BETA1_505.dat", 2, 0b100);
  hifiData[21] = load_aerodynamic_data("CN1220_ALPHA1_508.dat", 1, 0b000);
  hifiData[22] = load_aerodynamic_data("CN1320_ALPHA1_506.dat", 1, 0b000);
  hifiData[23] = load_aerodynamic_data("CN1520_ALPHA2_509.dat", 1, 0b100);
  hifiData[24] = load_aerodynamic_data("CN1620_ALPHA2_507.dat", 1, 0b100);
  hifiData[25] = load_aerodynamic_data("CN9999_ALPHA1_brett.dat", 1, 0b000);
  hifiData[26] =
      load_aerodynamic_data("CX0120_ALPHA1_BETA1_DH1_201.dat", 3, 0b000);
  hifiData[27] = load_aerodynamic_data("CX0820_ALPHA2_BETA1_202.dat", 2, 0b100);
  hifiData[28] = load_aerodynamic_data("CX1120_ALPHA1_204.dat", 1, 0b000);
  hifiData[29] = load_aerodynamic_data("CX1420_ALPHA2_205.dat", 1, 0b100);
  hifiData[30] = load_aerodynamic_data("CY0320_ALPHA1_BETA1_401.dat", 2, 0b000);
  hifiData[31] = load_aerodynamic_data("CY0620_ALPHA1_BETA1_403.dat", 2, 0b000);
  hifiData[32] = load_aerodynamic_data("CY0720_ALPHA1_BETA1_405.dat", 2, 0b000);
  hifiData[33] = load_aerodynamic_data("CY0820_ALPHA2_BETA1_402.dat", 2, 0b100);
  hifiData[34] = load_aerodynamic_data("CY0920_ALPHA2_BETA1_404.dat", 2, 0b100);
  hifiData[35] = load_aerodynamic_data("CY1220_ALPHA1_408.dat", 1, 0b000);
  hifiData[36] = load_aerodynamic_data("CY1320_ALPHA1_406.dat", 1, 0b000);
  hifiData[37] = load_aerodynamic_data("CY1520_ALPHA2_409.dat", 1, 0b100);
  hifiData[38] = load_aerodynamic_data("CY1620_ALPHA2_407.dat", 1, 0b100);
  hifiData[39] =
      load_aerodynamic_data("CZ0120_ALPHA1_BETA1_DH1_301.dat", 3, 0b000);
  hifiData[40] = load_aerodynamic_data("CZ0820_ALPHA2_BETA1_302.dat", 2, 0b100);
  hifiData[41] = load_aerodynamic_data("CZ1120_ALPHA1_304.dat", 1, 0b000);
  hifiData[42] = load_aerodynamic_data("CZ1420_ALPHA2_305.dat", 1, 0b100);
  hifiData[43] = load_aerodynamic_data("ETA_DH1_brett.dat", 1, 0b1000);

  for (int i = 0; i < DATA_LEN; i++) {
    if (hifiData[i] == NULL) {
      free(hifiData);
      return -1;
    }
  }

  return 0;
}

void free_hifi_data() {
  for (int i = 0; i < DATA_LEN; i++) {
    free_tensor(hifiData[i]);
  }
  free(hifiData);
}

int init_axis_data() {
  char errorMsg[100];

  axisData = (double **)malloc(sizeof(double *) * 5);
  axisData[0] = load_axis_data("ALPHA1.dat", 20);
  axisData[1] = load_axis_data("ALPHA2.dat", 14);
  axisData[2] = load_axis_data("BETA1.dat", 19);
  axisData[3] = load_axis_data("DH1.dat", 5);
  axisData[4] = load_axis_data("DH2.dat", 3);

  return 0;
}

void free_axis_data() {
  for (int i = 0; i < 5; i++) {
    free(axisData[i]);
  }
  free(axisData);
}

void set_data_dir(char *dir) { dataDir = dir; }

#pragma region
static double _Cx(double alpha, double beta, double dele) {
  GET_COEFF3(ALPHA1_BETA1_DH1, CX0120);
}

static double _Cz(double alpha, double beta, double dele) {
  GET_COEFF3(ALPHA1_BETA1_DH1, CZ0120);
}

static double _Cm(double alpha, double beta, double dele) {
  GET_COEFF3(ALPHA1_BETA1_DH1, CM0120);
}

static double _Cy(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CY0320);
}

static double _Cn(double alpha, double beta, double dele) {
  GET_COEFF3(ALPHA1_BETA1_DH2, CN0120);
}

static double _Cl(double alpha, double beta, double dele) {
  GET_COEFF3(ALPHA1_BETA1_DH2, CL0120);
}

static double _Cx_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CX0820);
}

static double _Cz_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CZ0820);
}

static double _Cm_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CM0820);
}

static double _Cy_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CY0820);
}

static double _Cn_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CN0820);
}

static double _Cl_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CL0820);
}

static double _CXq(double alpha) { GET_COEFF_ALPHA(ALPHA1, CX1120); }

static double _CZq(double alpha) { GET_COEFF_ALPHA(ALPHA1, CZ1120); }

static double _CMq(double alpha) { GET_COEFF_ALPHA(ALPHA1, CM1120); }

static double _CYp(double alpha) { GET_COEFF_ALPHA(ALPHA1, CY1220); }

static double _CYr(double alpha) { GET_COEFF_ALPHA(ALPHA1, CY1320); }

static double _CNr(double alpha) { GET_COEFF_ALPHA(ALPHA1, CN1320); }

static double _CNp(double alpha) { GET_COEFF_ALPHA(ALPHA1, CN1220); }

static double _CLp(double alpha) { GET_COEFF_ALPHA(ALPHA1, CL1220); }

static double _CLr(double alpha) { GET_COEFF_ALPHA(ALPHA1, CL1320); }

static double _delta_CXq_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CX1420); }

static double _delta_CYr_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CY1620); }

static double _delta_CYp_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CY1520); }

static double _delta_CZq_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CZ1420); }

static double _delta_CLr_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CL1620); }

static double _delta_CLp_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CL1520); }

static double _delta_CMq_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CM1420); }

static double _delta_CNr_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CN1620); }

static double _delta_CNp_lef(double alpha) { GET_COEFF_ALPHA(ALPHA2, CN1520); }

static double _Cy_r30(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CY0720);
}

static double _Cn_r30(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CN0720);
}

static double _Cl_r30(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CL0720);
}

static double _Cy_a20(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CY0620);
}

static double _Cy_a20_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CY0920);
}

static double _Cn_a20(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CN0620);
}

static double _Cn_a20_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CN0920);
}

static double _Cl_a20(double alpha, double beta) {
  GET_COEFF2(ALPHA1_BETA1, CL0620);
}

static double _Cl_a20_lef(double alpha, double beta) {
  GET_COEFF2(ALPHA2_BETA1, CL0920);
}

static double _delta_CNbeta(double alpha) { GET_COEFF_ALPHA(ALPHA1, CN9999); }

static double _delta_CLbeta(double alpha) { GET_COEFF_ALPHA(ALPHA1, CL9999); }

static double _delta_Cm(double alpha) { GET_COEFF_ALPHA(ALPHA1, CM9999); }

static double _eta_el(double el) {
  double targetData[1] = {el};
  double **axis = get_axis_data(DH1);
  double r = interpn(axis, hifiData[43], targetData);
  free(axis);
  return r;
}
#pragma endregion

int hifi_C(double alpha, double beta, double el, double *retVal) {
  retVal[0] = _Cx(alpha, beta, el);
  retVal[1] = _Cz(alpha, beta, el);
  retVal[2] = _Cm(alpha, beta, el);
  retVal[3] = _Cy(alpha, beta);
  retVal[4] = _Cn(alpha, beta, el);
  retVal[5] = _Cl(alpha, beta, el);

  CHECK_NAN(6);
}

int hifi_damping(double alpha, double *retVal) {
  retVal[0] = _CXq(alpha);
  retVal[1] = _CYr(alpha);
  retVal[2] = _CYp(alpha);
  retVal[3] = _CZq(alpha);
  retVal[4] = _CLr(alpha);
  retVal[5] = _CLp(alpha);
  retVal[6] = _CMq(alpha);
  retVal[7] = _CNr(alpha);
  retVal[8] = _CNp(alpha);

  CHECK_NAN(9);
}

int hifi_C_lef(double alpha, double beta, double *retVal) {
  retVal[0] = _Cx_lef(alpha, beta) - _Cx(alpha, beta, 0);
  retVal[1] = _Cz_lef(alpha, beta) - _Cz(alpha, beta, 0);
  retVal[2] = _Cm_lef(alpha, beta) - _Cm(alpha, beta, 0);
  retVal[3] = _Cy_lef(alpha, beta) - _Cy(alpha, beta);
  retVal[4] = _Cn_lef(alpha, beta) - _Cn(alpha, beta, 0);
  retVal[5] = _Cl_lef(alpha, beta) - _Cl(alpha, beta, 0);

  CHECK_NAN(6);
}

int hifi_damping_lef(double alpha, double *retVal) {
  retVal[0] = _delta_CXq_lef(alpha);
  retVal[1] = _delta_CYr_lef(alpha);
  retVal[2] = _delta_CYp_lef(alpha);
  retVal[3] = _delta_CZq_lef(alpha);
  retVal[4] = _delta_CLr_lef(alpha);
  retVal[5] = _delta_CLp_lef(alpha);
  retVal[6] = _delta_CMq_lef(alpha);
  retVal[7] = _delta_CNr_lef(alpha);
  retVal[8] = _delta_CNp_lef(alpha);

  CHECK_NAN(9);
}

int hifi_rudder(double alpha, double beta, double *retVal) {
  retVal[0] = _Cy_r30(alpha, beta) - _Cy(alpha, beta);
  retVal[1] = _Cn_r30(alpha, beta) - _Cn(alpha, beta, 0);
  retVal[2] = _Cl_r30(alpha, beta) - _Cl(alpha, beta, 0);

  CHECK_NAN(3);
}

int hifi_ailerons(double alpha, double beta, double *retVal) {
  retVal[0] = _Cy_a20(alpha, beta) - _Cy(alpha, beta);
  retVal[1] = _Cy_a20_lef(alpha, beta) - _Cy_lef(alpha, beta) - retVal[0];
  retVal[2] = _Cn_a20(alpha, beta) - _Cn(alpha, beta, 0);
  retVal[3] = _Cn_a20_lef(alpha, beta) - _Cn_lef(alpha, beta) - retVal[2];
  retVal[4] = _Cl_a20(alpha, beta) - _Cl(alpha, beta, 0);
  retVal[5] = _Cl_a20_lef(alpha, beta) - _Cl_lef(alpha, beta) - retVal[4];

  CHECK_NAN(6);
}

int hifi_other_coeffs(double alpha, double el, double *retVal) {
  retVal[0] = _delta_CNbeta(alpha);
  retVal[1] = _delta_CLbeta(alpha);
  retVal[2] = _delta_Cm(alpha);
  retVal[3] = _eta_el(el);
  retVal[4] = 0; /* ignore deep-stall regime, delta_Cm_ds = 0 */

  CHECK_NAN(5);
}
