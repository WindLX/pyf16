#include "utils.h"
#include "fr_plugin.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int fix(double in) {
  int out;

  if (in >= 0.0) {
    out = (int)floor(in);
  } else if (in < 0.0) {
    out = (int)ceil(in);
  }

  return out;
}

int sign(double in) {
  int out;

  if (in > 0.0) {
    out = 1;
  } else if (in < 0.0) {
    out = -1;
  } else if (in == 0.0) {
    out = 0;
  }
  return out;
}

double clamp(double x, double min, double max) {
  return (x < min) ? min : (x > max) ? max : x;
}

double r2d() { return 180.0 / acos(-1); }
double d2r() { return 180.0 / acos(-1); }

Tensor *create_tensor(int n_dimension, int *n_points) {
  int length = 1;
  TensorInfo *info = (TensorInfo *)malloc(sizeof(TensorInfo));
  info->n_dimension = n_dimension;
  info->n_points = (int *)malloc(n_dimension * sizeof(int));
  memcpy(info->n_points, n_points, n_dimension * sizeof(int));
  Tensor *tensor = (Tensor *)malloc(sizeof(Tensor));
  tensor->info = info;
  for (int i = 0; i < info->n_dimension; i++) {
    length *= info->n_points[i];
  }
  tensor->data = (double *)malloc(length * sizeof(double));
  return (tensor);
}

void free_tensor(Tensor *tensor) {
  free(tensor->info->n_points);
  free(tensor->info);
  free(tensor->data);
  free(tensor);
}

int get_lin_index(int *indexVector, TensorInfo info) {
  int linIndex = 0;
  int i, j, P;
  for (i = 0; i < info.n_dimension; i++) {
    P = 1;
    for (j = 0; j < i; j++)
      P = P * info.n_points[j];
    linIndex += P * indexVector[i];
  }
  return (linIndex);
}

int *create_ivector(int n) {
  int *vec = (int *)malloc(n * sizeof(int));
  return (vec);
}

double *create_dvector(int n) {
  double *vec = (double *)malloc(n * sizeof(double));
  return (vec);
}

int **create_imatrix(int n, int m) {
  int i;
  int **mat = (int **)malloc(n * sizeof(int *));
  for (i = 0; i < n; i++)
    mat[i] = (int *)malloc(m * sizeof(int));
  return (mat);
}

double **create_dmatrix(int n, int m) {
  int i;
  double **mat = (double **)malloc(n * sizeof(double *));
  for (i = 0; i < n; i++)
    mat[i] = (double *)malloc(m * sizeof(double));
  return (mat);
}

void free_imatrix(int **mat, int n, int m) {
  /*
      the column size is not used but is required only
      for debugging purpose
  */
  int i;
  for (i = 0; i < n; i++)
    free(mat[i]);
  free(mat);
}

void free_dmatrix(double **mat, int n, int m) {
  /*
      the column size is not used but is required only
      for debugging purpose
  */
  int i;
  for (i = 0; i < n; i++)
    free(mat[i]);
  free(mat);
}

// log trace
void trace(const char *format, ...) {
  char msg[256];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  frplugin_log(msg, TRACE);
}

// log debug
void debug(const char *format, ...) {
  char msg[256];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  frplugin_log(msg, DEBUG);
}

// log info
void info(const char *format, ...) {
  char msg[256];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  frplugin_log(msg, INFO);
}

// log warn
void warn(const char *format, ...) {
  char msg[256];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  frplugin_log(msg, WARN);
}

// log error
void error_(const char *format, ...) {
  char msg[256];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  frplugin_log(msg, ERROR);
}