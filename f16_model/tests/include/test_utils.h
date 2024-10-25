#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <stdarg.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "fr_model.h"
#include "fr_plugin.h"

#define DOUBLE_EQ(a, b) assert(fabs((a) - (b)) < 0.000001)

const char *frplugin_info_level_to_string(LogLevel level);

void test_frplugin_log(const char *msg, LogLevel level);

int frsys_init();

int frsys_step();

int frsys_stop();

int frmodel_install();

int frmodel_uninstall();

typedef int (*FrsysStep)();

int fr_main(FrsysStep frsys_step);

#endif // TEST_UTILS_H