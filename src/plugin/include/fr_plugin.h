#ifndef FR_PLUGIN
#define FR_PLUGIN

/// @brief the level of the message
typedef enum
{
    TRACE,
    DEBUG,
    INFO,
    WARN,
    ERROR,
} LogLevel;

/// @brief the type of logger callback function
typedef void (*Logger)(const char *msg, LogLevel level);

/// @brief the instance of logger callback function
extern Logger frplugin_log;

/// @brief register logger callback function,
///        system will call this function before loading plugins or installing models
/// @param lg the logger instance transfer from system
void frplugin_register_logger(Logger lg);

/// @brief Hook: when plugin is installing
/// @param argc the length of input args
/// @param argv the data of args
/// @return <0 represent occur some err
int frplugin_install_hook(int argc, char **argv);

/// @brief Hook: when plugin is uninstalled
/// @param argc the length of input args
/// @param argv the data of args
/// @return <0 represent occur some err
int frplugin_uninstall_hook(int argc, char **argv);

typedef struct
{
    double mach;
    double qbar;
    double ps;
} Atmos;

typedef Atmos (*AtmosFunc)(double altitude, double velocity);

extern AtmosFunc frplugin_atmos;

void frplugin_register_atmos(AtmosFunc atmos);

typedef void *(*IntegratorNew)(double init);

extern IntegratorNew frplugin_integrator_new;

void frplugin_register_integrator_new(IntegratorNew integrator_new);

typedef void (*IntegratorDrop)(void *integrator);

extern IntegratorDrop frplugin_integrator_drop;

void frplugin_register_integrator_drop(IntegratorDrop integrator_drop);

typedef int (*IntegratorUpdate)(void *integrator, double value, double t, double *result);

extern IntegratorUpdate frplugin_integrator_update;

void frplugin_register_integrator_update(IntegratorUpdate integrator_update);

typedef int (*IntegratorPast)(void *integrator, double *result);

extern IntegratorPast frplugin_integrator_past;

void frplugin_register_integrator_past(IntegratorPast integrator_past);

typedef int (*IntegratorReset)(void *integrator);

extern IntegratorReset frplugin_integrator_reset;

void frplugin_register_integrator_reset(IntegratorReset integrator_reset);

typedef void *(*ActuatorNew)(
    double init,
    double command_saturation_top,
    double command_saturation_bottom,
    double rate_saturation,
    double gain);

extern ActuatorNew frplugin_actuator_new;

void frplugin_register_actuator_new(ActuatorNew actuator_new);

typedef void (*ActuatorDrop)(
    void *actuator);

extern ActuatorDrop frplugin_actuator_drop;

void frplugin_register_actuator_drop(ActuatorDrop actuator_drop);

typedef int (*ActuatorUpdate)(void *actuator, double value, double t, double *result);

extern ActuatorUpdate frplugin_actuator_update;

void frplugin_register_actuator_update(ActuatorUpdate actuator_update);

typedef int (*ActuatorPast)(void *actuator, double *result);

extern ActuatorPast frplugin_actuator_past;

void frplugin_register_actuator_past(ActuatorPast actuator_past);

typedef int (*ActuatorReset)(void *actuator);

extern ActuatorReset frplugin_actuator_reset;

void frplugin_register_actuator_reset(ActuatorReset actuator_reset);

#endif // FR_PLUGIN