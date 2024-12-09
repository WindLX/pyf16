#ifndef FR_PLUGIN
#define FR_PLUGIN

/// @brief the level of the message
typedef enum {
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
///        system will call this function before loading plugins or installing
///        models
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

#endif // FR_PLUGIN