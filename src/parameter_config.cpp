#include "parameter_config.h"

template void parameterConfig::get(const char* key, float& value);
template void parameterConfig::get(const char* key, int& value);
template void parameterConfig::get(const char* key, bool& value);
template void parameterConfig::get(const char* key, double& value);
template void parameterConfig::get(const char* key, std::string& value);
