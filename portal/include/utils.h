#ifndef __PORTAL__UTILS__
#define __PORTAL__UTILS__

#include <map>
#include <string>
#include <vector>

std::vector<std::pair<std::string, std::map<std::string, std::string>>> readConfig(const char *path);

#endif