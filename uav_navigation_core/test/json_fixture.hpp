#pragma once

#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

inline std::string ReadFixture(const std::string &name) {
  std::ifstream stream(std::string(UAV_NAVIGATION_FIXTURE_DIR) + "/" + name);
  if (!stream)
    throw std::runtime_error("cannot open fixture " + name);
  return {std::istreambuf_iterator<char>(stream),
          std::istreambuf_iterator<char>()};
}
inline std::size_t ValueAt(const std::string &json, const std::string &key) {
  auto position = json.find("\"" + key + "\"");
  if (position == std::string::npos)
    throw std::runtime_error("missing key " + key);
  position = json.find(':', position);
  return json.find_first_not_of(" \t\r\n", position + 1);
}
inline double Number(const std::string &json, const std::string &key) {
  const char *begin = json.c_str() + ValueAt(json, key);
  char *end = nullptr;
  const double result = std::strtod(begin, &end);
  if (end == begin)
    throw std::runtime_error("invalid number " + key);
  return result;
}
inline bool Boolean(const std::string &json, const std::string &key) {
  const auto p = ValueAt(json, key);
  if (json.compare(p, 4, "true") == 0)
    return true;
  if (json.compare(p, 5, "false") == 0)
    return false;
  throw std::runtime_error("invalid bool " + key);
}
inline std::string String(const std::string &json, const std::string &key) {
  auto p = ValueAt(json, key);
  if (json[p++] != '"')
    throw std::runtime_error("invalid string " + key);
  return json.substr(p, json.find('"', p) - p);
}
inline std::vector<double> Numbers(const std::string &json,
                                   const std::string &key) {
  auto p = ValueAt(json, key);
  if (json[p] != '[')
    throw std::runtime_error("invalid array " + key);
  const auto end_array = json.find(']', p);
  std::vector<double> values;
  ++p;
  while (p < end_array) {
    p = json.find_first_not_of(" ,\t\r\n", p);
    if (p >= end_array)
      break;
    char *end = nullptr;
    values.push_back(std::strtod(json.c_str() + p, &end));
    if (end == json.c_str() + p)
      throw std::runtime_error("invalid array item " + key);
    p = static_cast<std::size_t>(end - json.c_str());
  }
  return values;
}
