// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/types.hpp"

namespace gzb {

bool knownRosType(const std::string& ros) {
  for (const TypePair& p : messageTypePairs()) {
    if (p.ros == ros) return true;
  }
  return false;
}

bool knownGzType(const std::string& gz) {
  for (const TypePair& p : messageTypePairs()) {
    if (p.gz == gz) return true;
  }
  return false;
}

bool isKnownPair(const std::string& ros, const std::string& gz) {
  for (const TypePair& p : messageTypePairs()) {
    if (p.ros == ros && p.gz == gz) return true;
  }
  return false;
}

std::vector<std::string> gzTypesFor(const std::string& ros) {
  std::vector<std::string> out;
  for (const TypePair& p : messageTypePairs()) {
    if (p.ros == ros) out.push_back(p.gz);
  }
  return out;
}

std::vector<std::string> rosTypesFor(const std::string& gz) {
  std::vector<std::string> out;
  for (const TypePair& p : messageTypePairs()) {
    if (p.gz == gz) out.push_back(p.ros);
  }
  return out;
}

}  // namespace gzb
