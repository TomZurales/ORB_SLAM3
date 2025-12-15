#pragma once

#include "viewpoint.h"

#include <boost/serialization/access.hpp>
#include <boost/serialization/array.hpp>

typedef struct Observation_t {
  Viewpoint v;
  float s;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_array(v.data(), v.size());
    ar & s;
  }
} Observation;