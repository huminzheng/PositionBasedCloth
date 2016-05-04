#include "BasicTypes.h"

#include <limits>
#include <cmath>

float const POSITIVE_MAX_FLOAT = (std::numeric_limits<float>::max)();
float const NEGATIVE_MAX_FLOAT = -POSITIVE_MAX_FLOAT;
float const DISTANCE_OVERLAP_THRESHOLD = 1e-20f;
float const DISTANCE_OVERLAP_SQUARED_THRESHOLD = 1e-30f;
float const MATH_PI = (std::atan)(1) * 4;

