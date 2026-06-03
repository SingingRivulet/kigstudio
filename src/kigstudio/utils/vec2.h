#pragma once
#include <cJSON.h>
#include <math.h>
#include <concepts>
#include <iostream>
#include "kigstudio/utils/vec3.h"

namespace sinriv::kigstudio {

template <Numeric T>
struct vec2 {
    T x, y;
};

}  // namespace sinriv::kigstudio