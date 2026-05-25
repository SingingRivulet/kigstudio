#include "kigstudio/sdf/sdf.h"

namespace sinriv::kigstudio::sdf {

std::shared_ptr<SDF_bool> sdf_union(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Union,
                                       std::move(a), std::move(b));
}

std::shared_ptr<SDF_bool> sdf_intersection(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Intersection,
                                       std::move(a), std::move(b));
}

std::shared_ptr<SDF_bool> sdf_subtraction(
    std::shared_ptr<SDFBase> a, std::shared_ptr<SDFBase> b) {
    return std::make_shared<SDF_bool>(SDFBoolOp::Subtraction,
                                       std::move(a), std::move(b));
}

}  // namespace sinriv::kigstudio::sdf
