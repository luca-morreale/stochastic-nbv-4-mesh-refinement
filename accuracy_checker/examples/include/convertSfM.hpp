#ifndef CONVERT_SFM_DATA
#define CONVERT_SFM_DATA

#include <meshac/alias_definition.hpp>
#include <meshac/SfMData.h>
#include <meshac/type_definition.hpp>

#include <manifoldReconstructor/SfMData.h>
#include <manifoldReconstructor/types_reconstructor.hpp>

SfMData convertSfMData(meshac::SfMData &src);
std::vector<CameraType> convertList(meshac::CameraList &cameras);
CameraType convertCameraType(meshac::CameraType &src);

#endif // CONVERT_SFM_DATA
