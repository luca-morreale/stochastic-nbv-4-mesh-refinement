#ifndef MESH_ACCURACY_UNPROJECTABLE_POINT_THROUGH_CAM_EXCEPTION_H
#define MESH_ACCURACY_UNPROJECTABLE_POINT_THROUGH_CAM_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UnprojectablePointThroughCamException : public std::runtime_error {
    public:
        UnprojectablePointThroughCamException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UnprojectablePointThroughCamException() : std::runtime_error("The given points can not be projected using the given camera.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_UNPROJECTABLE_POINT_THROUGH_CAM_EXCEPTION_H




