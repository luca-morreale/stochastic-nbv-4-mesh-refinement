#ifndef CAM_POSITION_GENERATOR_ORIENTATION_UTILITY_CHECK_H
#define CAM_POSITION_GENERATOR_ORIENTATION_UTILITY_CHECK_H

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>

#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {

    bool isMeaningfulPose(EigVector5 &pose, GLMVec3 &centroid, TreePtr tree, CameraGeneralConfiguration &camConfig);
    bool isOppositeView(EigVector5 &pose, GLMVec3 &centroid);
    bool isIntersecting(EigVector5 &pose, GLMVec3 &centroid, TreePtr tree);
    bool isMathemathicalError(Segment_intersection &intersection, Point &point);
    bool isPointInsideImage(EigVector5 &pose, GLMVec3 &centroid, CameraGeneralConfiguration &camConfig);

    RotationMatrix getRotationMatrix(float roll, float pitch, float yaw);
    CameraMatrix getExtrinsicMatrix(EigVector5 &pose);
    CameraMatrix getIntrisincMatrix(CameraGeneralConfiguration &camConfig);
    CameraMatrix getCameraMatrix(EigVector5 &pose, CameraGeneralConfiguration &camConfig);
    GLMVec2 getProjectedPoint(EigVector5 &pose, GLMVec3 &centroid, CameraGeneralConfiguration &camConfig);

    extern const GLMVec4 zdir;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_ORIENTATION_UTILITY_CHECK_H
