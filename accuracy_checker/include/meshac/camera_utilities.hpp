#ifndef MESH_ACCURACY_CAMERA_UTILITIES_H
#define MESH_ACCURACY_CAMERA_UTILITIES_H

#include <CGAL/exceptions.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>

#include <meshac/alias_definition.hpp>
#include <meshac/UnprojectablePointThroughCamException.hpp>

namespace meshac {

    ListMappingGLMVec2 projectMeshPointsThroughCameras(GLMVec3List &points, CameraList &acams, TreePtr tree);
    
    GLMVec2 projectThrough(GLMVec3 &meshPoint, CameraType &P, TreePtr tree);
    
    bool isMeaningfulPose(GLMVec3 &meshPoint, CameraType &P, TreePtr tree);
    bool isOppositeView(GLMVec3 &meshPoint, CameraType &P);
    bool isIntersecting(GLMVec3 &meshPoint, CameraType &P, TreePtr tree);
    bool isMathemathicalError(Segment_intersection &intersection, Point &point);
    bool isPointInsideImage(GLMVec2 &point2D, CameraType &P);

    GLMVec2 getProjectedPoint(GLMVec3 &meshPoint, CameraMatrix &P);
    int getImageWidth(CameraType &P);
    int getImageHeight(CameraType &P);
    GLMVec3 getCameraCenter(CameraType &P);
    CameraMatrix getExtrinsicMatrix(CameraType &P);
    RotationMatrix getRotationMatrix(CameraType &P);
    CameraMatrix getCameraMatrix(CameraType &P);

    extern const GLMVec4 zdir;

}

#endif // MESH_ACCURACY_CAMERA_UTILITIES_H
