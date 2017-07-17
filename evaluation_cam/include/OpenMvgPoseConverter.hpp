#ifndef EVALUATION_CAMERA_POSITION_OPENMVG_POSE_CONVERTER_H_
#define EVALUATION_CAMERA_POSITION_OPENMVG_POSE_CONVERTER_H_

#include <fstream>

#include <glm/vector_relational.hpp>

#include <aliases.h>
#include <utilities.hpp>

namespace cameval {
    
    class OpenMvgPoseConverter {
    public:
        // OpenMvgPoseConverter();
        // ~OpenMvgPoseConverter();

        static void convertPoses(PoseList &poses);
        static void convertAnglePoses(AnglePoseList &poses);

    private:

        static float getAngleDisplacement(GLMVec3 &angles);

        static const GLMVec3 twoPi;

        typedef GLMMat3 RotationMatrix;
        typedef GLMMat4 ProjectionMatrix;
    //     static GLMVec3 lookingWith(0.0f, 0.0f, -1.0f);
    //     static GLMVec3 targetLookingWith(0.0f, 0.0f, 1.0f);
        

    };
} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_OPENMVG_POSE_CONVERTER_H_
