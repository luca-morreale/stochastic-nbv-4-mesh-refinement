#include <OpenMvgPoseConverter.hpp>

namespace cameval {

    const GLMVec3 OpenMvgPoseConverter::twoPi = GLMVec3((float)(2.0f * M_PI));

    // OpenMvgPoseConverter::OpenMvgPoseConverter(GLMVec3 &lookingWith, GLMVec3 &targetLookingWith)
    // {
    //     OpenMvgPoseConverter::lookingWith = lookingWith;
    //     OpenMvgPoseConverter::targetLookingWith = targetLookingWith;
    // }

    // OpenMvgPoseConverter::~OpenMvgPoseConverter()
    // { /*    */ }

    /*
     * NOTE
     * can do this by inferring from direction vectors? like using acos of the 
     * target and something?
     * reversing with rotation matrix from looking with, then using target!
     * final rotation matrix is the product of all rotation matrixs
     */

    void OpenMvgPoseConverter::convertPoses(PoseList &poses)
    {
        for (int i = 0; i < poses.size(); i++) {
            convertPose(poses[i]);
        }
    }
    
    void OpenMvgPoseConverter::convertPose(Pose &pose)
    {
        // 180° around Y because vertical axis is y and not z
        pose.second = rotationPitch(M_PI) * pose.second;
    }

    void OpenMvgPoseConverter::convertAnglePoses(AnglePoseList &poses)
    {
        for (int i = 0; i < poses.size(); i++) {
            convertAnglePose(poses[i]);
        }
    }

    void OpenMvgPoseConverter::convertAnglePose(AnglePose &pose)
    {
        pose.second.y = pose.second.y + getAngleDisplacement(pose.second);
    }

    float OpenMvgPoseConverter::getAngleDisplacement(GLMVec3 &angles)
    {

        if (glm::all(glm::lessThan(angles, twoPi))) {
            return M_PI;
        } else {
            return 180.0f;
        }
    }


} // namespace cameval
