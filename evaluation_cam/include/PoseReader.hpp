#ifndef EVALUATION_CAMERA_POSITION_POSE_READER_H_
#define EVALUATION_CAMERA_POSITION_POSE_READER_H_

#include <fstream>

#include <glm/gtc/matrix_transform.hpp>

#include <aliases.h>
#include <utilities.hpp>

namespace cameval {
    
    class PoseReader {
    public:
        PoseReader(std::string &posesFilename, bool radians=false);
        ~PoseReader();

        void setFilename(std::string &posesFilename);

        PoseList getPoses();
        AnglePoseList getAnglePoses();


    private:
        GLMVec3 computeAngles(GLMMat3 &rot);

        std::string filename;
        bool radians;

        GLMVec3List positions;
        GLMVec3List angles;
        GLMMat3List rotations;

        void parse();

        typedef GLMMat4 ProjectionMatrix;
        typedef GLMMat3 RotationMatrix;

    };
} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_POSE_READER_H_
