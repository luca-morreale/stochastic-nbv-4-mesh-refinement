#include <PoseReader.hpp>

namespace cameval {

    PoseReader::PoseReader(std::string &posesFilename, bool radians)
    {
        this->filename = posesFilename;
        this->radians = radians;
        parse();
    }

    PoseReader::~PoseReader()
    {
        positions.clear();
        rotations.clear();
        angles.clear();
    }

    void PoseReader::setFilename(std::string &posesFilename)
    {
        this->filename = posesFilename;
        parse();
    }

    PoseList PoseReader::getPoses()
    {
        PoseList poses;
        for (int i = 0; i < positions.size(); i++) {
            poses.push_back(std::make_pair(positions[i], rotations[i]));
        }
        OpenMvgPoseConverter::convertPoses(poses);
        return poses;
    }

    AnglePoseList PoseReader::getAnglePoses()
    {
        AnglePoseList poses;
        for (int i = 0; i < positions.size(); i++) {
            poses.push_back(std::make_pair(positions[i], angles[i]));
        }
        OpenMvgPoseConverter::convertAnglePoses(poses);
        return poses;
    }

    void PoseReader::parse()
    {
        std::ifstream cin(this->filename);
        
        while(!cin.eof()) {
            GLMVec3 position, lookAt;
            cin >> position.x >> position.y >> position.z >> lookAt.x >> lookAt.y >> lookAt.z;
            if (position.x == 0.0 && position.y == 0.0 && position.z == 0.0)    // posa vuota
                continue;

            ProjectionMatrix view = glm::lookAt(position, lookAt, GLMVec3(0, -1, 0));
            RotationMatrix rot(view);
            rot = glm::transpose(rot);
            GLMVec3 angle = computeAngles(rot);

            this->positions.push_back(position);
            this->rotations.push_back(rot);
            this->angles.push_back(angle);
        }


        cin.close();
    }

    GLMVec3 PoseReader::computeAngles(RotationMatrix &rot)
    {
        GLMVec3 angle;
        // NOTE remember to invert rows and cols
        angle.x = std::atan2(rot[1][0], rot[0][0]);    // roll
        angle.y = std::atan2(-rot[2][0], std::sqrt(std::pow(rot[2][1],2) + std::pow(rot[2][2], 2)));   // pitch
        angle.z = std::atan2(rot[2][1], rot[2][2]);    // yaw

        if (!this->radians) {
            angle.x = deg(angle.x);
            angle.y = deg(angle.y);
            angle.z = deg(angle.z);
        }
        return angle;
    }

} // namespace cameval
