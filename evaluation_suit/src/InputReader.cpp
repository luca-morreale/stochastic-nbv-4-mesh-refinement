#include <InputReader.hpp>

namespace cameval {

    StringList InputReader::readDatabase(std::string &database)     // file should contain an entry for each line with no space
    {
        std::ifstream cin(database);
        StringList filelist;

        while (!cin.eof()) {
            std::string file;
            cin >> file;
            
            file = trim(file);
            if (file.size() > 0){
                filelist.push_back(file);
            }
        }
        cin.close();
        return filelist;
    }

    StringPoseMap InputReader::readMappingDatabase(std::string &mappingFile)
    {
        StringPoseMap mapping;
        std::ifstream cin(mappingFile);

        while (!cin.eof()) {
            std::string filename, poseString;
            cin >> filename >> poseString;

            filename = trim(filename);
            if (filename.size() == 0){
                continue;
            }

            AnglePose anglePose = parseEntry(poseString);            

            ProjectionMatrix view = glm::lookAt(anglePose.first, anglePose.second, GLMVec3(0, 1, 0));
            RotationMatrix rot(view);
            
            Pose pose = std::make_pair(anglePose.first, rot);
            OpenMvgPoseConverter::convertPose(pose);

            mapping.insert(std::make_pair(filename, pose));
        }

        cin.close();
        return mapping;
    }

} // namespace cameval
