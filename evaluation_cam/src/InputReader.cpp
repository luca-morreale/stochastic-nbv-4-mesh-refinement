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

    StringPoseMapping InputReader::readMappingDatabase(std::string &mappingFile)
    {
        PoseList poses;
        StringList fileList;
        std::ifstream cin(mappingFile);

        while (!cin.eof()) {
            std::string filename, poseString;
            cin >> filename >> poseString;

            filename = trim(filename);
            if (filename.size() == 0){
                continue;
            }

            AnglePose pose = parseEntry(poseString);            

            ProjectionMatrix view = glm::lookAt(pose.first, pose.second, GLMVec3(0, 1, 0));
            RotationMatrix rot(view);
            
            fileList.push_back(getPoseString(filename));
            poses.push_back(std::make_pair(pose.first, rot));
        }

        OpenMvgPoseConverter::convertPoses(poses);

        cin.close();
        return std::make_pair(fileList, poses);
    }

} // namespace cameval
