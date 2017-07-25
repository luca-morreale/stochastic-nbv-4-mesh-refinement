#include <Mapper.hpp>

namespace cameval {

    Mapper::Mapper()
    {
        std::string mappingFile = "output_mapping.txt";
        StringPoseMapping mapping = InputReader::readMappingDatabase(mappingFile);
        
        this->poses = mapping.second;
        for (int i = 0; i < mapping.first.size(); i++) {
            this->strings.insert(std::make_pair(mapping.first[i], i));
        }
    }

    Mapper::~Mapper()
    {
        this->poses.clear();
        this->strings.clear();
    }

    Pose Mapper::mapFileToPose(std::string entry)
    {
        entry = entry.substr(entry.find_last_of("/"), entry.size());
        entry = entry.substr(0, entry.find_last_of("."));

        auto position = strings.find(entry);
        if (position == strings.end()) {
            // FIXME error
            return Pose();
        }
        int index = position->second;
        return poses[index];
    }

    Pose Mapper::slowMappingToPose(std::string entry)
    {
        entry = entry.substr(entry.find_last_of("/") + 1, entry.size());
        entry = entry.substr(0, entry.find_last_of("."));
        
        std::ifstream cin("output_mapping.txt");

        while (!cin.eof()) {
            std::string filename, poseString;
            cin >> filename >> poseString;

            filename = trim(filename);
            // std::cout << filename << std::endl;
            if (filename.compare(entry) == 0) {

                poseString = "prefix_" + poseString;
                poseString += ".suffix";
                AnglePose position_lookAt = parseEntry(poseString);

                ProjectionMatrix view = glm::lookAt(position_lookAt.first, position_lookAt.second, GLMVec3(0, 1, 0));
                RotationMatrix rot(view);
                
                Pose pose = std::make_pair(position_lookAt.first, rot);
                OpenMvgPoseConverter::convertPose(pose);
                cin.close();
                return pose;
            }
        }
        cin.close();

        return Pose();
    }


} // namespace cameval
