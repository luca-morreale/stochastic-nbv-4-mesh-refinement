#include <Mapper.hpp>

namespace cameval {

    Mapper::Mapper(std::string mappingFile)
    {
        // std::string mappingFile = "output_mapping.txt";
        this->mappingFile = mappingFile;
        StringPoseMapping mapping = InputReader::readMappingDatabase(mappingFile);
        this->poses = mapping.second;

        setStringIndexMap(mapping.first);
        setFilenameMap();
    }

    Mapper::~Mapper()
    {
        this->poses.clear();
        this->strings.clear();
        this->filenameMap.clear();
    }

    void Mapper::setStringIndexMap(StringList &mapping)
    {
        for (int i = 0; i < mapping.size(); i++) {
            this->strings.insert(std::make_pair(mapping[i], i));
        }
    }

    void Mapper::setFilenameMap()
    {
        StringList files = keys(strings);
        for (std::string file : files) {
            std::string poseString = removePartsFromEntry(file);
            StringList coords = divideStringPose(poseString);

            filenameMap.insert(std::make_pair(concatBlocks(coords), file));
        }
    }

    std::string Mapper::mapStringToFile(std::string &entry)
    {
        std::string key = getPoseString(entry);

        return filenameMap[key];
    }

    Pose Mapper::mapFileToPose(std::string entry)
    {
        std::string key = getPoseString(entry);

        auto position = strings.find(key);
        if (position == strings.end()) {
            // FIXME error
            return Pose();
        }
        int index = position->second;
        return poses[index];
    }

    Pose Mapper::slowMappingToPose(std::string entry, std::string mappingFile)
    {
        std::string key = getPoseString(entry);
        
        std::ifstream cin(mappingFile);

        while (!cin.eof()) {
            std::string filename, poseString;
            cin >> filename >> poseString;

            filename = trim(filename);
            filename = getPoseString(entry);
            // std::cout << filename << std::endl;
            if (filename.compare(entry) == 0) {

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
