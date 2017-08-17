#include <Mapper.hpp>

namespace cameval {

    Mapper::Mapper(std::string mappingFile, std::string database)
    {
        this->mappingFile = mappingFile;
        this->databaseFile = database;

        StringPoseMap mapping = InputReader::readMappingDatabase(mappingFile);
        StringList databaseEntries = InputReader::readDatabase(database);

        setFilenameMap(databaseEntries);
        fixStringPoseMap(mapping);
    }

    Mapper::~Mapper()
    {
        this->poseMapping.clear();
        this->filenameMap.clear();
    }

    void Mapper::setFilenameMap(StringList &databaseEntries)
    {
        for (std::string file : databaseEntries) {
            filenameMap.insert(std::make_pair(getKey(file), file));
        }
    }

    void Mapper::fixStringPoseMap(StringPoseMap &mapping)
    {
        for (auto pair : mapping) {
            std::string file = pair.first;
            this->poseMapping.insert(std::make_pair(getKey(file), pair.second));
        }
    }

    std::string Mapper::mapStringToFile(std::string &entry)
    {
        Float6Array key = getKey(entry);
        return filenameMap[key];
    }

    Pose Mapper::mapFileToPose(std::string &entry)
    {
        Float6Array key = getKey(entry);
        return poseMapping[key];
    }

    Float6Array Mapper::getKey(std::string &entry)
    {
        std::string file = entry;
        if (countBlocks(file) > 6) {
            file = getPoseString(file);
        }

        StringList blocks = divideStringPose(file);
        return convert(blocks);
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
