#ifndef EVALUATION_CAMERA_POSITION_MAPPER_H_
#define EVALUATION_CAMERA_POSITION_MAPPER_H_

#include <aliases.h>
#include <InputReader.hpp>
#include <utilities.hpp>

namespace cameval {

    class Mapper {
    public:
        Mapper(std::string mappingFile);
        ~Mapper();

        std::string mapStringToFile(std::string &entry);
        Pose mapFileToPose(std::string entry);
        static Pose slowMappingToPose(std::string entry, std::string mappingFile="output_mapping.txt");

    protected:
        void setStringIndexMap(StringList &mapping);
        void setFilenameMap();

    private:
        PoseList poses;
        StringIntUMap strings;
        StringStringMap filenameMap;

        std::string mappingFile;

        typedef GLMMat3 RotationMatrix;
        typedef GLMMat4 ProjectionMatrix;

    };

    typedef Mapper* MapperPtr;

} // cameval


#endif // EVALUATION_CAMERA_POSITION_MAPPER_H_
