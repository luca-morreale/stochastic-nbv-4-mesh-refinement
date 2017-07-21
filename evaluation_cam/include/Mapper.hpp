#ifndef EVALUATION_CAMERA_POSITION_MAPPER_H_
#define EVALUATION_CAMERA_POSITION_MAPPER_H_

#include <InputReader.hpp>
#include <aliases.h>

namespace cameval {

    class Mapper {
    public:
        Mapper();
        ~Mapper();
        Pose mapFileToPose(std::string entry);
        static Pose slowMappingToPose(std::string entry);

    private:
        PoseList poses;
        StringIntUMap strings;

        typedef GLMMat3 RotationMatrix;
        typedef GLMMat4 ProjectionMatrix;

    };

    typedef Mapper* MapperPtr;

} // cameval


#endif // EVALUATION_CAMERA_POSITION_MAPPER_H_
