#ifndef EVALUATION_CAMERA_POSITION_INPUT_READER_H_
#define EVALUATION_CAMERA_POSITION_INPUT_READER_H_

#include <fstream>

#include <glm/gtc/matrix_transform.hpp>

#include <aliases.h>
#include <OpenMvgPoseConverter.hpp>
#include <utilities.hpp>

namespace cameval {
    
    class InputReader {
    public:
        static StringList readDatabase(std::string &database);
        static StringPoseMap readMappingDatabase(std::string &mappingFile);

    private:
        typedef GLMMat3 RotationMatrix;
        typedef GLMMat4 ProjectionMatrix;

    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_INPUT_READER_H_
