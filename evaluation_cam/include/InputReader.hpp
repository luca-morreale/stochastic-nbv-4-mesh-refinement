#ifndef EVALUATION_CAMERA_POSITION_INPUT_READER_H_
#define EVALUATION_CAMERA_POSITION_INPUT_READER_H_

#include <fstream>

#include <aliases.h>
#include <utilities.hpp>

namespace cameval {
    
    class InputReader {
    public:
        static StringList readDatabase(std::string &database);

    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_INPUT_READER_H_
