#ifndef EVALUATION_CAMERA_POSITION_SOURCE_DIRECTORY_NOT_EXISTS_EXCEPTION_H_
#define EVALUATION_CAMERA_POSITION_SOURCE_DIRECTORY_NOT_EXISTS_EXCEPTION_H_

#include <stdexcept>

namespace cameval {
    
    class SourceDirectoryDoesNotExistsException : public std::runtime_error {
    public:
        SourceDirectoryDoesNotExistsException(std::string msg) : std::runtime_error(msg) 
        { /*    */ }

        SourceDirectoryDoesNotExistsException(boost::filesystem::path &source) 
                : std::runtime_error("Source directory " + source.string() + " does not exist or is not a directory.\n") 
        { /*    */ }
    };


} // namespace cameval


#endif // EVALUATION_CAMERA_POSITION_SOURCE_DIRECTORY_NOT_EXISTS_EXCEPTION_H_
