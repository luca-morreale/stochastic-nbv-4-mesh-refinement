#ifndef EVALUATION_CAMERA_POSITION_UNABLE_CREATE_DIRECTORY_EXCEPTION_H_
#define EVALUATION_CAMERA_POSITION_UNABLE_CREATE_DIRECTORY_EXCEPTION_H_

#include <stdexcept>

namespace cameval {
    
    class UnableToCreateDirectoryException : public std::runtime_error {
    public:
        UnableToCreateDirectoryException(std::string msg) : std::runtime_error(msg) 
        { /*    */ }

        UnableToCreateDirectoryException(boost::filesystem::path &destination) 
                : std::runtime_error("Unable to create destination directory" + destination.string() + "\n") 
        { /*    */ }
    };


} // namespace cameval


#endif // EVALUATION_CAMERA_POSITION_UNABLE_CREATE_DIRECTORY_EXCEPTION_H_
