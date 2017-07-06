#ifndef EVALUATION_CAMERA_POSITION_NO_CLOSE_POINT_EXCEPTION_H_
#define EVALUATION_CAMERA_POSITION_NO_CLOSE_POINT_EXCEPTION_H_

#include <stdexcept>

namespace cameval {
    
    class NoClosePointException : public std::runtime_error {
    public:
        NoClosePointException(std::string msg) : std::runtime_error(msg) 
        { /*    */ }

        NoClosePointException() : std::runtime_error("Does not exist any point close to the given one. Maybe the tree has not been initialized.") 
        { /*    */ }
    };


} // namespace cameval


#endif // EVALUATION_CAMERA_POSITION_NO_CLOSE_POINT_EXCEPTION_H_
