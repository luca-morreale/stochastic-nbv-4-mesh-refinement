#ifndef EVALUATION_CAMERA_POSITION_SSH_EXCEPTION_H_
#define EVALUATION_CAMERA_POSITION_SSH_EXCEPTION_H_

#include <stdexcept>

namespace cameval {
    
    class SSHException : public std::runtime_error {
    public:
        SSHException(std::string msg) : std::runtime_error(msg) 
        { /*    */ }

        SSHException() : std::runtime_error("An error occured during the ssh connection") 
        { /*    */ }
    };


} // namespace cameval


#endif // EVALUATION_CAMERA_POSITION_SSH_EXCEPTION_H_
