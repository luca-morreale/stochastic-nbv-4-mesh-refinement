#ifndef CAM_POSITION_NOT_IMPLEMENTED_METHOD_EXCEPTION_H
#define CAM_POSITION_NOT_IMPLEMENTED_METHOD_EXCEPTION_H

#include <stdexcept>

namespace opview {

    class NotImplementedMethodException : public std::runtime_error {
    public:
        NotImplementedMethodException(std::string msg) : std::runtime_error(msg) { }
        
        ~NotImplementedMethodException() { }
    };


} // namespace opview

#endif // CAM_POSITION_NOT_IMPLEMENTED_METHOD_EXCEPTION_H
