/*
 * Exceptions.hpp
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef JSON_CAM_FILLING_EXCEPTIONS_H_
#define JSON_CAM_FILLING_EXCEPTIONS_H_

#include <stdexcept>

namespace cameval {
    
    struct JsonParseException : public std::runtime_error
    {
        JsonParseException(const std::string& msg) : std::runtime_error(msg) {}
    };

    struct JsonAccessException : public std::runtime_error
    {
        JsonAccessException(const std::string& msg) : std::runtime_error(msg) {}
    };

    struct CameraNotPresentException : public std::runtime_error
    {
        CameraNotPresentException(const std::string& msg) : std::runtime_error(msg) {}
    };
}

#endif /* JSON_CAM_FILLING_EXCEPTIONS_H_ */
