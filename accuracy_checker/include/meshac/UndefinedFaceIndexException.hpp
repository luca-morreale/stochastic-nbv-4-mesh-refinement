#ifndef MESH_ACCURACY_UNDEFINED_FACE_INDEX_TRIANGLE_EXCEPTION_H
#define MESH_ACCURACY_UNDEFINED_FACE_INDEX_TRIANGLE_EXCEPTION_H

#include <stdexcept>

namespace meshac {

    class UndefinedFaceIndexException : public std::runtime_error {
    public:
        UndefinedFaceIndexException(std::string msg) : std::runtime_error(msg)
        { /*    */ }

        UndefinedFaceIndexException(int index) : std::runtime_error("Undefined face/triangle in the mesh for the given index "+std::to_string(index)+".")
        { /*    */ }

        UndefinedFaceIndexException() : std::runtime_error("Undefined face/triangle in the mesh for the given index.")
        { /*    */ }

    };

} // namespace meshac

#endif // MESH_ACCURACY_UNDEFINED_FACE_INDEX_TRIANGLE_EXCEPTION_H
