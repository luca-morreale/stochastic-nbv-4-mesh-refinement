#ifndef MESH_ACCURACY_ACCURACY_MODEL_H
#define MESH_ACCURACY_ACCURACY_MODEL_H

#include <meshac/alias_definition.hpp>

namespace meshac {
    
    class AccuracyModel {
    public:
        virtual EigMatrixList getAccuracyForPoint(int index3DPoint) = 0;

    };


    typedef AccuracyModel * AccuracyModelPtr;



} // namespace meshac


#endif // MESH_ACCURACY_ACCURACY_MODEL_H
