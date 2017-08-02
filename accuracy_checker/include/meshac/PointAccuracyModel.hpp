#ifndef MESH_ACCURACY_ACCURACY_MODEL_H
#define MESH_ACCURACY_ACCURACY_MODEL_H

#include <meshac/alias_definition.hpp>

namespace meshac {
    
    class PointAccuracyModel {
    public:
        /*
         * Computes the matrix that represents the accuracy of the 3D point.
         * A matrix for each image is returned.
         */
        virtual EigMatrixList getAccuracyForPointByImage(int index3DPoint) = 0;

    };


    typedef PointAccuracyModel * PointAccuracyModelPtr;



} // namespace meshac


#endif // MESH_ACCURACY_ACCURACY_MODEL_H
