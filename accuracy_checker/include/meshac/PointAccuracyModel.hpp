#ifndef MESH_ACCURACY_POINT_ACCURACY_MODEL_H
#define MESH_ACCURACY_POINT_ACCURACY_MODEL_H

#include <meshac/alias_definition.hpp>
#include <meshac/UnexpectedPointException.hpp>

namespace meshac {
    
    class PointAccuracyModel {
    public:
        PointAccuracyModel(GLMVec3List &points);
        ~PointAccuracyModel();
        /*
         * Computes the matrix that represents the accuracy of the 3D point.
         * A matrix for each image is returned.
         */
        virtual EigMatrixList getAccuracyForPointByImage(int index3DPoint) = 0;
        virtual EigMatrixList getAccuracyForPointByImage(GLMVec3 &point);

        virtual GLMVec3List getPoints();

    protected:
        virtual int retreiveIndex(GLMVec3 &point);

    private:
        GLMVec3List points;

    };

    typedef PointAccuracyModel * PointAccuracyModelPtr;

} // namespace meshac


#endif // MESH_ACCURACY_POINT_ACCURACY_MODEL_H
