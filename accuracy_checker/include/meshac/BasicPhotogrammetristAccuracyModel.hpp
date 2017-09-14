#ifndef MESH_ACCURACY_BASIC_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
#define MESH_ACCURACY_BASIC_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

#include <realtimeMR/SfMData.h>

#include <meshac/ResidualPointAccuracyModel.hpp>
#include <meshac/alias_definition.hpp>
#include <meshac/utilities.hpp>

namespace meshac {
    
    class BasicPhotogrammetristAccuracyModel : public ResidualPointAccuracyModel {
    public:
        BasicPhotogrammetristAccuracyModel(SfMData &data);
        ~BasicPhotogrammetristAccuracyModel();
        
        /*
         * Computes the matrix representing the uncertainty of the 3D point.
         * It takes into account all the observation of that point.
         */
        virtual EigMatrixList getAccuracyForPointByImage(int index3DPoint);        

    protected:
        virtual GLMMat3 computeJacobian(CameraMatrix &cam, GLMVec3 &point);
        virtual GLMVec3 getStraightProjection(GLMMat4 &P, GLMVec3 &point);

    private:
        GLMVec3List points;
        IntArrayList camPointMap;

        typedef ResidualPointAccuracyModel super;
    };


} // namespace meshac


#endif // MESH_ACCURACY_BASIC_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
