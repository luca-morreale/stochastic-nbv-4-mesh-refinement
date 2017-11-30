#ifndef MESH_ACCURACY_INVERTED_RESIDUAL_ACCURACY_MODEL_H
#define MESH_ACCURACY_INVERTED_RESIDUAL_ACCURACY_MODEL_H


#include <meshac/alias_definition.hpp>
#include <meshac/ResidualPointAccuracyModel.hpp>
#include <meshac/SfMData.h>


namespace meshac {
    
    class InvertedResidualPointAccuracyModel : public ResidualPointAccuracyModel {
    public:
        InvertedResidualPointAccuracyModel(SfMData &data);
        ~InvertedResidualPointAccuracyModel();

    protected:

        virtual EigMatrix computeResidual(CamPointPair &camToPoint, GLMVec3 &point);

    private:
        GLMVec3 getRetroProjection(CameraMatrix &P, GLMVec2 &point2D);
        
    };

    typedef InvertedResidualPointAccuracyModel * InvertedResidualPointAccuracyModelPtr;

} // namespace meshac


#endif // MESH_ACCURACY_INVERTED_RESIDUAL_ACCURACY_MODEL_H
