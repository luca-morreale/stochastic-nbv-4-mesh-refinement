#ifndef CAM_POSITION_GENERATOR_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <opview/DimensionDisagreementLists.hpp>
#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    // in case of single point it just estimate the best point using a weighted function that consider all visible points
    
    class MultipointHierarchicalGraphicalModel : public OrientationHierarchicalGraphicalModel {
    public:
        MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                        GLMVec3List &cams, double goalAngle=55, double dispersion=5);

        ~MultipointHierarchicalGraphicalModel();

        // bring up overloaded function from parent
        using OrientationHierarchicalGraphicalModel::estimateBestCameraPosition;
        virtual LabelList estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVector);

    protected:
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3List &centroids, GLMVec3List &normVectors);
    
        virtual void fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void computeDistribution(GMExplicitFunction &modelFunctions, BoostObjFunction &evals, size_t coord[], GLMVec3List &centroids, GLMVec3List &normVectors);
    
        virtual void fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3List &centroids);
        virtual void fillExplicitFunction(GMExplicitFunction &function, LabelType val, size_t x, size_t y, size_t z);
    
    private:

        typedef OrientationHierarchicalGraphicalModel super;
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
