#ifndef CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

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
        MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                        size_t maxPoints, long double maxUncertainty, double goalAngle=55, double dispersion=5);
        ~MultipointHierarchicalGraphicalModel();

        // bring up overloaded function from parent
        using OrientationHierarchicalGraphicalModel::estimateBestCameraPosition;
        virtual void estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVector);

        void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy);
        void updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy);
        void updateMeshInfo(int pointIndex, double accuracy);
        void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty);

    protected:
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3List &centroids, GLMVec3List &normVectors);
    
        virtual void fillExplicitOrientationFunction(GMExplicitFunction &modelFunction, BoostObjFunction evals, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void fillObjectiveFunction(GMExplicitFunction &objFunction, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void computeDistribution(GMExplicitFunction &modelFunctions, BoostObjFunction &evals, size_t coord[], GLMVec3List &centroids, GLMVec3List &normVectors);
    
        virtual void fillConstraintFunction(GMExplicitFunction &constraints, GLMVec3List &centroids);
        virtual void addValueToConstraintFunction(GMExplicitFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3List &centroids, GLMVec3 spacePos);

        virtual LabelType visibilityDistribution(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector) override;
        virtual LabelType imagePlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector) override;

        virtual double estimateForWorstPointSeen(EigVector5 &pose, BoostObjFunction function);
        virtual double computeWeightForPoint(int pointIndex);
    
    private:
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;

        long double SUM_UNCERTAINTY;
        long double maxUncertainty;
        size_t maxPoints;

        typedef OrientationHierarchicalGraphicalModel super;

        LabelType parentCallToVisibilityEstimation(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);
        LabelType parentCallToPlaneWeight(EigVector5 &pose, GLMVec3 &centroid, GLMVec3 &normalVector);

    };


} // namespace opview

#endif // CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
