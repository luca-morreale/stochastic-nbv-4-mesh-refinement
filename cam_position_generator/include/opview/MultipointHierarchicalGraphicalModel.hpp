#ifndef CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <opview/OrientationHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>
#include <opview/DimensionDiagreementLists.hpp>

namespace opview {

    
    class MultipointHierarchicalGraphicalModel : public OrientationHierarchicalGraphicalModel {
    public:
        MultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, std::string meshFile, 
                                                        GLMVec3List &cams, double goalAngle=55, double dispersion=5);
        ~MultipointHierarchicalGraphicalModel();

        virtual void estimateBestCameraPosition(GLMVec3List &centroids, GLMVec3List &normVector);

    protected:
        
        virtual void fillModel(GraphicalModelAdder &model, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void fillObjectiveFunction(GMSparseFunction &vonMises, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual LabelType computeObjectiveValueForList(EigVector5 &pose, GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void fillConstraintFunction(GMSparseFunction &constraints, GMSparseFunction &distances, GLMVec3List &centroids);
        virtual void addValueToConstraintFunction(GMSparseFunction &function, GLMVec3 &point, GLMVec3 &cam, GLMVec3List &centroids, GLMVec3 spacePos);


    };


} // namespace opview

#endif // CAM_POSITION_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
