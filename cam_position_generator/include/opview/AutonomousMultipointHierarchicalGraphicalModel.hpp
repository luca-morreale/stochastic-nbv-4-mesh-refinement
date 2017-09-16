#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H

#include <opview/MultipointHierarchicalGraphicalModel.hpp>
#include <opview/type_definition.h>

namespace opview {
    
    class AutonomousMultipointHierarchicalGraphicalModel : public MultipointHierarchicalGraphicalModel {
    public:
        AutonomousMultipointHierarchicalGraphicalModel(SolverGeneratorPtr solver, OrientationHierarchicalConfiguration &config,
                                                        CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                        size_t maxPoints, double goalAngle=55, double dispersion=5);
        ~AutonomousMultipointHierarchicalGraphicalModel();

        virtual LabelList estimateBestCameraPosition();

        virtual void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, GLMVec3 normal, double accuracy);
        virtual void updateMeshInfo(int pointIndex, double accuracy);
        virtual void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty);

    protected:
        // bring up overloaded function from parent
        using MultipointHierarchicalGraphicalModel::estimateBestCameraPosition;

        virtual void setupWorstPoints();
        virtual void updateWorstPoints(int index, long double uncertainty);
        GLMVec3ListPair getWorstPointsList();

        virtual GLMVec3List getPoints();
        virtual GLMVec3List getNormals();
        virtual DoubleList getUncertainties();

    private:
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;

        DoubleIntList worstPointsList;
        // long double SUM_UNCERTAINTY;
        size_t maxPoints;

        // void precomputeSumUncertainty();
        void retainWorst();

        typedef MultipointHierarchicalGraphicalModel super;
    };


} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_MULTI_POINT_HIERARCHICAL_GRAPHICAL_MODEL_H
