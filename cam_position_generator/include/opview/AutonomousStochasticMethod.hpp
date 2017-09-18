#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_STOCHASTIC_METHOD_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_STOCHASTIC_METHOD_H

#include <opview/NotImplementedMethodException.hpp>
#include <opview/StochasticMethod.hpp>

namespace opview {

    
    class AutonomousStochasticMethod : public StochasticMethod {
    public:
        AutonomousStochasticMethod(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                    StochasticConfiguration &stoConfig, size_t maxPoints, double offspring=0.9, double goalAngle=45, double dispersion=5);

        ~AutonomousStochasticMethod();

        virtual DoubleList estimateBestCameraPosition() = 0;
        virtual DoubleList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

        GLMVec3List getPoints();
        GLMVec3List getNormals();
        DoubleList getUncertainties();
        void updateMeshInfo(int pointIndex, GLMVec3 point, GLMVec3 normal, double uncertainty);
        void updateMeshInfo(int pointIndex, GLMVec3 normal, double uncertainty);
        void updateMeshInfo(int pointIndex, double uncertainty);
        void addPoint(GLMVec3 point, GLMVec3 normal, double uncertainty);

        
    protected:

        OrderedPose uniformSamplingStep(GLMVec3List &centroids, GLMVec3List &normals, int round);
        OrderedPose computeEnergyForPoses(GLMVec3List &centroids, GLMVec3List &normals, EigVector5List &orientedPoints);

    

        virtual void setupWorstPoints();
        virtual void updateWorstPoints(int index, long double uncertainty);
        virtual void retainWorst();
        virtual GLMVec3ListPair getWorstPointsList();

    

        
        
    private:
        GLMVec3List points;
        GLMVec3List normals;
        DoubleList uncertainty;
        size_t maxPoints;

        DoubleIntList worstPointsList;
        
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_STOCHASTIC_METHOD_H
