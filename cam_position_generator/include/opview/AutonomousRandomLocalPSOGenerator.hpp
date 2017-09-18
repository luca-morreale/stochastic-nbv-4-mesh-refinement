#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_RANDOM_LOCAL_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_RANDOM_LOCAL_PSO_CAM_GENERATOR_H

#include <opview/AutonomousLocalPSOCamGenerator.hpp>

namespace opview {

    class AutonomousRandomLocalPSOGenerator : public AutonomousLocalPSOCamGenerator {
    public:
        AutonomousRandomLocalPSOGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                            StochasticConfiguration &config, double offspring=0.0, double goalAngle=45, double dispersion=5);

        ~AutonomousRandomLocalPSOGenerator();

        virtual DoubleList estimateBestCameraPosition();

    protected:
        virtual OrderedPose randomSampligStep(GLMVec3List &centroids, GLMVec3List &normals, int round);
        virtual EigVector5List randomPointsGetter();

        virtual EigVector5 getSample(EigVector5 &center, EigVector5 &variance);

    private:
        std::mt19937 randGen;

    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_RANDOM_LOCAL_PSO_CAM_GENERATOR_H