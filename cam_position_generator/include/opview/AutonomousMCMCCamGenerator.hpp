#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H

#include <opview/AutonomousStochasticMethod.hpp>
#include <opview/GaussianSampleGenerator.hpp>

namespace opview {

    class AutonomousMCMCCamGenerator : public AutonomousStochasticMethod {
    public:
        AutonomousMCMCCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                StochasticConfiguration &stoConfig, size_t maxPoints, double offspring=0.9, double goalAngle=45, double dispersion=5);

        ~AutonomousMCMCCamGenerator();


        virtual DoubleList estimateBestCameraPosition();
        
    protected:
        OrderedPose resamplingStep(GLMVec3List &centroids, GLMVec3List &normals, OrderedPose &currentOptima, int round);
        EigVector5List resamplingPointsGetter(OrderedPose &currentOptima);
        EigVector5List getCentersFromOptima(OrderedPose currentOptima); // not by refernce otherwise changes also the original
        DoubleList getWeightsFromOptima(OrderedPose currentOptima); // not by refernce otherwise changes also the original
        
    private:
        GaussianSampleGeneratorPtr sampler;
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_MCMC_CAM_GENERATOR_H
