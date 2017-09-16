#ifndef CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H


#include <opview/StochasticMethod.hpp>

namespace opview {


    class MCMCCamGenerator : public StochasticMethod {
    public:
        MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                    StochasticConfiguration &stoConfig, double goalAngle=45, double dispersion=5);

        ~MCMCCamGenerator();

        virtual DoubleList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:
        
        virtual OrderedPose pertubationSamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, int round);
        
        virtual EigVector5List getCentersFromOptima(OrderedPose currentOptima);
        virtual DoubleList getWeightsFromOptima(OrderedPose currentOptima);

        virtual EigVector5List resamplingPointsGetter(OrderedPose &currentOptima);


    private:
        GaussianSampleGeneratorPtr sampler;
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_MCMC_CAM_GENERATOR_H
