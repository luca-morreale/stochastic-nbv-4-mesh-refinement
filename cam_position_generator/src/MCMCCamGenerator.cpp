#include <opview/MCMCCamGenerator.hpp>


namespace opview {
    
    MCMCCamGenerator::MCMCCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, 
                        GLMVec3List &cams, StochasticConfiguration &stoConfig, double goalAngle, double dispersion)
                        : StochasticMethod(camConfig, meshFile, cams, stoConfig, goalAngle, dispersion)
    {
        this->sampler = new GaussianSampleGenerator();
        this->getLogger()->resetFile("mcmc.json");
    }

    MCMCCamGenerator::~MCMCCamGenerator()
    {
        delete sampler;
    }
    
    DoubleList MCMCCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose currentOptima = uniformSamplingStep(centroid, normVector, 0);

        for (int d = 0; d < getResamplingSteps(); d++) {
            currentOptima = pertubationSamplingStep(centroid, normVector, currentOptima, d+1);
        }

        ValuePose best = currentOptima.top();
        return convertVectorToList(best.second);    
    }

    OrderedPose MCMCCamGenerator::pertubationSamplingStep(GLMVec3 &centroid, GLMVec3 &normVector, OrderedPose &currentOptima, int round)
    {
        EigVector5List orientedPoints = resamplingPointsGetter(currentOptima);
        OrderedPose poses = computeEnergyForPoses(orientedPoints, centroid, normVector);

        return this->extractBestResults(poses, round);
    }

    EigVector5List MCMCCamGenerator::resamplingPointsGetter(OrderedPose &currentOptima)
    {  
        EigVector5List centers = getCentersFromOptima(currentOptima); 
        DoubleList weights = getWeightsFromOptima(currentOptima); 
        EigVector5List newCenters = sampler->getWeightedSamples(centers, weights, getResamplingParticles() * (1.0 - getOffspring()));
        return concatLists(newCenters, centers);
    }
    
    EigVector5List MCMCCamGenerator::getCentersFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        EigVector5List poses;
        while(!currentOptima.empty()) {
            poses.push_back(currentOptima.top().second);
            currentOptima.pop();
        }

        return poses;
    }

    DoubleList MCMCCamGenerator::getWeightsFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        DoubleList weights;
        while(!currentOptima.empty()) {
            double solution = currentOptima.top().first;
            weights.push_back(solution);
            currentOptima.pop();
        }

        return weights;
    }

} // namespace opview
