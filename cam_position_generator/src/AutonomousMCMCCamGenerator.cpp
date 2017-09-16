#include <opview/AutonomousMCMCCamGenerator.hpp>

namespace opview {

    AutonomousMCMCCamGenerator::AutonomousMCMCCamGenerator(CameraGeneralConfiguration &camConfig, 
                            MeshConfiguration &meshConfig, StochasticConfiguration &stoConfig, size_t maxPoints, 
                            double goalAngle, double dispersion)
                            : AutonomousStochasticMethod(camConfig, meshConfig, stoConfig, maxPoints, goalAngle, dispersion)
    {
        this->sampler = new GaussianSampleGenerator();
        this->getLogger()->resetFile("auto_mcmc.json");
    }
    
    AutonomousMCMCCamGenerator::~AutonomousMCMCCamGenerator()
    {
        delete sampler;
    }

    DoubleList AutonomousMCMCCamGenerator::estimateBestCameraPosition()
    {
        GLMVec3ListPair worst = getWorstPointsList();
        GLMVec3List centroids = worst.first;
        GLMVec3List normals = worst.second;

        OrderedPose currentOptima = uniformSamplingStep(centroids, normals, 0);

        for (int d = 0; d < getResamplingSteps(); d++) {
            currentOptima = resamplingStep(centroids, normals, currentOptima, d+1);
        }

        ValuePose best = currentOptima.top();
        return convertVectorToList(best.second);
    }

    OrderedPose AutonomousMCMCCamGenerator::resamplingStep(GLMVec3List &centroids, GLMVec3List &normals, OrderedPose &currentOptima, int round)
    {
        EigVector5List orientedPoints = resamplingPointsGetter(currentOptima);
        OrderedPose poses = computeEnergyForPoses(centroids, normals, orientedPoints);

        return this->extractBestResults(poses, round);
    }

    EigVector5List AutonomousMCMCCamGenerator::resamplingPointsGetter(OrderedPose &currentOptima)
    {  
        EigVector5List centers = getCentersFromOptima(currentOptima); 
        DoubleList weights = getWeightsFromOptima(currentOptima); 
        EigVector5List newCenters = sampler->getWeightedSamples(centers, weights, getResamplingParticles() * (1.0 - getOffspring()));
        return concatLists(newCenters, centers);
    }
    
    EigVector5List AutonomousMCMCCamGenerator::getCentersFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
    {
        EigVector5List poses;
        while(!currentOptima.empty()) {
            poses.push_back(currentOptima.top().second);
            currentOptima.pop();
        }

        return poses;
    }

    DoubleList AutonomousMCMCCamGenerator::getWeightsFromOptima(OrderedPose currentOptima) // not by refernce otherwise changes also the original
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
