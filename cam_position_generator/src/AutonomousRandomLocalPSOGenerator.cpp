#include <opview/AutonomousRandomLocalPSOGenerator.hpp>

namespace opview {

    AutonomousRandomLocalPSOGenerator::AutonomousRandomLocalPSOGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                                StochasticConfiguration &config, double offspring, double goalAngle, double dispersion)
                                                : AutonomousLocalPSOCamGenerator(camConfig, meshConfig, config, offspring, goalAngle, dispersion)
    {
        std::random_device rd;
        randGen = std::mt19937(rd());
    }

    AutonomousRandomLocalPSOGenerator::~AutonomousRandomLocalPSOGenerator()
    { /*    */ }

    DoubleList AutonomousRandomLocalPSOGenerator::estimateBestCameraPosition()
    {
        GLMVec3ListPair worst = getWorstPointsList();
        GLMVec3List centroids = worst.first;
        GLMVec3List normals = worst.second;
        OrderedPose currentOptima = randomSampligStep(centroids, normals, 0);

        convertSamplesToParticles(currentOptima);
        
        for (int d = 0; d < getResamplingSteps(); d++) {
            updateParticles(centroids, normals);
            logParticles(d);
            c1 = c1 * 0.75f;
            c2 = c2 * 0.75f;
        }

        EigVector5 best = getBestParticlePosition();
        return convertVectorToList(best);
    }

    OrderedPose AutonomousRandomLocalPSOGenerator::randomSampligStep(GLMVec3List &centroids, GLMVec3List &normals, int round)
    {
        EigVector5List orientedPoints = randomPointsGetter();
        OrderedPose poses = computeEnergyForPoses(centroids, normals, orientedPoints);
        return this->extractBestResults(poses, round);
    }

    EigVector5List AutonomousRandomLocalPSOGenerator::randomPointsGetter()
    {
        EigVector5List list;
        GLMVec3 lower = lowerBounds();
        GLMVec3 upper = upperBounds();

        EigVector5 variances, center;
        center << (upper.x + lower.x) / 2.0f, (upper.y + lower.y) / 2.0f, 
                (upper.z + lower.z) / 2.0f, 0.0f, 0.0f;
        variances << (upper.x - lower.x) / 6.0f, (upper.y - lower.y) / 6.0f, 
                (upper.z - lower.z) / 6.0f, M_PI / 3.0f, M_PI / 3.0f;

        for (int i = 0; i < getResamplingParticles(); i++) {
            list.push_back(getSample(center, variances));
        }
        return list;
    }

    EigVector5 AutonomousRandomLocalPSOGenerator::getSample(EigVector5 &center, EigVector5 &variance)
    {
        EigVector5 sample;
        for (int i = 0; i < 5; i++) {
            std::normal_distribution<float> d(center[i], variance[i]);
            sample[i] = d(randGen);
        }
        return sample;
    }


} // namespace opview
