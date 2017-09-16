#include <opview/AutonomousPSOCamGenerator.hpp>


namespace opview {

    AutonomousPSOCamGenerator::AutonomousPSOCamGenerator(CameraGeneralConfiguration &camConfig, 
                        MeshConfiguration &meshConfig, StochasticConfiguration &config, size_t maxPoints, double goalAngle, double dispersion)
                        : AutonomousStochasticMethod(camConfig, meshConfig, config, maxPoints, goalAngle, dispersion)
    {
        getLogger()->resetFile("auto_pso.json");

        this->randGen = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(randGen, SEED);

        inertiaWeight << 0.9, 0.9, 0.9, 0.9, 0.9;   // above 1.4 looks globally, below 0.8 look locally
        c1 << 0.70, 0.70, 0.60, 0.70, 0.70;
        c2 << 0.70, 0.70, 0.60, 0.70, 0.70;
    }
    
    AutonomousPSOCamGenerator::~AutonomousPSOCamGenerator()
    {
        deleteParticles();
        particles.clear();
        gsl_rng_free(randGen);
    }

    // void AutonomousPSOCamGenerator::precomputeSumUncertainty()
    // {
    //     SUM_UNCERTAINTY = 0.0;
    //     for (int p = 0; p < uncertainty.size(); p++) {
    //         SUM_UNCERTAINTY += uncertainty[p];
    //     }
    // }

    // double AutonomousPSOCamGenerator::computeWeightForPoint(int pointIndex)
    // {
    //     return (double)uncertainty[pointIndex] / (double)SUM_UNCERTAINTY;
    // }

    DoubleList AutonomousPSOCamGenerator::estimateBestCameraPosition()
    {
        GLMVec3ListPair worst = getWorstPointsList();
        GLMVec3List centroids = worst.first;
        GLMVec3List normals = worst.second;
        OrderedPose currentOptima = uniformSamplingStep(centroids, normals, 0);

        convertSamplesToParticles(currentOptima);
        
        for (int d = 0; d < getResamplingSteps(); d++) {
            updateParticles(centroids, normals);
            logParticles(d);
            c1 = c1 * 0.75f;
            c2 = c2 * 0.75f;
        }

        EigVector5 best = this->particles[bestParticleIndex]->position;
        return convertVectorToList(best);
    }

    void AutonomousPSOCamGenerator::convertSamplesToParticles(OrderedPose &samples)
    {
        this->bestParticleIndex = 0;    // due to priority queue the best particle is always the first one

        while(!samples.empty()){
            auto sample = samples.top();
            particles.push_back(new Particle(sample.second, sample.first));
            samples.pop();
        }
    }

    void AutonomousPSOCamGenerator::updateParticles(GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        #pragma omp parallel for
        for (int p = 0; p < particles.size(); p++) {
            updateVelocityParticle(p);
            updatePositionParticle(p);
        }
        evaluateSwarm(centroids, normVectors);
    }

    void AutonomousPSOCamGenerator::updateVelocityParticle(int p)
    {
        EigVector5 randBest = this->randVector();
        EigVector5 randPrevious = this->randVector();

        EigVector5 diffPreviousBest = particles[p]->bestPosition - particles[p]->position;
        EigVector5 diffGlobalBest = particles[bestParticleIndex]->position - particles[p]->position;

        particles[p]->velocity = inertiaWeight.cwiseProduct(particles[p]->velocity);
        particles[p]->velocity += randPrevious.cwiseProduct(c1.cwiseProduct(diffPreviousBest));
        particles[p]->velocity += randBest.cwiseProduct(c2.cwiseProduct(diffGlobalBest));      // should be 0 if p is the best particle

        this->fixSpaceVelocity(p);
    }

    void AutonomousPSOCamGenerator::fixSpaceVelocity(int p)
    {
        for (int i = 0; i < 3; i++) {
            if (fabs(particles[p]->velocity[i]) > (upperBounds()[i] - lowerBounds()[i])) {
                particles[p]->velocity[i] = this->uniform() * (upperBounds()[i] - lowerBounds()[i]);
            }
        }
    }

    void AutonomousPSOCamGenerator::updatePositionParticle(int p)
    {
        particles[p]->updatePosition();
        this->fixSpacePosition(p);
    }

    void AutonomousPSOCamGenerator::fixSpacePosition(int p)
    {
        for (int i = 0; i < 3; i++) {
            if (particles[p]->position[i] < lowerBounds()[i]) {
                particles[p]->position[i] = lowerBounds()[i];
                particles[p]->velocity[i] = 0.0;
            } else if (particles[p]->position[i] > upperBounds()[i]) {
                particles[p]->position[i] = upperBounds()[i];
                particles[p]->velocity[i] = 0.0;
            }
        }
    }

    EigVector5List AutonomousPSOCamGenerator::extractSwarmPositions()
    {
        EigVector5List orientedPoints(particles.size());

        #pragma omp parallel for
        for (int p = 0; p < particles.size(); p++) {
            orientedPoints[p] = particles[p]->position;
        }
        return orientedPoints;
    }

    void AutonomousPSOCamGenerator::evaluateSwarm(GLMVec3List &centroids, GLMVec3List &normVectors)
    {
        EigVector5List orientedPoints = extractSwarmPositions();
        
        DoubleList values(orientedPoints.size());

        #pragma omp parallel for
        for (int i = 0; i < orientedPoints.size(); i++) {
            values[i] = getFormulation()->computeEnergy(orientedPoints[i], centroids, normVectors);
        }

        updateSwarmValues(values);        
    }

    void AutonomousPSOCamGenerator::updateSwarmValues(DoubleList &values)
    {
        #pragma omp parallel for
        for (int p = 0; p < particles.size(); p++) {
            particles[p]->updateValue(values[p]);

            #pragma omp critical
            if (values[p] > particles[bestParticleIndex]->value) {
                this->bestParticleIndex = p;
            }
        }

        for (int p = 0; p < particles.size(); p++) {
            if (values[p] > particles[bestParticleIndex]->value) {
                this->bestParticleIndex = p;
            }
        }
    }

    double AutonomousPSOCamGenerator::uniform()
    {
        return gsl_rng_uniform(randGen);
    }

    EigVector5 AutonomousPSOCamGenerator::randVector()
    {
        EigVector5 random;
        random << uniform(), uniform(), uniform(), uniform(), uniform();
        return random;
    }

    void AutonomousPSOCamGenerator::logParticles(int round)
    {
        ((SwarmReportWriterPtr) getLogger())->append(particles, round);

        EigVector5 tmp = this->particles[bestParticleIndex]->position;
        tmp[3] = rad2deg(tmp[3]);
        tmp[4] = rad2deg(tmp[4]);
    }

    void AutonomousPSOCamGenerator::deleteParticles()
    {
        for (int i = 0; i < particles.size(); i++) {
            delete particles[i];
        }
    }

} // namespace opview
