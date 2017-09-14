#include <opview/PSOCamGenerator.hpp>

namespace opview {

    PSOCamGenerator::PSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            MCConfiguration &config, double goalAngle, double dispersion)
                                            : MCMCCamGenerator(camConfig, meshFile, cams, config, goalAngle, dispersion)
    {
        randGen = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(randGen, SEED);
        setLogger(new SwarmReportWriter("pso_log.txt"));

        inertiaWeight << 0.9, 0.9, 0.9, 0.9, 0.9;   // above 1.4 looks globally, below 0.8 look locally
        c1 << 0.70, 0.70, 0.60, 0.70, 0.70;
        c2 << 0.70, 0.70, 0.60, 0.70, 0.70;
    }

    PSOCamGenerator::~PSOCamGenerator()
    {
        deleteParticles();
        particles.clear();
        gsl_rng_free(randGen);
    }

    DoubleList PSOCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose currentOptima = uniformMCStep(centroid, normVector, 0);
        convertSamplesToParticles(currentOptima);
        
        for (int d = 0; d < getMCConfiguration().resamplingNum; d++) {
            updateParticles(centroid, normVector);
            logParticles(d);
            c1 = c1 * 0.75f;
            c2 = c2 * 0.75f;
        }

        EigVector5 best = this->particles[bestParticleIndex]->position;
        return convertVectorToList(best);
    }

    void PSOCamGenerator::convertSamplesToParticles(OrderedPose &samples)
    {
        this->bestParticleIndex = 0;    // due to priority queue the best particle is always the first one

        while(!samples.empty()){
            auto sample = samples.top();
            particles.push_back(new Particle(sample.second, sample.first));
            samples.pop();
        }
    }

    void PSOCamGenerator::updateParticles(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        #pragma omp parallel for
        for (int p = 0; p < particles.size(); p++) {
            updateVelocityParticle(p);
            updatePositionParticle(p);
        }
        evaluateSwarm(centroid, normVector);
    }

    void PSOCamGenerator::updateVelocityParticle(int p)
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

    void PSOCamGenerator::fixSpaceVelocity(int p)
    {
        for (int i = 0; i < 3; i++) {
            if (fabs(particles[p]->velocity[i]) > (upperBounds()[i] - lowerBounds()[i])) {
                particles[p]->velocity[i] = this->uniform() * (upperBounds()[i] - lowerBounds()[i]);
            }
        }
    }

    void PSOCamGenerator::updatePositionParticle(int p)
    {
        particles[p]->updatePosition();
        this->fixSpacePosition(p);
    }

    void PSOCamGenerator::fixSpacePosition(int p)
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

    EigVector5List PSOCamGenerator::extractSwarmPositions()
    {
        EigVector5List orientedPoints(particles.size());

        #pragma omp parallel for
        for (int p = 0; p < particles.size(); p++) {
            orientedPoints[p] = particles[p]->position;
        }
        return orientedPoints;
    }

    void PSOCamGenerator::evaluateSwarm(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        DoubleList visibility(particles.size(), 0.0);
        DoubleList vonMises(particles.size(), 0.0);
        DoubleList projection(particles.size(), 0.0);
        DoubleList constraints(particles.size(), 0.0);
        DoubleList values(particles.size(), 0.0);

        EigVector5List orientedPoints = extractSwarmPositions();
        
        #pragma omp parallel sections 
        {
            #pragma omp section
            computeObjectiveFunction(vonMises, orientedPoints, centroid, normVector);
            #pragma omp section
            computeVisibilityFunction(visibility, orientedPoints, centroid, normVector);
            #pragma omp section
            computeProjectionFunction(projection, orientedPoints, centroid, normVector);
            #pragma omp section
            computeConstraintFunction(constraints, orientedPoints, centroid, normVector);
        }

        sumUpAll(values, visibility, vonMises, projection, constraints);
        updateSwarmValues(values);        
    }

    void PSOCamGenerator::updateSwarmValues(DoubleList &values)
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

    double PSOCamGenerator::uniform()
    {
        return gsl_rng_uniform(randGen);
    }

    EigVector5 PSOCamGenerator::randVector()
    {
        EigVector5 random;
        random << uniform(), uniform(), uniform(), uniform(), uniform();
        return random;
    }

    void PSOCamGenerator::logParticles(int round)
    {
        ((SwarmReportWriterPtr)this->getLogger())->append(particles, round);

        EigVector5 tmp = this->particles[bestParticleIndex]->position;
        tmp[3] = rad2deg(tmp[3]);
        tmp[4] = rad2deg(tmp[4]);
    }

    void PSOCamGenerator::deleteParticles()
    {
        for (int i = 0; i < particles.size(); i++) {
            delete particles[i];
        }
    }

} // namespace opview
