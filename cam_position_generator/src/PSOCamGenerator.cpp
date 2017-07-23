#include <opview/PSOCamGenerator.hpp>

namespace opview {

    PSOCamGenerator::PSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            PSOConfiguration &psoConfig, double goalAngle, double dispersion)
                                            : MCMCCamGenerator(camConfig, meshFile, cams, psoConfig.mcConfig, goalAngle, dispersion)
    {
        randGen = gsl_rng_alloc(gsl_rng_mt19937);
        gsl_rng_set(randGen, SEED);
        setLogger(new SwarmReportWriter("pso_log.txt"));

        omega << 2, 0.729, 0.729, 0.729, 0.729;
        phiP << 2, 1.49445, 1.49445, 1.49445, 1.49445;

        spaceLowerBounds = psoConfig.spaceLowerBounds;
        spaceUpperBounds = psoConfig.spaceUpperBounds;
    }

    PSOCamGenerator::~PSOCamGenerator()
    {
        deleteParticles();
        particles.clear();
        gsl_rng_free(randGen);
    }

    void PSOCamGenerator::estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector)
    {
        OrderedPose currentOptima = uniformMCStep(centroid, normVector, 0);

        convertSamplesToParticles(currentOptima);

        for (int d = 0; d < getMCConfiguration().resamplingNum; d++) {
            updateParticles(centroid, normVector);
            logParticles(d);
        }
    }

    void PSOCamGenerator::convertSamplesToParticles(OrderedPose &samples)
    {
        this->bestParticleIndex = 0;    // due to priority queue the best particle is always the first one

        while(!samples.empty()){
            auto sample = samples.top();
            particles.push_back(new Particle(sample.second, sample.first));
            samples.pop();
        }
        // TODO build links
        // NOTE for now link only to the best particle
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
        double rp = this->uniform();
        double rg = this->uniform();

        EigVector5 dp = particles[p]->bestPosition - particles[p]->position;
        EigVector5 dg = particles[bestParticleIndex]->position - particles[p]->position;

        particles[p]->velocity = omega.cwiseProduct(particles[p]->velocity);
        particles[p]->velocity += rp * phiP.cwiseProduct(dp);
        
        if (bestParticleIndex != p) {
            particles[p]->velocity += rg * phiP.cwiseProduct(dg);
        }

        this->fixSpaceVelocity(p);
    }

    void PSOCamGenerator::fixSpaceVelocity(int p)
    {
        for (int i = 0; i < 3; i++) {
            if (fabs(particles[p]->velocity[i]) > (spaceUpperBounds[i] - spaceLowerBounds[i])) {
                particles[p]->velocity[i] = this->uniform() * (spaceUpperBounds[i] - spaceLowerBounds[i]);
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
            if (particles[p]->position[i] > (spaceLowerBounds[i])) {
                particles[p]->position[i] = spaceLowerBounds[i];
                particles[p]->velocity[i] = 0.0;
            } else if (particles[p]->position[i] < (spaceLowerBounds[i])) {
                particles[p]->position[i] = spaceLowerBounds[i];
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
    }

    double PSOCamGenerator::uniform()
    {
        return gsl_rng_uniform(randGen) * 0.001;
    }

    void PSOCamGenerator::logParticles(int round)
    {
        ((SwarmReportWriterPtr)this->getLogger())->append(particles, round);
        std::cout << "Best Value: " << this->particles[bestParticleIndex]->value << std::endl;
        std::cout << "Best Pose: " << this->particles[bestParticleIndex]->position.transpose() << std::endl << std::endl;
    }

    void PSOCamGenerator::deleteParticles()
    {
        for (int i = 0; i < particles.size(); i++) {
            delete particles[i];
        }
    }

} // namespace opview
