#ifndef CAM_POSITION_GENERATOR_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_PSO_CAM_GENERATOR_H


#include <opview/StochasticMethod.hpp>
#include <opview/SwarmReportWriter.hpp>
#include <opview/Particle.hpp>

namespace opview {

    class PSOCamGenerator : public StochasticMethod {
    public:
        PSOCamGenerator(CameraGeneralConfiguration &camConfig, std::string &meshFile, GLMVec3List &cams, 
                                            StochasticConfiguration &stoConfig, double offspring=0.0, double goalAngle=45, double dispersion=5);

        ~PSOCamGenerator();

        virtual DoubleList estimateBestCameraPosition(GLMVec3 &centroid, GLMVec3 &normVector);

    protected:
        virtual void convertSamplesToParticles(OrderedPose &currentOptima);
        virtual void updateParticles(GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void updateVelocityParticle(int p);
        virtual void updatePositionParticle(int p);
        virtual void evaluateSwarm(GLMVec3 &centroid, GLMVec3 &normVector);
        virtual void updateSwarmValues(DoubleList &values);

        virtual void fixSpaceVelocity(int p);
        virtual void fixSpacePosition(int p);

        virtual double uniform();
        virtual EigVector5 randVector();
        
    
        ParticleList particles;
        EigVector5 inertiaWeight;
        EigVector5 c1;
        EigVector5 c2;

    private:
        const gsl_rng *randGen;

        int bestParticleIndex;

        const long SEED = time(NULL);

        void logParticles(int round);
        void deleteParticles();
        EigVector5List extractSwarmPositions();

    };

    typedef PSOCamGenerator* PSOCamGeneratorPtr;

} // namespace opview

#endif // CAM_POSITION_GENERATOR_PSO_SAMPLE_GENERATOR_H
