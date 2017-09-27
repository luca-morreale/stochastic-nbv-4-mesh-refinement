#ifndef CAM_POSITION_GENERATOR_AUTONOMOUS_PSO_CAM_GENERATOR_H
#define CAM_POSITION_GENERATOR_AUTONOMOUS_PSO_CAM_GENERATOR_H

#include <opview/AutonomousStochasticMethod.hpp>
#include <opview/PSOCamGenerator.hpp>

namespace opview {

    class AutonomousPSOCamGenerator : public AutonomousStochasticMethod {
    public:
        AutonomousPSOCamGenerator(CameraGeneralConfiguration &camConfig, MeshConfiguration &meshConfig, 
                                            StochasticConfiguration &config, size_t maxPoints, double offspring=1.0,
                                            double goalAngle=45, double dispersion=5);

        ~AutonomousPSOCamGenerator();
        
        virtual DoubleList estimateBestCameraPosition();
        
    protected:

        virtual void evaluateSwarm(GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void updateSwarmValues(DoubleList &values);

        virtual void updateParticles(GLMVec3List &centroids, GLMVec3List &normVectors);
        virtual void updatePositionParticle(int p);
        virtual void updateVelocityParticle(int p);
        void fixSpaceVelocity(int p);
        void fixSpacePosition(int p);

        void convertSamplesToParticles(OrderedPose &samples);

        virtual EigVector5List extractSwarmPositions();

        EigVector5 getBestParticlePosition();

        double uniform();
        EigVector5 randVector();

        void logParticles(int round);

        ParticleList particles;
        EigVector5 inertiaWeight;
        EigVector5 c1;
        EigVector5 c2;
    
    private:
        const gsl_rng *randGen;

        int bestParticleIndex;

        const long SEED = time(NULL);

        void deleteParticles();
        
    };

} // namespace opview

#endif // CAM_POSITION_GENERATOR_AUTONOMOUS_PSO_CAM_GENERATOR_H
