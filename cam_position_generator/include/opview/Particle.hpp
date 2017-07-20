#ifndef CAM_POSITION_GENERATOR_PSO_PARTICLE_H
#define CAM_POSITION_GENERATOR_PSO_PARTICLE_H

#include <opview/alias_definition.h>
#include <opview/utilities.hpp>

namespace opview {
    
    typedef struct Particle {
        public:
            Particle(EigVector5 pose, double val=-DBL_MAX);
            Particle();

            void updatePosition();
            void updateValue(double value);

            EigVector5 position;
            EigVector5 velocity;            
            EigVector5 bestPosition;
            
            double value;
            double bestValue;


        } Particle;

        typedef Particle* ParticlePtr;
        typedef std::vector<ParticlePtr> ParticleList;


} // namespace opview

#endif // CAM_POSITION_GENERATOR_PSO_PARTICLE_H
