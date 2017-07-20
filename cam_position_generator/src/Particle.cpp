#include <opview/Particle.hpp>

namespace opview {

    Particle::Particle(EigVector5 pose, double val) 
    {
        position = pose;
        bestPosition = pose;
        velocity = EigVector5::Zero();
        
        value = val;
        bestValue = val;
    }

    void Particle::updatePosition()
    {
        position = position + velocity;
    }

    void Particle::updateValue(double value)
    {
        this->value = value;
        if (bestValue < value) {
            this->bestPosition = this->position;
        }
    }

} // namespace opview
