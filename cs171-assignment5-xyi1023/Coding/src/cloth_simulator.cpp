#include "cloth_simulator.hpp"

RectClothSimulator::
RectClothSimulator(
        RectCloth *cloth,
        float totalMass,
        float stiffnessReference,
        float airResistanceCoefficient,
        const glm::vec3& gravity) : cloth(cloth), airResistanceCoefficient(airResistanceCoefficient), gravity(gravity) {
    // Initialize particles, then springs according to the given cloth
    createMassParticles(totalMass);
    createSprings(stiffnessReference);
}

void RectClothSimulator::
createMassParticles(float totalMass) {
    // Create mass particles based on given cloth.
    particles.resize(cloth->nw * cloth->nh);
    for (unsigned int ih = 0; ih < cloth->nh; ih++) {
        for (unsigned int iw = 0; iw < cloth->nw; iw++) {
            MassParticle particle;
            particle.position = cloth->getPosition(iw, ih);

            // TODO: Initialize other mass properties.
            //  Use 'cloth->...' to access cloth properties.
            particle.mass = totalMass / (cloth->nw * cloth->nh);
            particle.velocity = glm::vec3(0.0f);
            particle.force = glm::vec3(0.0f);
            particle.acceleration = glm::vec3(0.0f);

            particles[cloth->idxFromCoord(iw, ih)] = particle;
        }
    }
}

void RectClothSimulator::
createSprings(float stiffnessReference) {
    // First clear all springs
    springs.clear();

    // TODO: Create springs connecting mass particles.
    //  You may find 'cloth->idxFromCoord(...)' useful.
    //  You can store springs into the member variable 'springs' which is a std::vector.
    //  You may want to modify mass particles too.
    for (unsigned int ih=0; ih<cloth->nh; ih++) {
        for (unsigned int iw=0; iw<cloth->nw; iw++) {
            // Horizontal springs
            if (iw < cloth->nw - 1) {
                Spring spring;
                spring.fromMassIndex = cloth->idxFromCoord(iw, ih);
                spring.toMassIndex = cloth->idxFromCoord(iw + 1, ih);
                spring.stiffness = stiffnessReference;
                spring.restLength = cloth->dx;
                spring.dampingCoefficient = 0.0f;
                springs.push_back(spring);
                particles[spring.fromMassIndex].connectedSpringStartIndices.push_back(springs.size() - 1);
                particles[spring.toMassIndex].connectedSpringEndIndices.push_back(springs.size() - 1);
            }
            // Vertical springs
            if (ih < cloth->nh - 1) {
                Spring spring;
                spring.fromMassIndex = cloth->idxFromCoord(iw, ih);
                spring.toMassIndex = cloth->idxFromCoord(iw, ih + 1);
                spring.stiffness = stiffnessReference;
                spring.restLength = cloth->dx;
                spring.dampingCoefficient = 0.0f;
                springs.push_back(spring);
                particles[spring.fromMassIndex].connectedSpringStartIndices.push_back(springs.size() - 1);
                particles[spring.toMassIndex].connectedSpringEndIndices.push_back(springs.size() - 1);
            }
            // Shear springs
            if (iw < cloth->nw - 1 && ih < cloth->nh - 1) {
                Spring spring;
                spring.fromMassIndex = cloth->idxFromCoord(iw, ih);
                spring.toMassIndex = cloth->idxFromCoord(iw + 1, ih + 1);
                spring.stiffness = stiffnessReference;
                spring.restLength = cloth->dx * sqrt(2.0f);
                spring.dampingCoefficient = 0.0f;
                springs.push_back(spring);
                particles[spring.fromMassIndex].connectedSpringStartIndices.push_back(springs.size() - 1);
                particles[spring.toMassIndex].connectedSpringEndIndices.push_back(springs.size() - 1);
            }
        }
    }

}

void RectClothSimulator::
step(float timeStep) {
    // TODO: Simulate one step based on given time step.
    //  Step 1: Update particle positions
    //  Step 2: Update springs
    //  Step 3: Apply constraints
    //  Hint: See cloth_simulator.hpp to check for member variables you need.
    //  Hint: You may use 'cloth->getInitialPosition(...)' for constraints.

    // BONUS: sphere colliders
    bool sphereColliders = true;

    // set sphere
    glm::vec3 spherePos = {0.0f, -1.0f, 0.0f};
    float sphereRadiance = 1.0f;
    
    // calculate force for each particle
    for (unsigned int i = 0u; i < particles.size(); i++) {
        particles[i].force = glm::vec3(0.0f);
        // gravity force
        particles[i].force += gravity * particles[i].mass;
        particles[i].force += -airResistanceCoefficient * particles[i].velocity; // should calculate twice velocity
        // calculate spring force, paticle as start point
        for (auto springIndex : particles[i].connectedSpringStartIndices) {
            Spring spring = springs[springIndex];
            glm::vec3 springVector = particles[spring.toMassIndex].position - particles[spring.fromMassIndex].position;
            float springLength = glm::length(springVector);
            glm::vec3 springDirection = glm::normalize(springVector);
            glm::vec3 springForce = spring.stiffness * (springLength - spring.restLength) * springDirection;
            particles[i].force += springForce;
        }
        // calculate spring force, paticle as end point
        for (auto springIndex : particles[i].connectedSpringEndIndices) {
            Spring spring = springs[springIndex];
            glm::vec3 springVector = particles[spring.fromMassIndex].position - particles[spring.toMassIndex].position;
            float springLength = glm::length(springVector);
            glm::vec3 springDirection = glm::normalize(springVector);
            glm::vec3 springForce = spring.stiffness * (springLength - spring.restLength) * springDirection;
            particles[i].force += springForce;
        }
        // BONUS: calculate wind force
        float windStrength = 0.005f;
        glm::vec3 windVector = glm::vec3(1.0f, 0.5f, 0.0f) * windStrength;
        glm::vec3 windForce = windVector;
        particles[i].force += windForce;
    }

    // update positions and velocities using euler method
    for (unsigned int i = 0u; i < particles.size(); i++) {
        // skip the fixed particles
        if (!((i == 0) || (i == cloth->nw - 1))) {
            particles[i].acceleration = particles[i].force / particles[i].mass;
            particles[i].velocity += particles[i].acceleration * timeStep;

            // update velocities considering sphere colliders
            if (sphereColliders) {
                // if the cloth is going to hit the sphere
                float r = glm::length(particles[i].position - spherePos);
                if (r - sphereRadiance <= 1e-3) {
                    glm::vec3 collisionNorm = glm::normalize(particles[i].position - spherePos);
                    // newPos = spherePos + collisionNorm * sphereRadiance;
                    if (glm::dot(particles[i].velocity, collisionNorm) < 0.0f) {
                    // calculate velocity along the normal should be minus
                        float velocityDotNorm = glm::dot(particles[i].velocity, collisionNorm);
                        particles[i].velocity -=  velocityDotNorm * collisionNorm;
                    // float restitutionCoefficient = 0.5f; 
                    // particles[i].velocity -= (1.0f + restitutionCoefficient) * velocityDotNorm * collisionNorm;
                    }
                }
            }/*  */
            // glm::vec3 newPos = particles[i].position + particles[i].velocity * timeStep;


            particles[i].position += particles[i].velocity * timeStep;
            // particles[i].position = newPos;
        }
    }

    // udpate fixed particles
    particles[0].position = cloth->getInitialPosition(0, 0);
    particles[0].velocity = glm::vec3(0.0f);
    particles[cloth->nw - 1].position = cloth->getInitialPosition(cloth->nw - 1, 0);
    particles[cloth->nw - 1].velocity = glm::vec3(0.0f);

    // Finally update cloth data
    updateCloth();
}

void RectClothSimulator::
updateCloth() {
    for (unsigned int i = 0u; i < cloth->nw * cloth->nh; i++)
    {
        cloth->setPosition(i, particles[i].position);
    }
}