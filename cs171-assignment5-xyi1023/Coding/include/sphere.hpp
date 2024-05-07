#pragma once

#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include "shader.hpp"
#include "camera.hpp"

struct SphereVertex {
    glm::vec3 position;
    glm::vec3 normal;
};

struct Sphere {
private:
    std::vector<SphereVertex> vertices;
    std::vector<unsigned int> indices;

public:
    glm::vec3 center;
    float radius;
    Sphere(glm::vec3 center, float radius, glm::mat4 transform = glm::mat4(1.0));
    void draw(Shader* shader, FirstPersonCamera* camera) const;
};