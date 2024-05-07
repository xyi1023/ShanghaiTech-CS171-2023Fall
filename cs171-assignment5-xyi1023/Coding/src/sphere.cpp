#include "sphere.hpp"
#include <cmath>
#include <fstream>
#include <vector>
#include <sstream>

Sphere::Sphere(glm::vec3 center, float radius, glm::mat4 transform)
    : center(center), radius(radius) {

    const unsigned int X_SEGMENTS = 64;
    const unsigned int Y_SEGMENTS = 64;
    const float M_PI = 3.14159265358979323846f;

    for (unsigned int y = 0; y <= Y_SEGMENTS; ++y) {
        for (unsigned int x = 0; x <= X_SEGMENTS; ++x) {
            float xSegment = (float)x / (float)X_SEGMENTS;
            float ySegment = (float)y / (float)Y_SEGMENTS;
            float xPos = std::cos(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI) * radius;
            float yPos = std::cos(ySegment * M_PI) * radius;
            float zPos = std::sin(xSegment * 2.0f * M_PI) * std::sin(ySegment * M_PI) * radius;

            glm::vec3 pos = glm::vec3(xPos, yPos, zPos) + center; // 加上球心位置作为偏移量
            glm::vec3 norm = glm::normalize(pos - center); // 计算法线时应考虑球心偏移

            SphereVertex vertex;
            vertex.position = pos;
            vertex.normal = norm;
            vertices.push_back(vertex);
        }
    }

    // Generate indices
    for (unsigned int y = 0; y < Y_SEGMENTS; ++y) {
        for (unsigned int x = 0; x < X_SEGMENTS; ++x) {
            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
            indices.push_back(y * (X_SEGMENTS + 1) + x);
            indices.push_back(y * (X_SEGMENTS + 1) + x + 1);

            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
            indices.push_back(y * (X_SEGMENTS + 1) + x + 1);
            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x + 1);
        }
    }
}



void Sphere::draw(Shader* shader, FirstPersonCamera* camera) const {


    // Create Vertex Array Object, Vertex Buffer Object, and Element Buffer Object
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // Bind VAO
    glBindVertexArray(VAO);

    // Bind VBO and upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(SphereVertex), vertices.data(), GL_STATIC_DRAW);

    // Bind EBO and upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    // Vertex position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SphereVertex), (void*)offsetof(SphereVertex, position));

    // Vertex normal attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SphereVertex), (void*)offsetof(SphereVertex, normal));

    // Unbind VAO for now
    glBindVertexArray(0);

    // Bind VAO and draw the sphere
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // Unbind VAO after drawing
    glBindVertexArray(0);

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);

        // set shader
    shader->use();
    shader->setMat4("Projection", camera->getProjection());
    shader->setMat4("View", camera->getView());
    shader->setVec3("CameraPos", camera->getCameraPos());
    shader->setBool("DrawLine", false);
}
