#pragma 

#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <memory>

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh() = default;

    void init();
    void clear();

    static void loadOBJ(const std::string& filename, std::shared_ptr<Mesh> mesh);
    
    static void loadOFF(const std::string& filename, std::shared_ptr<Mesh> mesh);

    const std::vector<Vertex>& vertexPositions() const { return _vertices; }
    const std::vector<glm::uvec3>& triangleIndices() const { return _indices; }
    
    void computeBoundingSphere(glm::vec3& center, float& radius);

public:
    std::vector<Vertex> _vertices;
    std::vector<glm::uvec3> _indices;
};

void loadOFF(const std::string &filename, std::shared_ptr<Mesh> mesh);