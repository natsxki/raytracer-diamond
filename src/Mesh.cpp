#include "Mesh.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

//OBJ Loader
void Mesh::loadOBJ(const std::string& filename, std::shared_ptr<Mesh> mesh) {
    std::ifstream in(filename.c_str());
    if (!in) {
        throw std::runtime_error("Failed to open OBJ file: " + filename);
    }

    mesh->clear();
    std::vector<glm::vec3> temp_positions;
    std::string line;

    while (std::getline(in, line)) {
        if (line.substr(0, 2) == "v ") {
            std::istringstream s(line.substr(2));
            glm::vec3 v;
            s >> v.x >> v.y >> v.z;
            temp_positions.push_back(v);
        }
        else if (line.substr(0, 2) == "f ") {
            std::istringstream s(line.substr(2));
            std::string segment;
            std::vector<unsigned int> faceIndices;
            
            while (std::getline(s, segment, ' ')) {
                if (segment.empty()) continue;
                std::stringstream ss(segment);
                std::string indexStr;
                std::getline(ss, indexStr, '/'); 
                if(!indexStr.empty()) {
                    faceIndices.push_back(std::stoi(indexStr) - 1); 
                }
            }

            for (size_t i = 1; i < faceIndices.size() - 1; i++) {
                glm::uvec3 tri;
                tri.x = faceIndices[0];
                tri.y = faceIndices[i];
                tri.z = faceIndices[i + 1];
                mesh->_indices.push_back(tri);
            }
        }
    }

    for (const auto& pos : temp_positions) {
        Vertex v;
        v.position = pos;
        v.normal = glm::vec3(0,0,0); // normals if needed
        mesh->_vertices.push_back(v);
    }

    for (auto& v : mesh->_vertices) v.normal = glm::vec3(0.0f);
    
    // accumulate face normals
    for (const auto& tri : mesh->_indices) {
        glm::vec3 v0 = mesh->_vertices[tri.x].position;
        glm::vec3 v1 = mesh->_vertices[tri.y].position;
        glm::vec3 v2 = mesh->_vertices[tri.z].position;
        glm::vec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        
        mesh->_vertices[tri.x].normal += normal;
        mesh->_vertices[tri.y].normal += normal;
        mesh->_vertices[tri.z].normal += normal;
    }
    
    // normalize
    for (auto& v : mesh->_vertices) v.normal = glm::normalize(v.normal);

    std::cout << "[Mesh] Loaded OBJ " << filename << ": " << mesh->_vertices.size() << " vertices, " << mesh->_indices.size() << " triangles." << std::endl;
}

void Mesh::init() {
    clear();
}

void Mesh::clear() {
    _vertices.clear();
    _indices.clear();
}

void Mesh::computeBoundingSphere(glm::vec3& center, float& radius) {
    if (_vertices.empty()) {
        center = glm::vec3(0.0f);
        radius = 0.0f;
        return;
    }

    glm::vec3 minAABB(1e30f);
    glm::vec3 maxAABB(-1e30f);

    for (const auto& v : _vertices) {
        minAABB = glm::min(minAABB, v.position);
        maxAABB = glm::max(maxAABB, v.position);
    }

    center = (minAABB + maxAABB) * 0.5f;
    radius = glm::length(maxAABB - minAABB) * 0.5f;
}


void loadOFF(const std::string &filename, std::shared_ptr<Mesh> mesh) {
}

void Mesh::loadOFF(const std::string& filename, std::shared_ptr<Mesh> mesh) {
    ::loadOFF(filename, mesh);
}