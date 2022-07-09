//
// Created by Dallin Hagman on 2/16/22.
//

#include "Mesh.h"
#include <unordered_set>
#include <utility>

Mesh::Mesh(VBuf vertices, std::vector<unsigned int> indices, EBuf edges, FBuf faces)
{
    this->vertices = std::move(vertices);
    this->indices = std::move(indices);
    this->edges = std::move(edges);
    this->faces = std::move(faces);

    setupMesh();
}

void Mesh::setupMesh()
{
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(VertexData), &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 &indices[0], GL_STATIC_DRAW);

    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), nullptr);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, Normal));

    glBindVertexArray(0);
}

void Mesh::Draw(Shader &shader) {
    // draw mesh
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::ToGraph() {
//    std::vector<Edge> edges;
//    edges.reserve(this->vertices.size() * 2);
//
//    for (int i = 0; i < this->indices.size(); i += 3) {
//        // Iterate by triangle
//
//        auto i1 = this->indices.at(i);
//        auto i2 = this->indices.at(i+1);
//        auto i3 = this->indices.at(i+2);
//
//        // Create edges
//        edges.emplace_back(i1, i2);
//        edges.emplace_back(i3, i1);
//        edges.emplace_back(i1, i2);
//        edges.emplace_back(i3, i1);
//    }
}

const VBuf &Mesh::GetVertices() const {
    return vertices;
}
