//
// Created by dallin on 3/7/23.
//

#include "OGLMesh.h"
#include <glad/glad.h>

OGLMesh::OGLMesh(const Prepare::OGLData& meshData) {
    this->vertices = meshData.vertices;
    this->faces = meshData.faces;
    this->indices.reserve(this->faces.size() * 3);
    for (const auto& face: this->faces) {
        this->indices.push_back(face.v0);
        this->indices.push_back(face.v1);
        this->indices.push_back(face.v2);
    }
    this->setupMesh();
}

void OGLMesh::setupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Prepare::VertexData), &vertices[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 &indices[0], GL_STATIC_DRAW);

    // vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Prepare::VertexData), nullptr);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Prepare::VertexData), (void*)offsetof(Prepare::VertexData, nrm));
//    // vertex texCoords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Prepare::VertexData), (void*)offsetof(Prepare::VertexData, texCoords));

    glBindVertexArray(0);
}

void OGLMesh::draw(const Shader &shader) const {
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}


