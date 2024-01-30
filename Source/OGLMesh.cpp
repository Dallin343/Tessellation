//
// Created by dallin on 3/7/23.
//

#include "OGLMesh.h"
#include <glad/glad.h>

OGLMesh::OGLMesh(const OGLData& meshData, bool multiRes): multiRes(multiRes) {
    this->vertices = meshData.vertices;
    if (multiRes) {
        for (const auto& v : this->vertices) {
            this->vertexPositions.push_back(v.pos);
        }
    }
    this->faces = meshData.faces;
    this->cornerVertexData = meshData.cornerVertexData;
    this->edgeVertexData = meshData.edgeVertexData;
    this->innerVertexData = meshData.innerVertexData;
    this->faceData = meshData.faceData;
    this->indices.reserve(this->faces.size() * 3);
    for (const auto& face: this->faces) {
        this->indices.push_back(face.v0);
        this->indices.push_back(face.v1);
        this->indices.push_back(face.v2);
    }
}

void OGLMesh::genMultiResBuffers() {
    glGenBuffers(1, &cornerBuffer);
    glGenBuffers(1, &edgeBuffer);
    glGenBuffers(1, &innerBuffer);
    glGenBuffers(1, &faceBuffer);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, cornerBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, cornerVertexData.size(), &cornerVertexData[0], GL_STATIC_DRAW);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, edgeBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, edgeVertexData.size(), &edgeVertexData[0], GL_STATIC_DRAW);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, innerBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, innerVertexData.size(), &innerVertexData[0], GL_STATIC_DRAW);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, faceBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, faceData.size(), &faceData[0], GL_STATIC_DRAW);
}

void OGLMesh::setupMesh() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    if (multiRes) {
        genMultiResBuffers();
    }

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    if (multiRes) {
        glBufferData(GL_ARRAY_BUFFER, vertexPositions.size() * sizeof(glm::vec3), &vertexPositions[0], GL_STATIC_DRAW);
    }
    else {
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(VertexData), &vertices[0], GL_STATIC_DRAW);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 &indices[0], GL_STATIC_DRAW);

    if (multiRes) {
        // vertex positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
    }
    else {
        // vertex positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), nullptr);
        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), (void*)offsetof(VertexData, normal));
    }

    glBindVertexArray(0);
}

void OGLMesh::draw(const Shader &shader, Type drawType) const {
    if (multiRes) {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, cornerBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, edgeBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, innerBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, faceBuffer);
    }
    glBindVertexArray(VAO);
    if (drawType == Patches) {
        glDrawElements(GL_PATCHES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    } else if (drawType == Triangles) {
        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, nullptr);
    }

    glBindVertexArray(0);
}


