//
// Created by dallin on 3/7/23.
//

#ifndef TESSELLATION_OGLMESH_H
#define TESSELLATION_OGLMESH_H


#include <vector>
#include "Prepare.h"
#include "Shader.h"

class OGLMesh {
public:
    enum Type {
        Patches,
        Triangles
    };

    OGLMesh(const Prepare::OGLData& meshData, bool multiRes = false);
    void genMultiResBuffers();
    void setupMesh();
    void draw(const Shader& shader, Type drawType) const;

    unsigned int VAO, VBO, EBO;
    unsigned int cornerBuffer, edgeBuffer, innerBuffer, faceBuffer;

    bool multiRes;

    std::vector<Prepare::VertexData> vertices;
    std::vector<glm::vec3> vertexPositions;
    std::vector<unsigned int> indices;
    std::vector<Prepare::FaceData> faces;
    MultiResStorage::VAttrs cornerVertexData;
    MultiResStorage::VAttrs edgeVertexData;
    MultiResStorage::VAttrs innerVertexData;
    MultiResStorage::FIdxs faceData;
};


#endif //TESSELLATION_OGLMESH_H
