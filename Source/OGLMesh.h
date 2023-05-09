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

    OGLMesh(const Prepare::OGLData& meshData);
    void setupMesh();
    void draw(const Shader& shader, Type drawType) const;

    unsigned int VAO, VBO, EBO;

    std::vector<Prepare::VertexData> vertices;
    std::vector<unsigned int> indices;
    std::vector<Prepare::FaceData> faces;
};


#endif //TESSELLATION_OGLMESH_H
