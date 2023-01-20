//
// Created by Dallin Hagman on 2/16/22.
//

#ifndef TESSELLATION_MESH_H
#define TESSELLATION_MESH_H

#include <glad/glad.h> // holds all OpenGL type declarations

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Shader.h"
#include "Primitives.h"

#include <string>
#include <vector>


class Mesh {
public:
    // mesh data
    Mesh(VBuf vertices, std::vector<unsigned int> indices, EBuf edges, FBuf faces);
    void Draw(Shader &shader);

    const VBuf &GetVertices() const;

    const EBuf &GetEdges() const;

    const FBuf &GetFaces() const;

    void ToGraph();
private:
    //  render data
    unsigned int VAO, VBO, EBO;

    void setupMesh();

    std::vector<unsigned int> indices;
    VBuf vertices;
    EBuf edges;
    FBuf faces;
};


#endif //TESSELLATION_MESH_H
