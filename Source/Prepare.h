//
// Created by dallin on 3/5/23.
//

#ifndef TESSELLATION_PREPARE_H
#define TESSELLATION_PREPARE_H

#include "MeshImpl.h"
#include "MultiResStorage.h"

namespace Prepare {
    typedef std::vector<std::pair<glm::vec2, glm::vec3>> TexCoordVals;

    struct VertexData {
        VertexData(const glm::vec3 &pos) : pos(pos), normal() {}
        VertexData(const glm::vec3 &pos, const glm::vec3& norm) : pos(pos), normal(norm) {}

        glm::vec3 pos;
        glm::vec3 normal;
    };

    struct FaceData {
        FaceData(unsigned int v0, unsigned int v1, unsigned int v2) : v0(v0), v1(v1), v2(v2) {}

        unsigned int v0, v1, v2;
    };

    struct OGLData {
        std::vector<VertexData> vertices;
        std::vector<FaceData> faces;
        MultiResStorage::VAttrs cornerVertexData;
        MultiResStorage::VAttrs edgeVertexData;
        MultiResStorage::VAttrs innerVertexData;
        MultiResStorage::FIdxs faceData;
    };

    OGLData toOGL(const SurfaceMeshPtr& sm);
    OGLData toOGL(std::vector<TessLevelData>& meshes);
};


#endif //TESSELLATION_PREPARE_H
