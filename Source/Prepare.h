//
// Created by dallin on 3/5/23.
//

#ifndef TESSELLATION_PREPARE_H
#define TESSELLATION_PREPARE_H

#include "MeshImpl.h"

namespace Prepare {
    typedef std::vector<std::pair<glm::vec2, glm::vec3>> TexCoordVals;
    typedef std::unordered_set<SM_edge_descriptor> EdgeSet;

    struct VertexData {
        VertexData(const glm::vec3 &pos, const glm::vec3 &nrm, const glm::vec2 &texCoords) : pos(pos), nrm(nrm),
                                                                                                texCoords(texCoords) {}

        glm::vec3 pos;
        glm::vec3 nrm;
        glm::vec2 texCoords;
    };

    struct FaceData {
        FaceData(unsigned int v0, unsigned int v1, unsigned int v2) : v0(v0), v1(v1), v2(v2) {}

        unsigned int v0, v1, v2;
    };

    struct OGLData {
        std::vector<VertexData> vertices;
        std::vector<FaceData> faces;
    };

    OGLData toOGL(const SurfaceMesh& sm);

    std::vector<glm::vec3> createTexture(const SurfaceMesh& sm, ProcessFaceMap& processedFaces, int w, int h, float& offset, int& max);
//    void processEdge(const ProcessEdgePtr& edge, const );
    ProcessEdgePtr findEdge(const ProcessFacePtr& face, const TessVertPtr& tessVert);
    glm::vec2 findMinDiff(TexCoordVals& texCoordVals);
};


#endif //TESSELLATION_PREPARE_H
