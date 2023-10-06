//
// Created by dallin on 10/4/23.
//

#ifndef TESSELLATION_MULTIRESSTORAGE_H
#define TESSELLATION_MULTIRESSTORAGE_H

#include <glm/glm.hpp>
#include <vector>
#include "MeshImpl.h"

namespace MultiResStorage {
    struct VertexAttributes {
        glm::vec3 displacement;
        glm::vec3 normal;
        VertexAttributes(glm::vec3 d, glm::vec3 n): displacement(d), normal(n) {}
        VertexAttributes(): displacement(), normal() {}
    };

    struct FaceLookupIndices {
        int firstCorner0, firstCorner1, firstCorner2;
        int firstInner;
        int firstEdge0, firstEdge1, firstEdge2;

        FaceLookupIndices(const std::array<int,3>& corners, int inner, const std::array<int,3> edges) {
            firstCorner0 = corners[0], firstCorner1 = corners[1], firstCorner2 = corners[2];
            firstInner = inner;
            firstEdge0 = edges[0], firstEdge1 = edges[1], firstEdge2 = edges[2];
        }
    };

    typedef std::vector<VertexAttributes> VAttrs;
    typedef std::vector<FaceLookupIndices> FIdxs;

    void assignAttributes(std::vector<TessLevelData>& meshes, VAttrs& corner, VAttrs& edges, VAttrs& inner, FIdxs& faces);
};


#endif //TESSELLATION_MULTIRESSTORAGE_H
