//
// Created by dallin on 3/5/23.
//

#include "Prepare.h"
#include "Utils.h"

namespace Prepare {
    std::vector<std::tuple<TessVertPtr, glm::vec3>> testing;
    OGLData toOGL(const SurfaceMeshPtr& sm) {
        OGLData data;
        data.vertices.reserve(sm->number_of_vertices());
        data.faces.reserve(sm->number_of_faces());
        data.edgeVertexData = {};
        data.cornerVertexData = {};
        data.innerVertexData = {};
        data.faceData = {};

        std::unordered_map<SM_vertex_descriptor, unsigned int> vdToIdx;

        //Process vertices,
        for (const SM_vertex_descriptor& vd : sm->vertices()) {
            vdToIdx.insert({vd, data.vertices.size()});
            glm::vec3 pos = Utils::toGLM(sm->point(vd));
            data.vertices.emplace_back(pos);
        }

        //Process faces
        for (const SM_face_descriptor& fd : sm->faces()) {
            std::array<unsigned int, 3> indices{};
            unsigned int count = 0;
            for (const auto& vd : sm->vertices_around_face(sm->halfedge(fd))) {
                indices.at(count++) = vdToIdx.at(vd);
            }
            data.faces.emplace_back(indices.at(0), indices.at(1), indices.at(2));
        }

        return data;
    }

    OGLData toOGL(std::vector<TessLevelData>& meshes) {
        auto sm = meshes.at(0).mesh;
        auto data = toOGL(sm);
        MultiResStorage::assignAttributes(meshes, data);

        return data;
    }
}

