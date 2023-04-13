//
// Created by dallin on 3/5/23.
//

#include "Prepare.h"
#include "Utils.h"

namespace Prepare {
    OGLData toOGL(const SurfaceMesh& sm) {
        OGLData data;
        data.vertices.reserve(sm.number_of_vertices());
        data.faces.reserve(sm.number_of_faces());

        std::unordered_map<SM_halfedge_descriptor, unsigned int> hdToVertIdx;

        Seam_vertex_pmap vSeamMap = sm.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        Seam_edge_pmap eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        Normal_vertex_pmap vNormMap = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        UV_pmap uvMap = sm.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

        //Process vertices,
        for (const SM_vertex_descriptor& vd : sm.vertices()) {
            if (get(vSeamMap, vd)) {
                std::vector<std::pair<Point_2, unsigned int>> uvs;
                for (const SM_halfedge_descriptor& hd : halfedges_around_source(vd, sm)) {
                    if (get(eSeamMap, sm.edge(hd))) {
                        Point_2 uv = get(uvMap, hd);
                        unsigned int idx = data.vertices.size();
                        hdToVertIdx.insert({hd, idx});
                        uvs.emplace_back(uv, idx);

                        glm::vec3 pos = Utils::toGLM(sm.point(vd));
                        glm::vec3 nrm = Utils::toGLM(get(vNormMap, vd));
                        glm::vec2 texCoords = Utils::toGLM(uv);
                        data.vertices.emplace_back(pos, nrm, texCoords);
                    }
                }

                for (const SM_halfedge_descriptor& hd : halfedges_around_source(vd, sm)) {
                    if (!get(eSeamMap, sm.edge(hd))) {
                        Point_2 uv = get(uvMap, hd);
                        auto it = std::min_element(uvs.begin(), uvs.end(), [uv](auto a, auto b) {
                            return CGAL::squared_distance(a.first, uv) < CGAL::squared_distance(b.first, uv);
                        });

                        unsigned int idx = (*it).second;
                        hdToVertIdx.insert({hd, idx});
                    }
                }
            }
            else {
                unsigned int idx = data.vertices.size();
                bool first = true;
                for (const SM_halfedge_descriptor& hd : halfedges_around_source(vd, sm)) {
                    if (first) {
                        first = false;
                        Point_2 uv = get(uvMap, hd);

                        glm::vec3 pos = Utils::toGLM(sm.point(vd));
                        glm::vec3 nrm = Utils::toGLM(get(vNormMap, vd));
                        glm::vec2 texCoords = Utils::toGLM(uv);
                        data.vertices.emplace_back(pos, nrm, texCoords);
                    }
                    hdToVertIdx.insert({hd, idx});
                }
            }
        }

        //Process faces
        for (const SM_face_descriptor& fd : sm.faces()) {
            std::array<unsigned int, 3> indices{};
            unsigned int count = 0;
            for (const SM_halfedge_descriptor& hd : sm.halfedges_around_face(sm.halfedge(fd))) {
                indices.at(count++) = hdToVertIdx.at(hd);
            }
            data.faces.emplace_back(indices.at(0), indices.at(1), indices.at(2));
        }

        return data;
    }

    std::pair<Point_2, Point_2> matchUV(const ProcessEdgePtr& edge, const ProcessFacePtr& face) {
        Point_2 uv_a, uv_b;
        if (edge->v0->vd == face->vds.at(0)) {
            uv_a = face->uvs.at(0);
        } else if (edge->v0->vd == face->vds.at(1)) {
            uv_a = face->uvs.at(1);
        } else if (edge->v0->vd == face->vds.at(2)) {
            uv_a = face->uvs.at(2);
        }

        if (edge->v1->vd == face->vds.at(0)) {
            uv_b = face->uvs.at(0);
        } else if (edge->v1->vd == face->vds.at(1)) {
            uv_b = face->uvs.at(1);
        } else if (edge->v1->vd == face->vds.at(2)) {
            uv_b = face->uvs.at(2);
        }
        return std::make_pair(uv_a, uv_b);
    }

    void processEdge(const ProcessEdgePtr& edge, const ProcessFacePtr& face, const Seam_edge_pmap& eSeamMap,
                     EdgeSet& edgeSet, TexCoordVals& texCoordVals) {
        Point_2 uv_a, uv_b;
        std::tie(uv_a, uv_b) = matchUV(edge, face);
        if (get(eSeamMap, edge->ed) || edgeSet.find(edge->ed) != edgeSet.end()) {
            for (auto& edgeVert : edge->tessVerts) {
                auto bary = edgeVert->baryCoords;
                auto uv = Utils::toGLM(uv_a) * bary.x + Utils::toGLM(uv_b) * (1.0f - bary.x);
                auto displace = edgeVert->newCoords - edgeVert->origCoords;
                texCoordVals.emplace_back(uv, displace);
            }
            edgeSet.insert(edge->ed);
        }
    }

    std::vector<glm::vec3> createTexture(const SurfaceMesh& sm, ProcessFaceMap& processedFaces, int w, int h, int& max) {
        std::vector<std::pair<glm::vec2, glm::vec3>> texCoordVals;
        EdgeSet processedEdges;
        auto eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;


        for (const auto& fd: sm.faces()) {
            ProcessFacePtr face = processedFaces.at(fd);
            auto [uv0, uv1, uv2] = face->uvs;

            //Create texCoords for inner vertices
            for (auto& innerVert : face->innerVerts) {
                auto bary = innerVert->baryCoords;
                auto uv = Utils::toGLM(uv0) * bary.x + Utils::toGLM(uv1) * bary.y + Utils::toGLM(uv2) * bary.z;
                auto displace = innerVert->newCoords - innerVert->origCoords;
                texCoordVals.emplace_back(uv, displace);
            }

            //Create texCoords for edge vertices, checking if they are on a seam.
            processEdge(face->e01, face, eSeamMap, processedEdges, texCoordVals);
            processEdge(face->e12, face, eSeamMap, processedEdges, texCoordVals);
            processEdge(face->e02, face, eSeamMap, processedEdges, texCoordVals);
        }

        //Find the minimum difference between values
        //glm::vec2 minDiff = findMinDiff(texCoordVals);
        std::vector<glm::vec3> tex;
        tex.reserve(w * h);
        for (int i = 0; i < w * h; i++) {
            tex.emplace_back(0.0f, 0.0f, 0.0f);
        }

        max = 0;
        int issues = 0;
        for (auto& pair : texCoordVals) {
            glm::vec2 uv = pair.first;
            glm::vec3 val = pair.second;

            int row, col;
            col = (int)trunc(uv.x * float(h));
            row = (int)trunc(uv.y * float(w));
            if (tex[row * h + col] != glm::vec3(0.0f, 0.0f, 0.0f)) {
                issues++;
            } else {
                if (ceil(val.r) > max) {
                    max = (int)ceil(val.r);
                }
                if (ceil(val.g) > max) {
                    max = (int)ceil(val.g);
                }
                if (ceil(val.b) > max) {
                    max = (int)ceil(val.b);
                }
                tex[row * h + col] = val;
            }
        }
        std::cout << "Overlap during texGen: " << issues << "\n";

        return tex;
    }

    ProcessEdgePtr findEdge(const ProcessFacePtr& face, const TessVertPtr& tessVert) {
        auto it01 = std::find(face->e01->tessVerts.begin(), face->e01->tessVerts.end(), tessVert);
        if (it01 != face->e01->tessVerts.end()) {
            return face->e01;
        }

        auto it12 = std::find(face->e12->tessVerts.begin(), face->e12->tessVerts.end(), tessVert);
        if (it12 != face->e12->tessVerts.end()) {
            return face->e12;
        }

        auto it02 = std::find(face->e02->tessVerts.begin(), face->e02->tessVerts.end(), tessVert);
        if (it02 != face->e02->tessVerts.end()) {
            return face->e02;
        }
    }

    glm::vec2 findMinDiff(TexCoordVals& texCoordVals) {
        auto maxFloat = std::numeric_limits<float>::max();
        glm::vec2 diff = {maxFloat, maxFloat};

        for (int i = 0;  i < texCoordVals.size(); i++) {
            for (int j = 0; j < texCoordVals.size(); j++) {
                if (i == j) {
                    continue;
                }
                auto a = texCoordVals.at(i).first;
                auto b = texCoordVals.at(j).first;
                if (abs(a.x - b.x) < diff.x) {
                    diff.x = abs(a.x - b.x);
                }
                if (abs(a.y - b.y) < diff.y) {
                    diff.y = abs(a.y - b.y);
                }
            }
        }
        return diff;
    }
}

