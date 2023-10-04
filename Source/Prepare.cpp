//
// Created by dallin on 3/5/23.
//

#include "Prepare.h"
#include "Utils.h"

namespace Prepare {
    std::vector<std::tuple<TessVertPtr, glm::vec3>> testing;
    OGLData toOGL(const SurfaceMesh& sm) {
        OGLData data;
        data.vertices.reserve(sm.number_of_vertices());
        data.faces.reserve(sm.number_of_faces());

        std::unordered_map<SM_halfedge_descriptor, unsigned int> hdToVertIdx;

        Seam_vertex_pmap vSeamMap = sm.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        Seam_edge_pmap eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        Normal_vertex_pmap vNormMap = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;

        auto [vColorMap, hasColor] = sm.property_map<SM_vertex_descriptor, glm::vec3>("v:color");
        auto [uvMap, hasUVMap] = sm.property_map<SM_halfedge_descriptor, Point_2>("h:uv");

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

                        glm::vec3 color = {0.4f, 0.4f, 0.4f};
//                        if (hasColor) {
//                            color = get(vColorMap, vd);
//                        }
                        data.vertices.emplace_back(pos, nrm, color, texCoords);
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

                        glm::vec3 pos = Utils::toGLM(sm.point(vd));
                        glm::vec3 nrm = Utils::toGLM(get(vNormMap, vd));
                        glm::vec2 texCoords = {};
                        if (hasUVMap) {
                            Point_2 uv = get(uvMap, hd);
                            texCoords = Utils::toGLM(uv);
                        }
                        glm::vec3 color = {0.4f, 0.4f, 0.4f};
//                        if (hasColor) {
//                            color = get(vColorMap, vd);
//                        }
                        data.vertices.emplace_back(pos, nrm, color, texCoords);
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
        if (get(eSeamMap, edge->ed) || edgeSet.find(edge->ed) == edgeSet.end()) {
            for (auto& edgeVert : edge->tessVerts) {
                auto bary = edgeVert->baryCoords;
                auto uv = Utils::toGLM(uv_a) * bary.x + Utils::toGLM(uv_b) * (1.0f - bary.x);
                auto displace = edgeVert->newCoords - edgeVert->origCoords;
                if (bary.x > 0.9 || bary.x < 0.1) {
                    int x = 1;
                }
                if (displace.x < 0.00001 && displace.y < 0.00001 && displace.z < 0.00001) {
                    int x = 1;
                }
                if (glm::length(displace) > 60.0f) {
                    testing.emplace_back(edgeVert, displace);
                }
                texCoordVals.emplace_back(uv, displace);
            }

            //TODO: THis will probably cause problems on seams, so we may need to adjust this slightly.
//            auto bary = edge->v0->baryCoords;
//            auto uv = Utils::toGLM(uv_a) * bary.x + Utils::toGLM(uv_b) * (1.0f - bary.x);
//            auto displace = edge->v0->newCoords - edge->v0->origCoords;
//            texCoordVals.emplace_back(uv, displace);
//
//            bary = edge->v1->baryCoords;
//            uv = Utils::toGLM(uv_a) * bary.x + Utils::toGLM(uv_b) * (1.0f - bary.x);
//            displace = edge->v1->newCoords - edge->v1->origCoords;
//            texCoordVals.emplace_back(uv, displace);

            edgeSet.insert(edge->ed);
        }
        else {
//            std::cout << "Edge not on seam or is in set.\n";
        }
    }

    std::pair<float, float> findPPMOffset(const std::vector<std::pair<glm::vec2, glm::vec3>>& texCoordVals) {
        float offsetVal = std::numeric_limits<float>::max();
        float maxVal = std::numeric_limits<float>::min();
        for (auto& pair : texCoordVals) {
            glm::vec3 val = pair.second;
            if (val.r < offsetVal) offsetVal = val.r;
            if (val.g < offsetVal) offsetVal = val.g;
            if (val.b < offsetVal) offsetVal = val.b;

            if (val.r > maxVal) maxVal = val.r;
            if (val.g > maxVal) maxVal = val.g;
            if (val.b > maxVal) maxVal = val.b;
        }

        offsetVal = abs(offsetVal);
        return {offsetVal, maxVal + offsetVal};
    }

    void createTextures(const SurfaceMeshPtr& sm, const TessLevelData& data, int& w, int& h,
                        std::vector<glm::vec3>& displaceTex, std::vector<glm::vec3>& normalTex) {
        auto tess_mesh = data.mesh;

        typedef std::vector<std::tuple<glm::vec2, glm::vec3, glm::vec3>> UV_Tex_Vals;
        UV_Tex_Vals texCoordVals;
        std::unordered_map<SM_vertex_descriptor, UV_Tex_Vals> texCoordValsMap{};
        EdgeSet processedEdges;
        auto eSeamMap = tess_mesh->property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        auto vSeamMap = tess_mesh->property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto uvmap = tess_mesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto vNorms = tess_mesh->property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        std::unordered_set<SM_vertex_descriptor> processedVerts;

        unsigned numTexCoords = 0;

        if (tess_mesh == sm) {
            for (const auto& vertex : tess_mesh->vertices()) {
                texCoordValsMap.insert({vertex, {}});
                auto& texCoordVec = texCoordValsMap.at(vertex);

                if (get(vSeamMap, vertex)) {
                    for (const auto& hd: halfedges_around_source(vertex, *tess_mesh)) {
                        auto uv = get(uvmap, hd);
                        auto it = std::find_if(texCoordVec.begin(), texCoordVec.end(), [&](const auto &tup) {
                            return glm::all(glm::lessThanEqual(glm::abs(Utils::toGLM(uv) - std::get<0>(tup)),
                                                               {0.00001, 0.00001}));
                        });

                        if (it == std::end(texCoordVec)) {
                            auto nrm = Utils::toGLM(get(vNorms, vertex));
                            texCoordVec.emplace_back(Utils::toGLM(uv), glm::vec3(0.0f, 0.0f, 1.0f),
                                                     glm::normalize(nrm));
                            numTexCoords++;
                        }
                    }
                } else {
                    auto nrm = Utils::toGLM(get(vNorms, vertex));
                    auto uv = get(uvmap, tess_mesh->opposite(tess_mesh->halfedge(vertex)));
                    texCoordVec.emplace_back(Utils::toGLM(uv), glm::vec3(0.0f, 0.0f, 1.0f), nrm);
                    numTexCoords++;
                }
            }
        }
        else {
            for (const auto& [fd, face] : data.processedFaces) {
                for (const auto& tessVert : face->tessVerts) {
                    if (processedVerts.find(tessVert->vd) != processedVerts.end()) {
                        continue;
                    }

                    if (texCoordValsMap.find(tessVert->vd) == texCoordValsMap.end()) {
                        texCoordValsMap.insert({tessVert->vd, {}});
                    }
                    auto& texCoordVec = texCoordValsMap.at(tessVert->vd);
                    for (const auto& hd : halfedges_around_source(tessVert->vd, *data.mesh)) {
                        auto uv = get(uvmap, hd);
                        auto it = std::find_if(texCoordVec.begin(), texCoordVec.end(), [&](const auto& tup) {
                            return glm::all(glm::lessThanEqual(glm::abs(Utils::toGLM(uv) - std::get<0>(tup)), {0.00001, 0.00001}));
                        });

                        if (it == std::end(texCoordVec)) {
                            uv = get(uvmap, hd);
                            auto nrm = Utils::toGLM(get(vNorms, tess_mesh->source(hd)));
                            auto displace = tessVert->newCoords - tessVert->origCoords;
                            texCoordVec.emplace_back(Utils::toGLM(uv), displace, glm::normalize(nrm));
                            numTexCoords++;
                        }
                        processedVerts.insert(tessVert->vd);
                    }
                }
            }
        }

        texCoordVals.reserve(numTexCoords);
        for (const auto& [vd, vec] : texCoordValsMap) {
            texCoordVals.insert(texCoordVals.end(), vec.begin(), vec.end());
        }

        int overlapCount;
        std::unordered_set<int> overlaps;
        float maxPercentOverlap = 0.005;
        int resolution = 64;

        std::unordered_map<int, std::unordered_set<int>> usedPixels = {};
        std::vector<std::pair<int, int>> usedIndices = {};

        for (; resolution <= 4096; resolution *= 2) {
            std::cout << "Generating Texture with resolution: " << resolution << "\n";
            w = resolution;
            h = resolution;
            overlapCount = 0;
            overlaps.clear();
            usedPixels.clear();
            usedIndices.clear();

            displaceTex.clear();
            displaceTex.reserve(resolution * resolution);
            normalTex.clear();
            normalTex.reserve(resolution * resolution);
            for (int i = 0; i < resolution * resolution; i++) {
                displaceTex.emplace_back(0.0f, 0.0f, 0.0f);
                normalTex.emplace_back(0.0f, 0.0f, 0.0f);
            }

            bool increaseResolution = false;
            for (auto &tup: texCoordVals) {
                glm::vec2 uv = std::get<0>(tup);
                glm::vec3 displaceVal = std::get<1>(tup);
                glm::vec3 normalVal = std::get<2>(tup);

                int row, col;
                col = (int) trunc(uv.x * float(resolution));
                row = (int) trunc(uv.y * float(resolution));

                {
                    //Add to usedPixels for use in blur algorithm
                    if (usedPixels.find(row) == usedPixels.end()) {
                        usedPixels.insert({row, {}});
                    }
                    usedPixels.at(row).insert(col);
                }

                if (displaceTex[row * resolution + col] != glm::vec3(0.0f, 0.0f, 0.0f) ||
                    normalTex[row * resolution + col] != glm::vec3(0.0f, 0.0f, 0.0f)) {
                    overlapCount++;
                    overlaps.insert((row * resolution) + col);
                    usedIndices.emplace_back(row, col);
                    if ((float)overlapCount / (float)numTexCoords > maxPercentOverlap) {
                        increaseResolution = true;
                        break;
                    }
                } else {
                    displaceTex[row * resolution + col] = displaceVal;
                    normalTex[row * resolution + col] = normalVal;
                }
            }

            if (!increaseResolution) {
                break;
            }
            std::cout << "Increasing resolution..." << "\n\n";
        }

        std::cout << "Number of overlaps during texGen: " << overlapCount << "\n";
        std::cout << "Resetting overlaps." << "\n";
        for (const auto index : overlaps) {
            displaceTex[index] = {};
            normalTex[index] = {};
        }

//        for (const auto& [row, col] : usedIndices) {
//            auto pos = usedPixels.at(row).find(col);
//            if (pos != usedPixels.at(row).end()) {
//                usedPixels.at(row).erase(pos);
//            }
//        }
//
//        std::stringstream ss;
//        ss << "{";
//        int numRows = usedPixels.size();
//        int rowCount = 1;
//        for (const auto& [row, cols] : usedPixels) {
//            ss << row << ":{" ;
//            int numCols = cols.size();
//            int colCount = 1;
//            for (const auto& col : cols) {
//                ss << col << ":True";
//                if (colCount++ < numCols) {
//                    ss << ",";
//                }
//            }
//            ss << "}";
//            if (rowCount++ < numRows) {
//                ss << ",";
//            }
//        }
//        ss << "}";
//        std::cout << ss.str() << std::endl;
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

