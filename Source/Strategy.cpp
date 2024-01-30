//
// Created by dallin on 3/1/23.
//

#include "Strategy.h"
#include "Utils.h"
#include "Tessellator.h"
#include "Mapping.h"
#include <Hungarian.h>

namespace Strategy {
    void set_seam_maps(const SurfaceMesh& sm, const TessVertPtr& v0, const TessVertPtr& v1, const TessVertPtr& v2) {
        auto vSeamMap = sm.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;

        auto v0v1 = sm.edge(sm.halfedge(v0->vd, v1->vd));
        auto v1v2 = sm.edge(sm.halfedge(v1->vd, v2->vd));
        auto v2v0 = sm.edge(sm.halfedge(v2->vd, v0->vd));

        put(vSeamMap, v0->vd, v0->isOnSeam);
        put(vSeamMap, v1->vd, v1->isOnSeam);
        put(vSeamMap, v2->vd, v2->isOnSeam);

        if (v0->isOnSeam && v1->isOnSeam) {
            put(eSeamMap, v0v1, true);
        } else if (v1->isOnSeam && v2->isOnSeam) {
            put(eSeamMap, v1v2, true);
        } else if (v2->isOnSeam && v0->isOnSeam) {
            put(eSeamMap, v2v0, true);
        }
    }

    void tessellateFace(const ProcessFacePtr& processFace, SurfaceMesh& sm, const TessLevel& tL, TessEdgeMap& edgeVerts,
                        Stats& stats) {
        auto eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        auto vSeamMap = sm.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto uvmap = sm.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;
        auto removed = sm.property_map<SM_vertex_descriptor, bool>("v:removed").first;
        auto hd01 = sm.halfedge(processFace->vds.at(0), processFace->vds.at(1));
        auto hd12 = sm.halfedge(processFace->vds.at(2), processFace->vds.at(1));
        auto hd20 = sm.halfedge(processFace->vds.at(2), processFace->vds.at(0));
        auto e01hd = sm.edge(hd01);
        auto e02hd = sm.edge(hd20);
        auto e12hd = sm.edge(hd12);

        bool e01_exists = edgeVerts.find(e01hd) != edgeVerts.end();
        bool e02_exists = edgeVerts.find(e02hd) != edgeVerts.end();
        bool e12_exists = edgeVerts.find(e12hd) != edgeVerts.end();

        bool e01_is_seam = get(eSeamMap, e01hd);
        bool e02_is_seam = get(eSeamMap, e02hd);
        bool e12_is_seam = get(eSeamMap, e12hd);

        std::vector<TessVertPtr> e01Verts, e02Verts, e12Verts;

        if (e01_exists) {
            processFace->e01 = edgeVerts.at(e01hd);
            e01Verts = processFace->e01->tessVerts;
        } else {
            auto e01 = std::make_shared<ProcessEdge>(e01hd, hd01, sm.opposite(hd01));
            processFace->e01 = e01;
            edgeVerts.insert({e01hd, e01});
        }

        if (e02_exists) {
            processFace->e02 = edgeVerts.at(e02hd);
            e02Verts = processFace->e02->tessVerts;
        } else {
            auto e02 = std::make_shared<ProcessEdge>(e02hd, hd20, sm.opposite(hd20));
            processFace->e02 = e02;
            edgeVerts.insert({e02hd, e02});
        }

        if (e12_exists) {
            processFace->e12 = edgeVerts.at(e12hd);
            e12Verts = processFace->e12->tessVerts;
        } else {
            auto e12 = std::make_shared<ProcessEdge>(e12hd, hd12, sm.opposite(hd12));
            processFace->e12 = e12;
            edgeVerts.insert({e12hd, e12});
        }

        glm::vec3 t0 = Utils::toGLM(processFace->coords.at(0));
        glm::vec3 t1 = Utils::toGLM(processFace->coords.at(1));
        glm::vec3 t2 = Utils::toGLM(processFace->coords.at(2));
        auto uv0 = Utils::toGLM(processFace->uvs.at(0)); // (1, 0, 0)
        auto uv1 = Utils::toGLM(processFace->uvs.at(1)); // (0, 1, 0)
        auto uv2 = Utils::toGLM(processFace->uvs.at(2)); // (0, 0, 1)

        // find barycentric coords for tessellated vertices
        Triangle baryTri = {glm::vec3(1.0, 0.0, 0.0), glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 1.0)};
        auto tessTri = Tessellator::TessellateTriangle(baryTri, tL.ol0, tL.ol1, tL.ol2, tL.il);

        TessellatedVerts tessellatedVerts;
        tessellatedVerts.reserve(tessTri->vertices.size());
        std::unordered_map<unsigned int, SM_vertex_descriptor> tessIdxToVd;

        unsigned int idx = 0;
        for (auto& vert : tessTri->vertices) {
            auto bary = vert;
            glm::vec3 coord = bary.x * t0 + bary.y * t1 + bary.z * t2;
            TessVertPtr tessVert;
            if (bary.x == 1.0) {
                //At p0
                if (e01_exists) {
                    tessVert = processFace->e01->v0->vd == processFace->vds.at(0) ? processFace->e01->v0 : processFace->e01->v1;
                } else if (e02_exists) {
                    tessVert = processFace->e02->v0->vd == processFace->vds.at(0) ? processFace->e02->v0 : processFace->e02->v1;
                } else {
                    //Neither 0-1 or 0-2 exist yet, create the tessVert
                    tessVert = std::make_shared<TessellatedVert>(coord, bary);
                    tessVert->vd = processFace->vds.at(0);
                }

                if (!e01_exists) {
                    processFace->e01->v0 = tessVert;
                }
                if (!e02_exists) {
                    processFace->e02->v0 = tessVert;
                }

                tessIdxToVd.insert({idx, tessVert->vd});
            }
            else if (bary.y == 1.0) {
                //At p1
                if (e01_exists) {
                    tessVert = processFace->e01->v0->vd == processFace->vds.at(1) ? processFace->e01->v0 : processFace->e01->v1;
                } else if (e12_exists) {
                    tessVert = processFace->e12->v0->vd == processFace->vds.at(1) ? processFace->e12->v0 : processFace->e12->v1;
                } else {
                    //Neither 0-1 or 0-2 exist yet, create the tessVert
                    tessVert = std::make_shared<TessellatedVert>(coord, bary);
                    tessVert->vd = processFace->vds.at(1);
                }

                if (!e01_exists) {
                    processFace->e01->v1 = tessVert;
                }
                if (!e12_exists) {
                    processFace->e12->v0 = tessVert;
                }

                tessIdxToVd.insert({idx, tessVert->vd});
            }
            else if (bary.z == 1.0) {
                //At p2
                if (e02_exists) {
                    tessVert = processFace->e02->v0->vd == processFace->vds.at(2) ? processFace->e02->v0 : processFace->e02->v1;
                } else if (e12_exists) {
                    tessVert = processFace->e12->v0->vd == processFace->vds.at(2) ? processFace->e12->v0 : processFace->e12->v1;
                } else {
                    //Neither 0-1 or 0-2 exist yet, create the tessVert
                    tessVert = std::make_shared<TessellatedVert>(coord, bary);
                    tessVert->vd = processFace->vds.at(2);
                }

                if (!e02_exists) {
                    processFace->e02->v1 = tessVert;
                }
                if (!e12_exists) {
                    processFace->e12->v1 = tessVert;
                }

                tessIdxToVd.insert({idx, tessVert->vd});
            }
            else {
                // Nonexisting vertex, so lets create it.
                if (bary.x == 0.0) {
                    // On p1 - p2 edge
                    if (!e12_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, bary);
                        tessVert->vd = vd;
//                        if (e12_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e12->push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(1));
                        auto b = get(vNorms, processFace->vds.at(2));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.y));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
                        //Finds the given barycentric tessellated vert in the existing edges vector of tessVerts
//                        double testBary = processFace->e12->v0->vd == processFace->vds.at(1) ? bary.y : bary.z;
                        bool found = false;
                        for (auto& searchTessVert : processFace->e12->tessVerts) {
                            if (Utils::nearly_equal(searchTessVert->otherBaryCoords.x, bary.y)) {
                                tessVert = searchTessVert;
                                tessVert->otherBaryCoords = bary;
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            int x = 0;
                        }
//                        auto i = std::find_if(processFace->e12->tessVerts.begin(), processFace->e12->tessVerts.end(), [&](const TessVertPtr& a) {
//                            return glm::all(glm::equal(a->otherBaryCoords, bary));
//                        });
//                        tessVert = *i;
                    }

                }
                else if (bary.y == 0.0) {
                    // On p0 - p2 edge
                    if (!e02_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, bary);
                        tessVert->vd = vd;
//                        if (e02_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e02->push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(0));
                        auto b = get(vNorms, processFace->vds.at(2));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.x));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
//                        double testBary = processFace->e02->v0->vd == processFace->vds.at(0) ? bary.x : bary.z;
                        bool found = false;
                        for (auto& searchTessVert : processFace->e02->tessVerts) {
                            if (Utils::nearly_equal(searchTessVert->otherBaryCoords.x, bary.z)) {
                                tessVert = searchTessVert;
                                tessVert->otherBaryCoords = bary;
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            int x = 0;
                        }
//                        auto i = std::find_if(processFace->e02->tessVerts.begin(), processFace->e02->tessVerts.end(), [&](const TessVertPtr& a) {
//                            return glm::all(glm::equal(a->otherBaryCoords, bary));
//                        });
//                        tessVert = *i;
                    }
                }
                else if (bary.z == 0.0) {
                    // On p0 - p1 edge
                    if (!e01_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, bary);
                        tessVert->vd = vd;
//                        if (e01_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e01->push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(0));
                        auto b = get(vNorms, processFace->vds.at(1));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.x));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
//                        double testBary = processFace->e01->v0->vd == processFace->vds.at(0) ? bary.x : bary.y;
                        bool found = false;
                        for (auto& searchTessVert : processFace->e01->tessVerts) {
                            if (Utils::nearly_equal(searchTessVert->otherBaryCoords.x, bary.x)) {
                                tessVert = searchTessVert;
                                tessVert->otherBaryCoords = bary;
                                found = true;
                                break;
                            }
                        }
                        if (!found) {
                            int x = 0;
                        }
//                        auto i = std::find_if(processFace->e01->tessVerts.begin(), processFace->e01->tessVerts.end(), [&](const TessVertPtr& a) {
//                            return glm::all(glm::equal(a->otherBaryCoords, bary));
//                        });
//                        tessVert = *i;
                    }
                }
                else {
                    // Interior
                    auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                    tessVert = std::make_shared<TessellatedVert>(coord, bary);
                    tessVert->vd = vd;
                    tessVert->isInner = true;
                    processFace->innerVerts.push_back(tessVert);

                    auto a = get(vNorms, processFace->vds.at(0));
                    auto b = get(vNorms, processFace->vds.at(1));
                    auto c = get(vNorms, processFace->vds.at(2));
                    auto nrm = Utils::normalize(Utils::lerp(a, b, c, bary.x, bary.y, bary.z));
                    put(vNorms, vd, nrm);
                }
            }
            tessellatedVerts.push_back(tessVert);
            processFace->vdToTessVert.insert({tessVert->vd, tessVert});
        }
        processFace->tessVerts = tessellatedVerts;

        if (!e01_exists) processFace->e01->sort();
        if (!e02_exists) processFace->e02->sort();
        if (!e12_exists) processFace->e12->sort();

        //TODO: Maybe make new face normals based on vertex normals.
        //Add faces
        CGAL::Euler::remove_face(sm.halfedge(processFace->fd), sm);
        auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
        for (auto fIdx : tessTri->faces) {
            auto v0 = tessellatedVerts.at(fIdx.at(0));
            auto v1 = tessellatedVerts.at(fIdx.at(1));
            auto v2 = tessellatedVerts.at(fIdx.at(2));
            auto fd = sm.add_face(v0->vd, v1->vd, v2->vd);
            if (fd == SurfaceMesh::null_face()) {
                sm.add_face(v0->vd, v1->vd, v2->vd);
                stats.invalid_tessFaces++;
                continue;
            }

            set_seam_maps(sm, v0, v1, v2);
            auto tessellatedFace = std::make_shared<TessellatedFace>();
            tessellatedFace->fd = fd;
            tessellatedFace->vertices = {v0, v1, v2};
            processFace->tessFaces.push_back(tessellatedFace);

            //TODO: Maybe get rid of this
            //Calc face normal based on interpolated vertex normals;
            auto nrm = Utils::compute_face_normal(Utils::toPoint3(v0->origCoords), Utils::toPoint3(v1->origCoords), Utils::toPoint3(v2->origCoords));
            put(fNorms, fd, nrm);
        }

        stats.processed_edges = edgeVerts.size();
#if DRAW_STEPS || DRAW_T
        CGAL::draw(sm, "Tessellation");
#endif
    }

    void projectEdgeVerts(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh, Tree& aabbTree,
                          VertSet& interpolateVerts, TessEdgeMap& edgeVerts, Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

        //Project edges along normal to intersection
        for (const auto& vertex : processFace->tessVerts) {
            if (!vertex->isInner && !vertex->anchored && interpolateVerts.find(vertex) == interpolateVerts.end()) {
                Point_3 vPoint = sm.point(vertex->vd);
                Vector norm = get(vNorms, vertex->vd);
                auto projOnNormal = Utils::findIntersection(vPoint, norm, aabbTree, highResMesh);
                if (projOnNormal.has_value()) {
                    auto [point, nrm] = projOnNormal.value();
                    vertex->newCoords = Utils::toGLM(point);
                    vertex->anchored = true;
                    vertex->assignedBy = AssigningSection::ProjectEdge;
                    sm.point(vertex->vd) = point;
                    put(vNorms, vertex->vd, nrm);
                } else {
                    interpolateVerts.insert(vertex);
                }
            }
        }

        //TODO: Evaluate if actuatlly necessary
        for (const auto& tessFace : processFace->tessFaces) {
            auto fNorm = Utils::compute_face_normal(tessFace, sm);
            put(fNorms, tessFace->fd, fNorm);
        }
#if DRAW_STEPS || DRAW_EP
        CGAL::draw(sm, "Edge Projection");
#endif
    }

    FeatureVerts extractFeatureVertices(const SurfaceMesh& sm, const ProcessFacePtr& tri, const SurfaceMesh& highResMesh,
                                        Stats& stats) {
        auto fd = tri->fd;
        auto mapping = sm.property_map<SM_face_descriptor, std::vector<FaceMapping>>("f:feature_verts").first;
        auto featureVerts = get(mapping, fd);

        FeatureVerts foundVerts;
        foundVerts.reserve(featureVerts.size());

        for (const auto featVert : featureVerts) {
            auto hd = highResMesh.halfedge(featVert.vd);
            Point_3 point = highResMesh.point(SM_vertex_descriptor(featVert.vd));
            glm::vec3 vertex = {point.x(), point.y(), point.z()};
            auto baryCoords = Utils::align_barycentric(featVert.bary, tri->vds.at(0));

            FeatureVertPtr feature = std::make_shared<FeatureVert>(hd, vertex, baryCoords, glm::vec2());
            foundVerts.emplace_back(feature);
        }

        return foundVerts;
    }

    void minBiGraphMatch(const ProcessFacePtr& processFace, const FeatureVerts& featureVerts, Stats& stats) {
        int ol0 = 5, ol1 = 5, ol2 = 5, il = 5;

        auto t0 = Utils::toGLM(processFace->coords.at(0)); // (1, 0, 0)
        auto t1 = Utils::toGLM(processFace->coords.at(1)); // (0, 1, 0)
        auto t2 = Utils::toGLM(processFace->coords.at(2)); // (0, 0, 1)

        std::vector<glm::vec3> innerVertBaryCoords;
        innerVertBaryCoords.reserve(processFace->innerVerts.size());

        // save inner vertex bary coords
        for (const auto& innerVert : processFace->innerVerts) {
            innerVertBaryCoords.push_back(innerVert->baryCoords);
        }

        //Create Distance matrix for hungarian algo
        std::vector<std::vector<double>> distMatrix(innerVertBaryCoords.size());
        for (auto& col : distMatrix) {
            col.reserve(featureVerts.size());
        }

        //Generate edges between sides of the graph
        for (int i = 0; i < innerVertBaryCoords.size(); i++) {
            for (const auto& fVert : featureVerts) {
                auto tVert = innerVertBaryCoords.at(i);
                double t0t1 = glm::length(t1 - t0);
                double t0t2 = glm::length(t2 - t0);
                double t1t2 = glm::length(t2 - t1);
                double dist = Utils::barycentric_distance(tVert, fVert->baryCoords, t1t2, t0t2, t0t1);
                distMatrix.at(i).push_back(dist);
            }
        }

        HungarianAlgorithm hungarian;
        std::vector<int> assignment;
        hungarian.Solve(distMatrix, assignment);

        for (int i = 0; i < assignment.size(); i++) {
            auto assign = assignment.at(i);

            if (assign != -1) {
                processFace->innerVerts.at(i)->matchingFeature = featureVerts.at(assign);
            }
        }
    }

    bool checkFlips(const ProcessFacePtr& processFace, SurfaceMesh& sm, const SurfaceMesh& highResMesh,
                    const Gauss_vertex_pmap& gaussMap, FlipResult& flipped) {
        flipped.clear();
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

        for (const auto &tessFace : processFace->tessFaces) {
            const auto norm = get(fNorms, tessFace->fd);
            Vector newNorm = Utils::compute_face_normal(tessFace, sm);
            double dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

            if (dot < 0) {
                // TODO: Not sure about this approach
                for (const auto& tessVert : tessFace->vertices) {
                    if (tessVert->matchingFeature != nullptr) {
                        auto gaussian = get(gaussMap, highResMesh.source(tessVert->matchingFeature->hd));
                        flipped.emplace(gaussian, tessVert);
                    }
                }
            }
        }
        return !flipped.empty();
    }

    void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh,
                         const Gauss_vertex_pmap& gaussMap, Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;


        glm::vec3 t0 = Utils::toGLM(processFace->coords.at(0));
        glm::vec3 t1 = Utils::toGLM(processFace->coords.at(1));
        glm::vec3 t2 = Utils::toGLM(processFace->coords.at(2));

        bool cap = false;
        for (const auto& vert : processFace->tessVerts) {
            if (vert->vd.idx() == 24033) {
                cap = true;
            }
        }

        std::unordered_map<SM_face_descriptor, Vector> prevNorms;
        std::unordered_set<int> vs{24033, 24030, 24039};
        if (cap) {
            for (auto& tessFace : processFace->tessFaces) {
                if (vs.find(tessFace->vertices[0]->vd.idx()) != vs.end() &&
                    vs.find(tessFace->vertices[1]->vd.idx()) != vs.end() &&
                    vs.find(tessFace->vertices[2]->vd.idx()) != vs.end()) {
                    int x = 1;
                }
                auto preNorm = get(fNorms, tessFace->fd);
                prevNorms.insert({tessFace->fd, preNorm});
            }
        }
        //Move
        for (auto& innerVert : processFace->innerVerts) {
            if (innerVert->matchingFeature != nullptr) {
                glm::vec3 newCoords = innerVert->matchingFeature->cartCoords;
                innerVert->newCoords = newCoords;
                innerVert->assignedBy = AssigningSection::MoveValidate;
                sm.point(innerVert->vd) = {newCoords.x, newCoords.y, newCoords.z};
            }
        }

//        if (processFace->fd.idx() == 988) {
        FlipResult flipped;
        for (int i = 0; i < processFace->tessVerts.size() && checkFlips(processFace, sm, highResMesh, gaussMap, flipped); i++) {
            auto [gauss, vert] = *(flipped.begin());
            sm.point(vert->vd) = Utils::toPoint3(vert->origCoords);
            vert->matchingFeature = nullptr;
            vert->undoMove();
            vert->assignedBy = AssigningSection::Undone;
        }
//            int x = 1;
//        } else {
//
//            //Validate
//            for (const auto &tessFace: processFace->tessFaces) {
//                if (tessFace->fd.idx() == 41279) {
//                    int x = 1;
//                }
//                Vector norm = get(fNorms, tessFace->fd);
//                double dot;
//                do {
//                    Vector newNorm = Utils::compute_face_normal(tessFace,
//                                                                sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
//                    dot = norm.x() * newNorm.x() + norm.y() * newNorm.y() + norm.z() * newNorm.z();
//
//                    if (dot < 0) {
//                        SM_vertex_descriptor minVD;
//                        unsigned int numEdges = 0;
//                        double minCurvature = std::numeric_limits<double>::max();
//                        for (auto vd: sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
//                            auto tessVert = processFace->vdToTessVert.at(vd);
//                            if (!tessVert->isInner) {
//                                //This is an edge vertex, skip it
//                                numEdges++;
//                                continue;
//                            }
//
//                            auto feature = tessVert->matchingFeature;
//                            if (feature != nullptr) {
//                                auto featureVD = highResMesh.source(feature->hd);
//                                if (get(gaussMap, featureVD) < minCurvature) {
//                                    minCurvature = get(gaussMap, featureVD);
//                                    minVD = vd;
//                                }
//                            }
//                        }
//
//                        if (numEdges == 2 && (minVD == SurfaceMesh::null_vertex() ||
//                                              processFace->vdToTessVert.at(minVD)->matchingFeature ==
//                                              nullptr)) {
//                            stats.mv_edge_cases++;
//                            processFace->vdToTessVert.at(minVD)->assignedBy = AssigningSection::MVEdgeCase;
//                            break;
//                        }
//                        if (minVD == SurfaceMesh::null_vertex()) {
//                            auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
//                            put(fColor, tessFace->fd, CGAL::IO::Color(0xAA, 0x00, 0x00));
//                            CGAL::draw(sm);
//                            break;
//                        }
//                        auto vert = processFace->vdToTessVert.at(minVD);
//                        sm.point(minVD) = Utils::toPoint3(vert->origCoords);
//                        vert->matchingFeature = nullptr;
//                        vert->undoMove();
//                        vert->assignedBy = AssigningSection::Undone;
//                    }
//                } while (dot < 0);
//            }
//
//        }
        //Anchor vertices
        for (const auto& tessVert : processFace->innerVerts) {
            if (tessVert->matchingFeature != nullptr) {
                tessVert->anchored = true;
            }
        }
#if DRAW_STEPS || DRAW_MV
        CGAL::draw(sm, "Move and Validate");
#endif
    }

    void projectAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh, Tree& aabbTree,
                            VertSet& interpolateVerts, Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;
        auto gaussMap = highResMesh.property_map<SM_vertex_descriptor, double>("v:curvature").first;

        //Project unmatched inner verts along normal to intersection
        for (auto& tessVert : processFace->innerVerts) {
            if (!tessVert->isAssigned()) {
                Point_3 vPoint = sm.point(tessVert->vd);
                Vector norm = get(vNorms, tessVert->vd);
                auto projOnNormal = Utils::findIntersection(vPoint, norm, aabbTree, highResMesh);
                if (projOnNormal.has_value()) {
                    auto [point, nrm] = projOnNormal.value();
                    auto newCoords = Utils::toGLM(point);
                    tessVert->newCoords = newCoords;
                    tessVert->assignedBy = AssigningSection::ProjectValidate;
                    sm.point(tessVert->vd) = point;
                    put(vNorms, tessVert->vd, nrm);
                }
            }
        }

        FlipResult flipped;
        checkFlips(processFace, sm, highResMesh, gaussMap, flipped);
        for (auto& [gauss, vert] : flipped) {
            sm.point(vert->vd) = Utils::toPoint3(vert->origCoords);
            vert->matchingFeature = nullptr;
            vert->undoMove();
            vert->assignedBy = AssigningSection::Undone;
        }

        //Validate
//        for (const auto& tessFace : processFace->tessFaces) {
//            Vector norm = get(fNorms, tessFace->fd);
//            double dot;
//            do {
//                Vector newNorm = Utils::compute_face_normal(tessFace, sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
//                dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();
//
//                if (dot < 0) {
//                    SM_vertex_descriptor undoVD;
//                    for (auto vd : sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
//                        auto tessVert = processFace->vdToTessVert.at(vd);
//                        if (!tessVert->isInner || tessVert->anchored) {
//                            //This is an edge or anchored vertex, skip it
//                            continue;
//                        }
//                        undoVD = vd;
//                    }
//
//                    auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
//
//                    if (processFace->vdToTessVert.at(undoVD)->matchingFeature == nullptr) {
//                        put(fColor, tessFace->fd, CGAL::IO::Color(0x00, 0xFF, 0x00));
//                        stats.pv_edge_cases++;
//                        processFace->vdToTessVert.at(undoVD)->assignedBy = AssigningSection::PVEdgeCase;
//                        break;
//                    }
//                    auto vert = processFace->vdToTessVert.at(undoVD);
//                    sm.point(undoVD) = Utils::toPoint3(vert->origCoords);
//                    vert->matchingFeature = nullptr;
//                    vert->undoMove();
//                    vert->assignedBy = AssigningSection::Undone;
//                }
//            } while(dot < 0);
//        }

        //Add unmatched edges to interpolation list
        for (const auto& innerVert : processFace->innerVerts) {
            if (!innerVert->isAssigned()) {
                interpolateVerts.insert(innerVert);
            }
        }
#if DRAW_STEPS || DRAW_PV
        CGAL::draw(sm, "Project and Validate");
#endif
    }

    void interpolateUnmatched(const VertSet& vertices, SurfaceMesh& sm) {
        glm::dvec3 sum = {};
        std::vector<glm::vec3> adjPoints;
        for (auto& vert : vertices) {
            adjPoints.clear();
            sum = {};

            for (auto hd : halfedges_around_source(vert->vd, sm)) {
                Point_3 p = sm.point(sm.target(hd));
                adjPoints.emplace_back(p.x(), p.y(), p.z());
            }

            //Iterate over each adjacent vertex, find vector from that vertex to all others, add it to sum
            //we will then average them all to get the points interpolated pos.
            for (int to = 1; to < adjPoints.size(); to++) {
                auto vector = adjPoints.at(to) - adjPoints.at(0);
                sum += vector;
            }
            auto avg = (glm::vec3)(sum / (double)(adjPoints.size() - 1));
            auto newPoint = adjPoints.at(0) + avg;
            vert->newCoords = newPoint;
            vert->anchored = true;
            vert->assignedBy = InterpolateUnmatched;
            sm.point(vert->vd) = Utils::toPoint3(newPoint);
        }
    }

    void calculateUVs(SurfaceMesh& sm_tess, const SurfaceMesh& sm_orig, const ProcessFaceMap& processedFaces) {
        auto tess_uvmap = sm_tess.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto orig_uvmap = sm_orig.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto orig_eSeams = sm_orig.property_map<SM_edge_descriptor , bool>("e:on_seam").first;

        auto calculateEdgeUVs = [&](const ProcessEdgePtr& e, const ProcessFacePtr& f, const TessVertPtr& v0, const TessVertPtr& v1) {
            bool bary_basis_match = e->v0->vd == v0->vd;
            auto v0_v1_hd = sm_orig.halfedge(v0->vd, v1->vd);
            auto v0_uv = Utils::toGLM(get(orig_uvmap, v0_v1_hd));
            auto v1_uv = Utils::toGLM(get(orig_uvmap, sm_orig.next(v0_v1_hd)));
            auto v0_v1_vec = v1_uv - v0_uv;

            auto& tessVerts = e->tessVerts;
            std::sort(tessVerts.begin(), tessVerts.end(), [bary_basis_match](const TessVertPtr& a, const TessVertPtr& b) {
                //Flip the sort direction if the bary coords are based opposite.
                //This way, the verts will be sorted going from v0 -> v1;
                return bary_basis_match ? a->baryCoords.x > b->baryCoords.x : a->baryCoords.x < b->baryCoords.x;
            });

            std::vector<TessVertPtr> allVerts;
            allVerts.reserve(tessVerts.size() + 2);
            allVerts.push_back(v0);
            allVerts.insert(allVerts.end(), tessVerts.begin(), tessVerts.end());
            allVerts.push_back(v1);

            for (int i = 0; i < allVerts.size(); i++) {
                if (i == allVerts.size() - 1) {
                    auto a = allVerts.at(i-1);
                    auto b = allVerts.at(i);
                    auto hd = sm_tess.next(sm_tess.halfedge(a->vd, b->vd));
                    Point_2 uv = {v1_uv.x, v1_uv.y};
                    put(tess_uvmap, hd, uv);
                    continue;
                }

                auto a = allVerts.at(i);
                auto b = allVerts.at(i+1);
                auto edge_hd = sm_tess.halfedge(a->vd, b->vd);

                auto offset = bary_basis_match ? 1.0f - a->baryCoords.x : a->baryCoords.x;
                auto newUV = v0_uv + (v0_v1_vec * offset);
                Point_2 uv = {newUV.x, newUV.y};
                if (i == 0) {
                    uv = {v0_uv.x, v0_uv.y};
                }

                put(tess_uvmap, edge_hd, uv);

                for (auto hd : halfedges_around_source(a->vd, sm_tess)) {
                    auto& vdMap = f->vdToTessVert;
                    auto t = sm_tess.target(hd);
                    if (vdMap.find(t) != vdMap.end() && vdMap.at(t)->isInner) {
                        put(tess_uvmap, hd, uv);
                    }
                }
            }
        };

        for (const auto& [fd, face] : processedFaces) {
            auto [uv0, uv1, uv2] = face->uvs;
            glm::vec2 uv0_glm = Utils::toGLM(uv0), uv1_glm = Utils::toGLM(uv1), uv2_glm = Utils::toGLM(uv2);

            //Inner verts are easy, just calculate the barycentric combination of each uv.
            for (const auto& innerVert: face->innerVerts) {
                auto bary = innerVert->baryCoords;
                auto uv_glm = (bary.x * uv0_glm) + (bary.y * uv1_glm )+ (bary.z * uv2_glm);
                Point_2 uv = {uv_glm.x, uv_glm.y};

                for (auto hd : halfedges_around_source(innerVert->vd, sm_tess)) {
                    put(tess_uvmap, hd, uv);
                }
            }

            auto& vdMap = face->vdToTessVert;
            calculateEdgeUVs(face->e01, face, vdMap.at(face->vds.at(0)), vdMap.at(face->vds.at(1)));
            calculateEdgeUVs(face->e12, face, vdMap.at(face->vds.at(1)), vdMap.at(face->vds.at(2)));
            calculateEdgeUVs(face->e02, face, vdMap.at(face->vds.at(2)), vdMap.at(face->vds.at(0)));
        }
    }

    double calculateError(const SurfaceMeshPtr& tessMesh, const SurfaceMeshPtr& origMesh) {

    }
}