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
        auto e01hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(1)));
        auto e02hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(2)));
        auto e12hd = sm.edge(sm.halfedge(processFace->vds.at(1), processFace->vds.at(2)));

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
            auto e01 = std::make_shared<ProcessEdge>(e01hd);
            processFace->e01 = e01;
            edgeVerts.insert({e01hd, e01});
        }

        if (e02_exists) {
            processFace->e02 = edgeVerts.at(e02hd);
            e02Verts = processFace->e02->tessVerts;
        } else {
            auto e02 = std::make_shared<ProcessEdge>(e02hd);
            processFace->e02 = e02;
            edgeVerts.insert({e02hd, e02});
        }

        if (e12_exists) {
            processFace->e12 = edgeVerts.at(e12hd);
            e12Verts = processFace->e12->tessVerts;
        } else {
            auto e12 = std::make_shared<ProcessEdge>(e12hd);
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
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.y, 0.0, 0.0));
                        tessVert->vd = vd;
//                        if (e12_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e12->tessVerts.push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(1));
                        auto b = get(vNorms, processFace->vds.at(2));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.y));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
                        //Finds the given barycentric tessellated vert in the existing edges vector of tessVerts
                        double testBary = processFace->e12->v0->vd == processFace->vds.at(1) ? bary.y : bary.z;

                        auto i = std::find_if(processFace->e12->tessVerts.begin(), processFace->e12->tessVerts.end(), [&](const TessVertPtr& a) {
                            return abs(a->baryCoords.x - testBary) <= 1e-6;
                        });
                        tessVert = *i;
                    }

                }
                else if (bary.y == 0.0) {
                    // On p0 - p2 edge
                    if (!e02_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.x, 0.0, 0.0));
                        tessVert->vd = vd;
//                        if (e02_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e02->tessVerts.push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(0));
                        auto b = get(vNorms, processFace->vds.at(2));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.x));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
                        double testBary = processFace->e02->v0->vd == processFace->vds.at(0) ? bary.x : bary.z;

                        auto i = std::find_if(processFace->e02->tessVerts.begin(), processFace->e02->tessVerts.end(), [&](const TessVertPtr& a) {
                            return abs(a->baryCoords.x - testBary) <= 1e-6;
                        });
                        tessVert = *i;
                    }
                }
                else if (bary.z == 0.0) {
                    // On p0 - p1 edge
                    if (!e01_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.x, 0.0, 0.0));
                        tessVert->vd = vd;
//                        if (e01_is_seam) {
//                            //If edge is a seam, make this vertex on seam.
//                            tessVert->isOnSeam = true;
//                        }
                        processFace->e01->tessVerts.push_back(tessVert);

                        auto a = get(vNorms, processFace->vds.at(0));
                        auto b = get(vNorms, processFace->vds.at(1));
                        auto nrm = Utils::normalize(Utils::lerp(a, b, bary.x));
                        put(vNorms, vd, nrm);
                    } else {
                        //TODO: FIX THIS!!! It'll be nasty
                        double testBary = processFace->e01->v0->vd == processFace->vds.at(0) ? bary.x : bary.y;

                        auto i = std::find_if(processFace->e01->tessVerts.begin(), processFace->e01->tessVerts.end(), [&](const TessVertPtr& a) {
                            return abs(a->baryCoords.x - testBary) <= 1e-6;
                        });
                        tessVert = *i;
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

        // temporarily save uvs for ends of a seam edge, so we can use them after the face is tessellated.
//        struct SeamUV {
//            Point_2 v0_v1, v1_vL, v1_v0, v0_vR;
//        };
//        std::unordered_map<SM_edge_descriptor, SeamUV> seamUVMap;
//        auto addSeamUV = [&](const TessVertPtr& v0, const TessVertPtr& v1, SM_edge_descriptor ed) {
//            SeamUV seamUV;
//            auto v0_v1 = sm.halfedge(v0->vd, v1->vd), v1_v0 = sm.halfedge(v1->vd, v0->vd);;
//            seamUV.v0_v1 = get(uvmap, v0_v1);
//            seamUV.v1_v0 = get(uvmap, v1_v0);
//            seamUV.v0_vR = get(uvmap, sm.next(v1_v0));
//            seamUV.v1_vL = get(uvmap, sm.next(v0_v1));
//            seamUVMap.insert({ed, seamUV});
//        };
//
//        if (e01_is_seam && !e01_exists) {
//            addSeamUV(processFace->e01->v0, processFace->e01->v1, e01hd);
//        }
//        if (e12_is_seam && !e12_exists) {
//            addSeamUV(processFace->e12->v0, processFace->e01->v1, e12hd);
//        }
//        if (e02_is_seam && !e02_exists) {
//            addSeamUV(processFace->e02->v0, processFace->e02->v1, e02hd);
//        }

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

        auto setSeams = [&](const ProcessEdgePtr& e) {
            auto& tessVerts = e->tessVerts;
            std::sort(tessVerts.begin(), tessVerts.end(), [](const TessVertPtr& a, const TessVertPtr& b) {
                return a->baryCoords.x < b->baryCoords.x;
            });

            auto v0 = e->v0, v1 = e->v1;
            auto e0 = sm.edge(sm.halfedge(v1->vd, tessVerts.at(0)->vd)), e1 = sm.edge(sm.halfedge(tessVerts.at(tessVerts.size()-1)->vd, v0->vd));
            put(eSeamMap, e0, true);
            put(eSeamMap, e1, true);

            for (int i = 0; i < tessVerts.size() - 1; i++) {
                auto a = tessVerts.at(i);
                auto b = tessVerts.at(i+1);
                put(vSeamMap, a->vd, true);
                put(eSeamMap, sm.edge(sm.halfedge(a->vd, b->vd)), true);
            }
        };

        if (!e01_exists && e01_is_seam) {
            setSeams(processFace->e01);
        }
        if (!e12_exists && e12_is_seam) {
            setSeams(processFace->e12);
        }
        if (!e02_exists && e02_is_seam) {
            setSeams(processFace->e02);
        }

//        auto calculateSeamUV = [&](const ProcessEdgePtr& e) {
//            auto& tessVerts = e->tessVerts;
//            std::sort(tessVerts.begin(), tessVerts.end(), [](const TessVertPtr& a, const TessVertPtr& b) {
//                return a->baryCoords.x < b->baryCoords.x;
//            });
//
//            std::vector<TessVertPtr> allVerts;
//            allVerts.reserve(tessVerts.size() + 2);
//            allVerts.push_back(e->v0);
//            allVerts.insert(allVerts.end(), tessVerts.begin(), tessVerts.end());
//            allVerts.push_back(e->v1);
//
//            auto seamUVs = seamUVMap.at(e->ed);
//            auto v0_v1_vec = Utils::toGLM(seamUVs.v1_vL) - Utils::toGLM(seamUVs.v0_v1);
//            auto v1_v0_vec = Utils::toGLM(seamUVs.v0_vR) - Utils::toGLM(seamUVs.v1_v0);
//
//            for (int i = 0; i < allVerts.size(); i++) {
//                auto a = allVerts.at(i);
//                auto b = allVerts.at(i + 1);
//                auto hd = sm.halfedge(a->vd, b->vd);
//                Point_2 uv;
//                if (i == 0) {
//                    uv = seamUVs.v0_v1;
//                } else if (i == allVerts.size() - 1) {
//                    uv = seamUVs.v1_vL;
//                } else {
//                    auto newUV = Utils::toGLM(seamUVs.v0_v1) + (v0_v1_vec * a->baryCoords.x);
//                    uv = {newUV.x, newUV.y};
//                }
//
//                put(uvmap, hd, uv);
//            }
//
//            for (int i = allVerts.size() - 1; i > 0; i--) {
//                auto a = allVerts.at(i);
//                auto b = allVerts.at(i - 1);
//                auto hd = sm.halfedge(a->vd, b->vd);
//                Point_2 uv;
//                if (i == allVerts.size() - 1) {
//                    uv = seamUVs.v1_v0;
//                } else if (i == 1) {
//                    uv = seamUVs.v0_vR;
//                } else {
//                    auto newUV = Utils::toGLM(seamUVs.v1_v0) + (v1_v0_vec * (1.0f - a->baryCoords.x));
//                    uv = {newUV.x, newUV.y};
//                }
//
//                put(uvmap, hd, uv);
//            }
//        };
//        if (e01_is_seam && !e01_exists) {
//            calculateSeamUV(processFace->e01);
//        }
//        if (e12_is_seam && !e12_exists) {
//            calculateSeamUV(processFace->e12);
//        }
//        if (e02_is_seam && !e02_exists) {
//            calculateSeamUV(processFace->e02);
//        }

        stats.processed_edges = edgeVerts.size();
#if DRAW_STEPS || DRAW_T
        CGAL::draw(sm, "Tessellation");
#endif
    }

    void projectEdgeVerts(const ProcessFacePtr& processFace, SurfaceMesh& sm, Tree& aabbTree, VertSet& interpolateVerts,
                          TessEdgeMap& edgeVerts, Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

        //Project edges along normal to intersection
        for (const auto& vertex : processFace->tessVerts) {
            if (!vertex->isInner && !vertex->anchored && interpolateVerts.find(vertex) == interpolateVerts.end()) {
                Point_3 vPoint = sm.point(vertex->vd);
                Vector norm = get(vNorms, vertex->vd);
                auto projOnNormal = Utils::findIntersection(vPoint, norm, aabbTree);
                if (projOnNormal.has_value()) {
                    vertex->newCoords = Utils::toGLM(projOnNormal.value());
                    vertex->anchored = true;
                    vertex->assignedBy = AssigningSection::ProjectEdge;
                    sm.point(vertex->vd) = projOnNormal.value();
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
            auto hd = highResMesh.halfedge(tri->vds.at(0));
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

    std::vector<int> vs = {104582,104102,104092,103301,103009,102118,102125,101511,101414,101415,101156,100990,100994,101001,100665,100298,99907,99912,
                           99674,99676,99677,99682,99640,99621,99349,99273,99246,99249,98954,98904,98907,98542,98544,98546,98547,98549,98308,97920,97663,
                           97669,97588,97410,97349,96921,53119,52386,52026,51987,51990,51941,51942,50758,49441,49426,49428,49205,49206,49138,49140,49144,
                           49145,48914,48366,48367,48375,48376,48221,48154,47779,46918,46666,46669,46674,46657,45882,45883,45809,45565,45542,45522,45529,
                           45504,45337,44963,44965,44968,44970,44901,44689,44633,44564,44205,44206,43907,43911,43844,43534,43536,43537,43538,43542,43543,
                           43545,43478,43239,43240,42730,42741,42191,42195,41433,41414,41418,41421,40994,41001,41002,41003,40647,40020,40023,40025,39658,
                           39659,39666,39586,39595,39139,38140,38095,37806,36558,36560,36561,36540,35971,34847,34481,34461,33694,33606,33522,33489,33403,
                           33208,32929,32673,32581,32423,32211,32197,32065,32005,31810,31815,31816,31819,31820,31665,31534,31537,31538,31479,31463,31376,
                           31352,31357,31114,31056,31059,30985,30922,30931,30565,30566,30432,30136,30139,30065,30029,30030,29886,29782,29784,29787,29790,
                           29750,29684,29641,29296,29216,29217,28991,28890,28869,28870,28668,28669,28670,28624,28598,28609,28586,28456,28371,28248,28100,
                           28072,28026,28005,28010,27721,27722,27453,27457,27306,27312,27098,27022,27024,14351,14306,14220,14224,14202,14175,14154,14094,
                           14095,14097,14098,14100,14061,14064,14067,13974,13940,13921,13837,13812,13718,13626,13632,13505,13483,13486,13487,13488,13492,
                           13493,13375,13377,13378,13098,13028,12980,12870,12638,12601,12602,12438,12440,12357,12280,12281,12289,12254,12257,12258,12259,
                           12260,12261,12263,12264,12265,12206,12210,12211,12124,12133,12113,12085,12086,11945,11825,11795,11775,11731,11586,11559,11515,11517,11392,11363,11366,11167,11174,11010,10970,10947,10900,10884,10855,10860,10810,10798,10753,10688,10689,10613,10616,10618,10547,10557,10302,10261,10263,10266,10269,10126,10133,10105,10107,10110,9972,9977,9869,9870,9779,9785,9734,9737,9742,9743,9744,9691,9620,9621,9624,9626,9602,9579,9585,9542,9417,9418,9419,9355,9357,9365,9331,9334,9290,9226,9118,9119,9121,9125,8967,8954,8860,8862,8627,
                           8579,8583,8585,8444,5576,5535,5537,5538,5539,5467,5468,5477,5388,5391,5393,5394,5396,5346,5303,5305,5307,5190,5193,5198,5171,5156,5020,4950,4925,4786,4787,4768,4667,4668,4673,4580,4582,4584,4466,4472,4474,4476,4449,4450,4372,4374,4380,4350,4357,4212,4216,4191,4194,4200,3415,3423,2913,3562,3626,3630,3631,3649,3651,3814,3835,3837,3998,4124,5702,5707,5728,5730,5824,5838,5839,5884,5887,5889,5890,5893,5940,5995,6218,6242,6243,6262,6325,6474,6513,6658,6699,6701,6706,6739,6762,6763,6794,6854,6861,6945,6967,7019,7020,7174,7176,7180,7214,7215,7218,7221,7223,7238,7246,7418,7434,7443,7445,7455,7477,7484,7720,7957,7958,7959,8092,8210,14556,14597,14598,14623,14624,14680,14682,14683,14689,14709,14874,15155,15190,15472,15513,15514,15633,15655,15663,15664,15773,15812,15833,15837,15838,15895,15896,15899,15914,15915,15917,15918,16170,16171,16173,16176,16257,16270,16271,16278,16281,16431,16450,16545,16606,16797,16799,16861,16869,16905,16920,16925,16926,16927,16929,16997,17011,17014,17015,17019,17039,17109,17110,17117,17136,17309,17324,17341,17434,17435,17436,17501,17696,17716,17883,17890,17985,18009,18016,18426,18520,18597,18663,18947,18950,19265,19266,19389,19397,19405,19475,19490,19493,19494,19498,19708,19713,
                           19862,19863,19864,19867,19995,19996,20032,20246,20307,20541,20829,20830,20852,20857,20860,20914,20916,20920,20922,20923,20979,20984,21013,21058,21163,21168,21183,21311,21315,21373,21380,21549,21589,21692,21753,21775,21777,21792,21795,21816,21901,21971,21980,22176,22446,22498,22499,22501,22534,22541,22554,22571,22635,22638,22639,22642,22643,22658,22813,22818,22916,22937,23052,23166,23212,23213,23680,23683,23704,24012,24021,24239,24241,24772,24775,24779,24780,24781,24854,24855,24857,24859,24860,25244,25306,25465,25483,25488,25530,25533,25825,26455,53335,53336,53345,53490,54151,54219,54285,54886,54887,54912,55455,56209,56717,57335,57339,57440,57798,57800,58219,58270,59075,59079,60036,61202,61238,61281,61301,61738,62055,62547,62551,62838,63183,63185,63186,63677,65192,65264,65274,65304,65314,65318,65355,65359,65360,65366,65579,65664,65668,65703,65705,65706,65707,65708,65770,65803,65827,65837,65859,65861,65862,65863,65867,65868,65869,65893,65894,65906,65910,66073,66146,66242,66245,66250,66308,66403,66407,66412,66465,66466,66479,66480,66483,66484,66552,66624,66652,66686,66742,66809,66810,66830,66843,66859,66864,66891,66892,66893,66900,66918,66920,66921,66956,67010,67012,67123,67124,67127,67128,67146,67148,67151,67155,67166,67167,67190,67192,67197,67199,67201,67251,67255,67261,67269,67312,67374,67377,67453,67480,67484,67578,67586,67592,67679,67713,67761,67821,67835,67845,67897,67986,67987,67988,67997,68062,68063,68137,68142,68143,68223,68271,68287,68291,68316,68323,68369,68370,68456,68579,68587,68589,68590,68611,68620,68681,68690,68699,68718,68770,68773,68780,68789,68795,68796,68820,68836,69085,69322,69323,69324,69333,69393,69456,69525,69576,69647,69691,69713,69719,69731,69744,69749,69753,69772,69827,69828,69978,69983,70131,70140,70144,70152,70156,70172,70299,70300,70304,70308,70309,70586,70588,70693,70694,70727,70767,70772,70773,70775,70776,70778,70993,71008,71014,71018,71083,71191,71347,71428,71431,71440,71446,71506,71508,71513,71725,
                           71727,71900,71926,71954,71969,71980,71985,72126,72129,72146,72148,72332,72336,72352,72422,72425,72522,72529,72578,72593,72623,72719,72721,72893,72896,72998,73043,73050,73052,73056,73059,73061,73112,73113,73121,73284,73335,73369,73374,73379,73389,73433,73524,73528,73600,73609,73754,73760,73766,73768,73770,73772,73774,73785,73791,73794,73796,73882,73883,73885,73886,73889,73890,73891,73892,73929,73930,73938,73941,73948,73954,73956,73959,73961,73963,73964,73993,74335,74339,74375,74478,74484,74572,74711,74892,74894,74919,75067,75320,75322,75323,75325,75335,75351,75425,75513,75514,75528,75538,75551,75603,75605,75606,75653,75695,75742,75748,75974,75975,75984,75988,76013,76014,76016,76018,76022,76025,76026,76146,76273,76362,76444,76490,76671,76673,76674,76675,76676,76711,76719,76809,76815,76822,76830,76846,76865,76881,76887,76888,76906,76954,76992,77169,77274,77369,77370,77415,77418,77483,77502,77657,77693,77721,77785,77801,77826,77840,77842,77844,77848,77849,77855,77856,77872,77894,77897,77938,77941,78101,78102,78419,78632,78635,78636,78704,78752,78828,78835,78852,78859,78860,78942,79111,79145,79166,79167,79177,79202,79368,79392,79411,79415,79511,79515,79570,79614,79616,79626,79918,79989,80005,80006,80105,80350,80631,80839,80843,80969,80976,81009,81075,81080,81469,81494,81505,81574,81789,82244,82330,82576,82618,82665,82790,82838,82915,83032,83141,83228,83234,83237,83293,83395,83439,83581,83616,83832,83838,83999,84003,84031,84036,84039,84526,84548,85655,85940,86050,86792,87347,87356,87366,87439,88079,88080,88084,88358,88412,88651,88807,88808,88871,89070,89078,89377,89380,89381,89830,89878,89884,89972,90209,90470,90817,91295,996296,96301,96302,96501};


    void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh,
                         const Gauss_vertex_pmap& gaussMap, Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;


        glm::vec3 t0 = Utils::toGLM(processFace->coords.at(0));
        glm::vec3 t1 = Utils::toGLM(processFace->coords.at(1));
        glm::vec3 t2 = Utils::toGLM(processFace->coords.at(2));

        //Move
        for (auto& innerVert : processFace->innerVerts) {
            if (innerVert->matchingFeature != nullptr) {
                glm::vec3 newCoords = innerVert->matchingFeature->cartCoords;
                innerVert->newCoords = newCoords;
                innerVert->assignedBy = AssigningSection::MoveValidate;
                sm.point(innerVert->vd) = {newCoords.x, newCoords.y, newCoords.z};
            }
        }

        //Validate
        for (const auto& tessFace : processFace->tessFaces) {
            Vector norm = get(fNorms, tessFace->fd);
            for (const auto& vert: tessFace->vertices) {
                if (std::find(vs.begin(), vs.end(), vert->vd.idx()) != std::end(vs)) {
                    int x = 1;
                }
            }
            double dot;
            do {
                Vector newNorm = Utils::compute_face_normal(tessFace, sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
                dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

                if (dot < 0) {
                    SM_vertex_descriptor minVD;
                    unsigned int numEdges = 0;
                    double minCurvature = std::numeric_limits<double>::max();
                    for (auto vd : sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
                        auto tessVert = processFace->vdToTessVert.at(vd);
                        if (!tessVert->isInner) {
                            //This is an edge vertex, skip it
                            numEdges++;
                            continue;
                        }

                        auto feature = tessVert->matchingFeature;
                        if (feature != nullptr) {
                            auto featureVD = highResMesh.source(feature->hd);
                            if (get(gaussMap, featureVD) < minCurvature) {
                                minCurvature = get(gaussMap, featureVD);
                                minVD = vd;
                            }
                        }
                    }

                    if (numEdges == 2 && (minVD == SurfaceMesh::null_vertex() || processFace->vdToTessVert.at(minVD)->matchingFeature ==
                                                                                 nullptr)) {
                        stats.mv_edge_cases++;
                        processFace->vdToTessVert.at(minVD)->assignedBy = AssigningSection::MVEdgeCase;
                        break;
                    }
                    if (minVD == SurfaceMesh::null_vertex()) {
                        auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
                        put(fColor, tessFace->fd, CGAL::IO::Color(0xAA, 0x00, 0x00));
                        CGAL::draw(sm);
                        break;
                    }
                    auto vert = processFace->vdToTessVert.at(minVD);
                    sm.point(minVD) = Utils::toPoint3(vert->origCoords);
                    vert->matchingFeature = nullptr;
                    vert->undoMove();
                    vert->assignedBy = AssigningSection::Undone;
                }
            } while(dot < 0);
        }

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

        //Project unmatched inner verts along normal to intersection
        for (auto& tessVert : processFace->innerVerts) {
            if (!tessVert->isAssigned()) {
                Point_3 vPoint = sm.point(tessVert->vd);
                Vector norm = get(vNorms, tessVert->vd);
                auto projOnNormal = Utils::findIntersection(vPoint, norm, aabbTree);
                if (projOnNormal.has_value()) {
                    auto newCoords = Utils::toGLM(projOnNormal.value());
                    tessVert->newCoords = newCoords;
                    tessVert->assignedBy = AssigningSection::ProjectValidate;
                    sm.point(tessVert->vd) = projOnNormal.value();
                }
            }
        }

        //Validate
        for (const auto& tessFace : processFace->tessFaces) {
            Vector norm = get(fNorms, tessFace->fd);
            double dot;
            do {
                Vector newNorm = Utils::compute_face_normal(tessFace, sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
                dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

                if (dot < 0) {
                    SM_vertex_descriptor undoVD;
                    for (auto vd : sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
                        auto tessVert = processFace->vdToTessVert.at(vd);
                        if (!tessVert->isInner || tessVert->anchored) {
                            //This is an edge or anchored vertex, skip it
                            continue;
                        }
                        undoVD = vd;
                    }

                    auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;

                    if (processFace->vdToTessVert.at(undoVD)->matchingFeature == nullptr) {
                        put(fColor, tessFace->fd, CGAL::IO::Color(0x00, 0xFF, 0x00));
                        stats.pv_edge_cases++;
                        processFace->vdToTessVert.at(undoVD)->assignedBy = AssigningSection::PVEdgeCase;
                        break;
                    }
                    auto vert = processFace->vdToTessVert.at(undoVD);
                    sm.point(undoVD) = Utils::toPoint3(vert->origCoords);
                    vert->matchingFeature = nullptr;
                    vert->undoMove();
                    vert->assignedBy = AssigningSection::Undone;
                }
            } while(dot < 0);
        }

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