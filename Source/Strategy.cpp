//
// Created by dallin on 3/1/23.
//

#include "Strategy.h"
#include "Utils.h"
#include "Tessellator.h"
#include <Hungarian.h>

namespace Strategy {
    void tessellateFace(const ProcessFacePtr& processFace, SurfaceMesh& sm, const TessLevel& tL, TessEdgeMap& edgeVerts,
                        Stats& stats) {
        auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;
        auto removed = sm.property_map<SM_vertex_descriptor, bool>("v:removed").first;
        auto e01hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(1)));
        auto e02hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(2)));
        auto e12hd = sm.edge(sm.halfedge(processFace->vds.at(1), processFace->vds.at(2)));

        bool e01_exists = edgeVerts.find(e01hd) != edgeVerts.end();
        bool e02_exists = edgeVerts.find(e02hd) != edgeVerts.end();
        bool e12_exists = edgeVerts.find(e12hd) != edgeVerts.end();

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
//            auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                if (bary.x == 0.0) {
                    // On p1 - p2 edge
                    if (!e12_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.y, 0.0, 0.0));
                        tessVert->vd = vd;
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

                } else if (bary.y == 0.0) {
                    // On p0 - p2 edge
                    if (!e02_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.x, 0.0, 0.0));
                        tessVert->vd = vd;
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
                } else if (bary.z == 0.0) {
                    // On p0 - p1 edge
                    if (!e01_exists) {
                        auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                        tessVert = std::make_shared<TessellatedVert>(coord, glm::vec3(bary.x, 0.0, 0.0));
                        tessVert->vd = vd;
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
                } else {
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

    FeatureVerts extractFeatureVertices(Point2Set& pSet, const ProcessFacePtr& tri, const SurfaceMesh& highResMesh,
                                        Stats& stats) {
        std::list<Vertex_handle> vertexList;
        pSet.range_search(tri->uvs.at(0), tri->uvs.at(1), tri->uvs.at(2), std::back_inserter(vertexList));

        FeatureVerts foundVerts;
        foundVerts.reserve(vertexList.size());

        auto t0 = Utils::toGLM(tri->uvs.at(0)); // (1, 0, 0)
        auto t1 = Utils::toGLM(tri->uvs.at(1)); // (0, 1, 0)
        auto t2 = Utils::toGLM(tri->uvs.at(2)); // (0, 0, 1)

        for (const auto vHandle : vertexList) {
            auto hd = SM_halfedge_descriptor(vHandle->info());
            Point_3 point = highResMesh.point(highResMesh.source(hd));
            glm::vec3 vertex = {point.x(), point.y(), point.z()};
            glm::vec2 uv ={vHandle->point().x(), vHandle->point().y()};
            glm::vec3 baryCoords = Utils::barycentric(uv, t0, t1, t2);

            FeatureVertPtr feature = std::make_shared<FeatureVert>(hd, vertex, baryCoords, uv);
            foundVerts.emplace_back(feature);
        }

        return foundVerts;
    }

    void minBiGraphMatch(const ProcessFacePtr& processFace, const FeatureVerts& featureVerts, Stats& stats) {
        int ol0 = 5, ol1 = 5, ol2 = 5, il = 5;

        auto t0 = Utils::toGLM(processFace->uvs.at(0)); // (1, 0, 0)
        auto t1 = Utils::toGLM(processFace->uvs.at(1)); // (0, 1, 0)
        auto t2 = Utils::toGLM(processFace->uvs.at(2)); // (0, 0, 1)

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

    void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig,
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
                sm.point(innerVert->vd) = {newCoords.x, newCoords.y, newCoords.z};
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
                            auto featureVD = sm_orig.source(feature->hd);
                            if (get(gaussMap, featureVD) < minCurvature) {
                                minCurvature = get(gaussMap, featureVD);
                                minVD = vd;
                            }
                        }
                    }

                    if (numEdges == 2 && (minVD == SurfaceMesh::null_vertex() || processFace->vdToTessVert.at(minVD)->matchingFeature ==
                                                                                 nullptr)) {
                        stats.mv_edge_cases++;
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

    void projectAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig, Tree& aabbTree,
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

                    if (undoVD == SurfaceMesh::null_vertex() || processFace->vdToTessVert.at(undoVD)->matchingFeature == nullptr) {
                        put(fColor, tessFace->fd, CGAL::IO::Color(0x00, 0xFF, 0x00));
                        stats.pv_edge_cases++;
                        break;
                    }
                    if (undoVD == SurfaceMesh::null_vertex()) {
                        put(fColor, tessFace->fd, CGAL::IO::Color(0x00, 0xFF, 0x00));
                        CGAL::draw(sm);
                        break;
                    } if (undoVD.idx() == 16659) {
                        CGAL::draw(sm);
                    }
                    auto vert = processFace->vdToTessVert.at(undoVD);
                    sm.point(undoVD) = Utils::toPoint3(vert->origCoords);
                    vert->matchingFeature = nullptr;
                    vert->undoMove();
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
}