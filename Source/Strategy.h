//
// Created by dallin on 3/1/23.
//

#ifndef TESSELLATION_STRATEGY_H
#define TESSELLATION_STRATEGY_H

#include "MeshImpl.h"
#include "IO.h"
#include "Utils.h"

const int test_vd = 5767;

namespace Strategy {
    struct Stats {
        unsigned int invalid_tessFaces = 0;
        unsigned int processed_edges = 0;
        unsigned int mv_edge_cases = 0;
        unsigned int pv_edge_cases = 0;
        unsigned int moved_vertices = 0;
        unsigned int total_vertices = 0;

        void print() const {
            std::cout << "\n== Strategy Stats ==\n";
            std::cout << "Invalid faces during tessellation: " << invalid_tessFaces << "\n";
            std::cout << "Edges tessellated: " << processed_edges << "\n";
            std::cout << "MoveAndValidate edge cases: " << mv_edge_cases << "\n";
            std::cout << "ProjectAndValidate edge cases: " << pv_edge_cases << "\n";
            std::cout << "Moved Vertices: " << moved_vertices << "\n";
            std::cout << "Total Vertices: " << total_vertices << "\n";
            std::cout << "====\n\n";
        }

        void clear() {
            invalid_tessFaces = 0;
            processed_edges = 0;
            mv_edge_cases = 0;
            pv_edge_cases = 0;
            moved_vertices = 0;
            total_vertices = 0;
        }
    };
    struct CollapseStats {
        unsigned int removed_verts = 0;
        unsigned int overwritten_verts = 0;
        unsigned int unmatched_verts = 0;
        std::unordered_set<SM_halfedge_descriptor> removed_hds {};
        unsigned int processed_edges = 0;
        unsigned int seam_inner_collapses = 0;
        unsigned int total_seam_inner = 0;
        unsigned int seam_pairs = 0;

        void print() const {
            std::cout << "\n== Collapse Stats ==\n";
            std::cout << "Removed Vertices: " << removed_verts << "\n";
            std::cout << "Unique Halfedges: " << removed_hds.size() << "\n";
            std::cout << "Overwritten Vertices: " << overwritten_verts << "\n";
            std::cout << "Unmatched Vertices: " << unmatched_verts << "\n";
            std::cout << "Seam Inner collapses: " << seam_inner_collapses << "\n";
            std::cout << "Total Seam-Inner pairs: " << total_seam_inner << "\n";
            std::cout << "Total Seam-Seam pairs: " << seam_pairs << "\n";
            std::cout << "====\n\n";
        }
    };

    struct CollapseVisitor : SMS::Edge_collapse_visitor_base<SurfaceMesh> {
        CollapseVisitor(UV_pmap& uvmap, CollapseStats& stats) : uvmap(uvmap), stats(stats) {}

//        void OnSelected(const EdgeProfile& prof, boost::optional<double> cost, std::size_t initial, std::size_t curr) {
//            int x = 0;
//        }

        // Called during the processing phase for each edge being collapsed.
        // If placement is absent the edge is left uncollapsed.
        void OnCollapsing(const EdgeProfile& prof,
                          boost::optional<Point> placement)
        {
            feature0 = {};
            feature1 = {};

            if (placement) {
                auto newPoint = placement.value();
                const auto& mesh = prof.surface_mesh();
                const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
                const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
                auto faceFeatures = mesh.property_map<SM_face_descriptor, std::unordered_set<FeaturePointPtr>>("f:features").first;

                p0 = prof.p0();
                p1 = prof.p1();

                Vector p0Norm = Utils::compute_vertex_normal(mesh, prof.v0());
                Vector p1Norm = Utils::compute_vertex_normal(mesh, prof.v1());

                feature0 = std::make_shared<FeaturePoint>(p0, p0Norm);
                feature1 = std::make_shared<FeaturePoint>(p1, p1Norm);

                freeFeatures.clear();
                auto& features_fL = get(faceFeatures, mesh.face(prof.v1_vL()));
                auto& features_fR = get(faceFeatures, mesh.face(prof.v0_vR()));
                std::copy(features_fL.begin(), features_fL.end(), std::back_inserter(freeFeatures));
                std::copy(features_fR.begin(), features_fR.end(), std::back_inserter(freeFeatures));
                freeFeatures.push_back(feature0);
                freeFeatures.push_back(feature1);

                glm::vec3 glmP0 = {p0.x(), p0.y(), p0.z()};
                glm::vec3 glmP1 = {p1.x(), p1.y(), p1.z()};
                glm::vec3 glmP = {newPoint.x(), newPoint.y(), newPoint.z()};

                float modelEdgeLen = glm::length(glmP1 - glmP0);
                float p_p0Len = glm::length(glmP - glmP0);
                float pp0Ratio = p_p0Len / modelEdgeLen;

                v0_v1_uv = get(uvmap, prof.v0_v1());
                v1_vL_uv = get(uvmap, prof.v1_vL());

                v1_v0_uv = get(uvmap, prof.v1_v0());
                v0_vR_uv = get(uvmap, prof.v0_vR());

                glm::vec2 glm_v0_v1_uv = Utils::toGLM(v0_v1_uv);
                glm::vec2 glm_v1_vL_uv = Utils::toGLM(v1_vL_uv);

                glm::vec2 glm_v1_v0_uv = Utils::toGLM(v1_v0_uv);
                glm::vec2 glm_v0_vR_uv = Utils::toGLM(v0_vR_uv);

                auto p0Vec = glm_v1_vL_uv - glm_v0_v1_uv;
                glm::vec2 v0_v1_new_uv = glm_v0_v1_uv + (p0Vec * pp0Ratio);
                v0v1_p_2 = {v0_v1_new_uv.x, v0_v1_new_uv.y};

                p0Vec = glm_v0_vR_uv - glm_v1_v0_uv;
                glm::vec2 v1_v0_new_uv = glm_v1_v0_uv + (p0Vec * (1.0f - pp0Ratio));
                v1v0_p_2 = {v1_v0_new_uv.x, v1_v0_new_uv.y};



                v0 = prof.v0();
                v1 = prof.v1();
                vR = prof.vR();
                vL = prof.vL();
                v0ds.clear();
                v1ds.clear();
                v0Seams.clear();
                v1Seams.clear();
                v0P_2.clear();
                v1P_2.clear();
                v0_hd_newUV.clear();
                v1_hd_newUV.clear();
                v0_seam_uvs.clear();
                v1_seam_uvs.clear();

                bool v0_is_on_seam = get(vSeamMap, v0);
                bool v1_is_on_seam = get(vSeamMap, v1);
                if (stats.processed_edges == test_vd) {
                    int x = 1;
//                    auto uv0 = get(uvmap, prof.v0_v1());
//                    auto uv1 = get(uvmap, prof.v1_v0());
//                    std::cout << "v0: " << v0 << " - " << uv0.x() << ", " << uv0.y() << "\n";
//                    std::cout << "v1: " << v1 << " - " << uv1.x() << ", " << uv1.y() << "\n";
                }

                const float epsilon = 1e-4;
                const auto p2_equal = [epsilon] (Point_2 a, Point_2 b) {
                    return abs((b - a).x()) < epsilon && abs((b - a).x()) < epsilon;
                };

                int count = 0;
                for (auto hd : halfedges_around_source(v0, mesh)) {
                    bool edge_is_on_seam = get(eSeamMap, mesh.edge(hd));

                    auto v0_uv = get(uvmap, hd);
                    auto vnext_uv = get(uvmap, mesh.next(hd));

                    if (v0_is_on_seam && v1_is_on_seam) {
                        if (p2_equal(v0_uv, v0_vR_uv)) {
                            v0_hd_newUV.insert({mesh.target(hd), v1_v0_uv});
                        }
                        else if (p2_equal(v0_uv, v0_v1_uv)) {
                            v0_hd_newUV.insert({mesh.target(hd), v1_vL_uv});
                        }
                        else {
                            v0_hd_newUV.insert({mesh.target(hd), v0_uv});
                        }
                    }
                    else if (v0_is_on_seam) {
                        if (mesh.target(hd) != v1) {
                            v0_hd_newUV.insert({mesh.target(hd), v0_uv});
                        }
                    }
                    else if (v1_is_on_seam && hd == prof.v0_vR()) {
                        auto v1_vR_hd = mesh.opposite(prof.vR_v1());
                        v0_hd_newUV.insert({mesh.target(v1_vR_hd), get(uvmap, v1_vR_hd)});
                    }
                    else {
                        v0_hd_newUV.insert({mesh.target(hd), v0v1_p_2});
                    }

                    if (edge_is_on_seam) {
                        v0_seam_uvs.emplace_back(mesh.target(hd), get(uvmap, hd));
                        v0Seams.insert({mesh.target(hd)});
                    }
                    count++;
                }

                count = 0;
                for (auto hd : halfedges_around_source(v1, mesh)) {
                    bool edge_is_on_seam = get(eSeamMap, mesh.edge(hd));

                    auto v1_uv = get(uvmap, hd);
                    auto vnext_uv = get(uvmap, mesh.next(hd));
                    if (mesh.target(hd).idx() == 29827 || mesh.target(hd).idx() == 30002) {
                        int x = 0;
                    }

                    if (v0_is_on_seam && v1_is_on_seam) {
                        if (p2_equal(v1_uv, v1_vL_uv)) {
                            v1_hd_newUV.insert({mesh.target(hd), v0_v1_uv});
                        }
                        else if (p2_equal(v1_uv, v1_v0_uv)) {
                            v1_hd_newUV.insert({mesh.target(hd), v0_vR_uv});
                        }
                        else {
                            v1_hd_newUV.insert({mesh.target(hd), v1_uv});
                        }
                    }
                    else if (v1_is_on_seam) {
                        if (mesh.target(hd) != v0) {
                            v1_hd_newUV.insert({mesh.target(hd), v1_uv});
                        }
                    }
                    else if (v0_is_on_seam && hd == prof.v1_vL()) {
                        auto v0_vL_hd = mesh.opposite(prof.vL_v0());
                        v1_hd_newUV.insert({mesh.target(v0_vL_hd), get(uvmap, v0_vL_hd)});
                    }
                    else {
                        v1_hd_newUV.insert({mesh.target(hd), v1v0_p_2});
                    }

                    if (edge_is_on_seam) {
                        v1_seam_uvs.emplace_back(mesh.target(hd), get(uvmap, hd));
                        v1Seams.insert({mesh.target(hd)});
                    }
                    count++;
                }
            }
        }

        // Called after each edge has been collapsed
        void OnCollapsed(const EdgeProfile& prof, vertex_descriptor vd)
        {
            const auto& mesh = prof.surface_mesh();
            const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
            const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
            auto faceFeatures = mesh.property_map<SM_face_descriptor, std::unordered_set<FeaturePointPtr>>("f:features").first;

//            auto f0_face = Utils::project_vertex(mesh, vd, feature0);
//            auto f1_face = Utils::project_vertex(mesh, vd, feature1);

//            for (auto f : mesh.faces_around_target(mesh.halfedge(vd))) {
//                auto& feats = get(faceFeatures, f);
//                std::copy(feats.begin(), feats.end(), std::back_inserter(freeFeatures));
//                feats.clear();
//            }
//
//            for (const auto& feat : freeFeatures) {
//                auto fd = Utils::project_vertex(mesh, vd, feat);
//                auto& faceFeats = get(faceFeatures, fd);
//                faceFeats.insert(feat);
//            }

//            bool v0Seam = get(vSeamMap, v0);
//            bool v1Seam = get(vSeamMap, v1);
//
//            bool using_non_seam_vd = false;
//
//            if (v0Seam && !v1Seam) {
//                stats.total_seam_inner++;
//                if (vd == v1) {
//                    put(vSeamMap, v0, false);
//                    put(vSeamMap, v1, true);
//                    for (auto seamVd : v0Seams) {
//                        if (seamVd != vd) {
//                            auto new_hd = mesh.halfedge(vd, seamVd);
//                            if (new_hd != SurfaceMesh::null_halfedge()) {
//                                put(eSeamMap, mesh.edge(new_hd), true);
//                            }
//                        }
//                    }
//                    using_non_seam_vd = true;
//                }
////                auto p = mesh.point(vd);
//                if (mesh.point(vd) != p0) {
//                    stats.seam_inner_collapses++;
////                    std::stringstream press;
////                    press << "../out/beast-collapse" << stats.seam_inner_collapses << ".obj";
////                    std::ofstream preOut(press.str());
////                    IO::toOBJ(mesh, preOut);
//                }
//            }
//            else if (v1Seam && !v0Seam) {
//                stats.total_seam_inner++;
//                if (vd == v0) {
//                    put(vSeamMap, v1, false);
//                    put(vSeamMap, v0, true);
//                    for (auto seamVd : v1Seams) {
//                        if (seamVd != vd) {
//                            auto new_hd = mesh.halfedge(vd, seamVd);
//                            if (new_hd != SurfaceMesh::null_halfedge()) {
//                                put(eSeamMap, mesh.edge(new_hd), true);
//                            }
//                        }
//                    }
//                    using_non_seam_vd = true;
//                }
////                auto p = mesh.point(vd);
//                if (mesh.point(vd) != p1) {
//                    stats.seam_inner_collapses++;
////                    std::stringstream press;
////                    press << "../out/beast-collapse" << stats.seam_inner_collapses << ".obj";
////                    std::ofstream preOut(press.str());
////                    IO::toOBJ(mesh, preOut);
//                }
//            }
//            else if (v0Seam && v1Seam) {
//                stats.seam_pairs++;
//                auto correctVDs = vd == v0 ? v1Seams : v0Seams;
//                for (auto seamVd : correctVDs) {
//                    if (seamVd != vd) {
//                        auto new_hd = mesh.halfedge(vd, seamVd);
//                        if (new_hd != SurfaceMesh::null_halfedge()) {
//                            put(eSeamMap, mesh.edge(new_hd), true);
//                        }
//                    }
//                }
//
//                if ((vd == v0 && mesh.point(vd) == p1) || (vd == v1 && mesh.point(vd) == p0)) {
//                    using_non_seam_vd = true;
//                }
//            }
//
//            if (v0Seam && v1Seam) {
//                int count = 0;
//                int aCount = 0, bCount = 0;
////                auto& correctSide = vd == v0 ? v1Seams : v0Seams;
//                auto& otherUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;
//                auto& vdUVs = vd == v0 ? v0_hd_newUV : v1_hd_newUV;
//
//                auto& otherSeamUVs = vd == v0 ? v1_seam_uvs : v0_seam_uvs;
//                auto& vdSeamUVs = vd == v0 ? v0_seam_uvs : v1_seam_uvs;
//
//                for (auto hd : halfedges_around_source(vd, mesh)) {
////                    if (stats.processed_edges == test_vd) {
////                        std::stringstream press;
////                        press << "../out/beast-step" << count << "-pe.obj";
////                        std::ofstream preOut(press.str());
////                        IO::toOBJ(mesh, preOut);
////                    }
//
//
////                    std::cout << count << " found in map? ";
////                    auto targetVD = mesh.target(hd);
////                    auto vR_uv = vdUVs.at(vR);
////                    auto vL_uv = vdUVs.at(vL);
////                    Point_2 hd_uv;
////                    bool target_in_other = vdUVs.find(targetVD) != vdUVs.end();
////                    if (target_in_other) {
////                        hd_uv = vdUVs.at(targetVD);
////                    }
//
////                    if (target_in_other && otherSeamUVs.size() == 3 && hd_uv != vR_uv && hd_uv != vL_uv) {
////                        put(uvmap, hd, hd_uv);
////                    }
//                    auto next = get(uvmap, mesh.next(hd));
//                    auto targetVD = mesh.target(hd);
////                    if(stats.processed_edges == test_vd) {
////                        std::cout << "targetVD: " << targetVD << " - " << next.x() << ", " << next.y() << std::endl;
////                    }
//
//                    bool target_in_vd = vdUVs.find(targetVD) != vdUVs.end();
//                    bool target_in_other = otherUVs.find(targetVD) != otherUVs.end();
//                    auto target_in_other_seams = std::find_if(otherSeamUVs.begin(), otherSeamUVs.end(),[targetVD] (std::pair<SM_vertex_descriptor, Point_2> a) {
//                        return a.first == targetVD;
//                    });
//
//                    if (target_in_other_seams != otherSeamUVs.end()) {
//                        auto p = (*target_in_other_seams).second;
//                        put(uvmap, hd, p);
////                        if (stats.processed_edges == test_vd) {
////                            std::cout << "in seams\n" << std::endl;
////                            std::cout << "VD: " << targetVD << " - UV: " << p.x() << ", " << p.y() << "\n"<<std::endl;
////                        }
//                    }
//                    else if (target_in_vd /*&& !target_in_other*/) {
//                        aCount++;
//                        auto p = vdUVs.at(targetVD);
////                        std::cout << "in first. " << p.x() << ", " << p.y();
//                        put(uvmap, hd, p);
//
////                        if (stats.processed_edges == test_vd) {
////                            std::cout << "in vds uv" << std::endl;
////                            if (target_in_other) {
////                                std::cout << "Also in other" << std::endl;
////                            }
////                            std::cout << "VD: " << targetVD << " - UV: " << p.x() << ", " << p.y() << "\n"<<std::endl;
////                        }
//                    }/* else if (stats.processed_edges == test_vd) {
//                        std::cout << "None\n" << std::endl;
//                    }*/
////                    else if (target_in_other) {
////                        auto p = otherUVs.at(targetVD);
//////                        std::cout << "in first. " << p.x() << ", " << p.y();
////                        put(uvmap, hd, p);
////
////                        if (stats.processed_edges == 4985) {
////                            std::cout << "in other uv" << std::endl;
////                            std::cout << "VD: " << targetVD << " - UV: " << p.x() << ", " << p.y() << "\n"<<std::endl;
////                        }
////                    }
///*                    if (target_in_other*//* && !target_in_vd*//*) {
//                        bCount++;
//                        auto p = otherUVs.at(targetVD);
//
//                        put(uvmap, hd, p);
//                    }*/
////                    else if (stats.processed_edges == 4985) {
////                        std::cout << "Not in either uv" << std::endl;
////                        if (target_in_other) {
////                            auto o = otherUVs.at(targetVD);
////                            std::cout << "VD: " << targetVD << " - UV: " << o.x() << ", " << o.y() << "\n"<<std::endl;
////                        }
////                    }
////                    if (stats.processed_edges == test_vd) {
////                        std::stringstream poss;
////                        poss << "../out/beast-step" << count << "-post.obj";
////                        std::ofstream posOut(poss.str());
////                        IO::toOBJ(mesh, posOut);
////                    }
//                    count++;
//                }
//                int x = 1;
//            }
//            else if (get(vSeamMap, vd)) {
//                int count = 0;
//                auto& seamVertexUVs = v0Seam ? v0_hd_newUV : v1_hd_newUV;
//                for (auto hd: halfedges_around_source(vd, mesh)) {
//                    auto& otherUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;
//                    auto& vdUVs = vd == v0 ? v0_hd_newUV : v1_hd_newUV;
//
////                    if (stats.processed_edges == 4470) {
////                        auto next = get(uvmap, mesh.next(hd));
////                        std::cout << "Vd: " << mesh.target(hd) << " - " << next.x() << ", " << next.y() << std::endl;
////                        std::stringstream press;
////                        press << "../out/beast-step" << count << "-pe.obj";
////                        std::ofstream preOut(press.str());
////                        IO::toOBJ(mesh, preOut);
////                    }
//
//                    if (using_non_seam_vd && seamVertexUVs.find(mesh.target(hd)) != seamVertexUVs.end()) {
//                        put(uvmap, hd, seamVertexUVs.at(mesh.target(hd)));
//                    }
//                    else if (otherUVs.find(mesh.target(hd)) != otherUVs.end()) {
//                        put(uvmap, hd, otherUVs.at(mesh.target(hd)));
//                    }
//                    else if (vdUVs.find(mesh.target(hd)) != vdUVs.end()) {
//                        put(uvmap, hd, vdUVs.at(mesh.target(hd)));
//                    }
////                    if (stats.processed_edges == 4470) {
////                        std::stringstream press;
////                        press << "../out/beast-step" << count++ << "-post.obj";
////                        std::ofstream preOut(press.str());
////                        IO::toOBJ(mesh, preOut);
////                    }
//                }
//            }
//            else {
//                for (auto hd: halfedges_around_source(vd, mesh)) {
//                    put(uvmap, hd, v0v1_p_2);
//                }
//            }
////            if (stats.processed_edges == test_vd) {
////                int x = 1;
////            }
//            stats.processed_edges++;
////            if (stats.processed_edges % 500 == 0) {
////            if (stats.processed_edges % 1 == 0 && stats.processed_edges >= 5760 && stats.processed_edges <= 5770) {
////            if (stats.processed_edges % 1 == 0 && stats.processed_edges == test_vd) {
////                    std::stringstream press;
////                    press << "../out/beast-collapse" << stats.processed_edges << ".obj";
////                    std::ofstream preOut(press.str());
////                    IO::toOBJ(mesh, preOut);
////            }
        }

        UV_pmap uvmap;
        CollapseStats& stats;
//        Vertex_bary_map vertBaryMap;
        SM_vertex_descriptor v0, v1, vR, vL;
        SM_halfedge_descriptor v0_v1, v1_v0;
        FeaturePointPtr feature0, feature1;
        std::vector<FeaturePointPtr> freeFeatures;
        Point_3 p0, p1, pR, pL;
        Point_2 p0_2, p1_2, v0v1_p_2, v1v0_p_2;
        Point_2 v0_v1_uv, v1_vL_uv, v1_v0_uv, v0_vR_uv;
        std::unordered_map<SM_vertex_descriptor, Point_2> v0P_2, v1P_2, v0_hd_newUV, v1_hd_newUV;
        std::vector<std::pair<SM_vertex_descriptor, Point_2>> v0_seam_uvs, v1_seam_uvs;

        std::unordered_set<SM_vertex_descriptor> v0ds, v1ds;
        std::unordered_set<SM_vertex_descriptor> v0Seams, v1Seams;
        std::vector<std::pair<SM_vertex_descriptor, SM_halfedge_descriptor>> v0vdhds, v1vdhds;
        std::unordered_set<SM_halfedge_descriptor> v0hds, v1hds;
    };

    template <typename GHPolicies>
    void collapseMesh(const SeamMeshPtr& seamMesh, const SurfaceMeshPtr& mesh, Point2Set& pSet, const double ratio)
    {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        SMS::Count_ratio_stop_predicate<SeamMesh> stop(ratio);
        auto vSeamMap = mesh->property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto eSeamMap = mesh->property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        auto uvmap = mesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto faceFeatures = mesh->property_map<SM_face_descriptor, std::unordered_set<FeaturePointPtr>>("f:features").first;

        Seam_is_constrained_edge_map seam_edge_map(*seamMesh, faceFeatures, uvmap);

        // Garland&Heckbert simplification policies
        typedef typename GHPolicies::Get_cost                                        GH_cost;
        typedef typename GHPolicies::Get_placement                                   GH_placement;
        typedef SMS::Constrained_placement<GH_placement, Seam_is_constrained_edge_map > Constrained_GH_placement;

        GHPolicies gh_policies(*mesh);
        const GH_cost& gh_cost = gh_policies.get_cost();
        const GH_placement& gh_placement = gh_policies.get_placement();
        Constrained_GH_placement placement(seam_edge_map, gh_placement);
        SMS::Bounded_normal_change_filter<> filter;

        CollapseStats stats;
        CollapseVisitor vis(uvmap, stats);

        int r = SMS::edge_collapse(*mesh, stop,
                                   CGAL::parameters::get_cost(gh_cost)
                                           .edge_is_constrained_map(seam_edge_map)
                                           .visitor(vis)
                                           .filter(filter)
                                           .get_placement(placement));

        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::cout << "Time elapsed: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
                  << "ms" << std::endl;
        std::cout << "\nFinished!\n" << r << " edges removed.\n" << edges(*mesh).size() << " final edges.\n";

        stats.print();
    }

    void tessellateFace(const ProcessFacePtr& processFace, SurfaceMesh& sm, const TessLevel& tL,
                        TessEdgeMap& edgeVerts, Stats& stats);

    void projectEdgeVerts(const ProcessFacePtr& processFace, SurfaceMesh& sm, Tree& aabbTree, VertSet& interpolateVerts,
                          TessEdgeMap& edgeVerts, Stats& stats);

    FeatureVerts extractFeatureVertices(const SurfaceMesh& sm, const ProcessFacePtr& tri, const SurfaceMesh& highResMesh, Stats& stats);

    void minBiGraphMatch(const ProcessFacePtr& processFace, const FeatureVerts& featureVerts, Stats& stats);

    class GaussCmp
    {
    public:
        bool operator()(const std::pair<double, TessVertPtr>& a, const std::pair<double, TessVertPtr>& b) const {
            return abs(a.first) < abs(b.first);
        }
    };
    typedef std::pair<double, TessVertPtr> GVPair;
    typedef std::set<GVPair, GaussCmp> FlipResult;
    bool checkFlips(const ProcessFacePtr& processFace, SurfaceMesh& sm, const SurfaceMesh& highResMesh,
                    const Gauss_vertex_pmap& gaussMap, FlipResult& flipped);

    void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh,
                         const Gauss_vertex_pmap& gaussMap, Stats& stats);

    void projectAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& highResMesh, Tree& aabbTree,
                            VertSet& interpolateVerts, Stats& stats);

    void interpolateUnmatched(const VertSet& vertices, SurfaceMesh& sm);

    void calculateUVs(SurfaceMesh& sm_tess, const SurfaceMesh& sm_orig, const ProcessFaceMap& processedFaces);
};


#endif //TESSELLATION_STRATEGY_H
