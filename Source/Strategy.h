//
// Created by dallin on 3/1/23.
//

#ifndef TESSELLATION_STRATEGY_H
#define TESSELLATION_STRATEGY_H

#include "MeshImpl.h"
#include "IO.h"
#include "Utils.h"

namespace Strategy {
    struct Stats {
        unsigned int invalid_tessFaces = 0;
        unsigned int processed_edges = 0;
        unsigned int mv_edge_cases = 0;
        unsigned int pv_edge_cases = 0;

        void print() const {
            std::cout << "\n== Strategy Stats ==\n";
            std::cout << "Invalid faces during tessellation: " << invalid_tessFaces << "\n";
            std::cout << "Edges tessellated: " << processed_edges << "\n";
            std::cout << "MoveAndValidate edge cases: " << mv_edge_cases << "\n";
            std::cout << "ProjectAndValidate edge cases: " << pv_edge_cases << "\n";
            std::cout << "====\n\n";
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
            if (placement) {
                auto newPoint = placement.value();
                const auto& mesh = prof.surface_mesh();
                const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
                const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;

                p0 = prof.p0();
                p1 = prof.p1();

                glm::vec3 glmP0 = {p0.x(), p0.y(), p0.z()};
                glm::vec3 glmP1 = {p1.x(), p1.y(), p1.z()};
                glm::vec3 glmP = {newPoint.x(), newPoint.y(), newPoint.z()};

                float modelEdgeLen = glm::length(glmP1 - glmP0);
                float p_p0Len = glm::length(glmP - glmP0);
                float pp0Ratio = p_p0Len / modelEdgeLen;

                auto v0_v1_uv = get(uvmap, prof.v0_v1());
                auto v1_vL_uv = get(uvmap, prof.v1_vL());

                auto v1_v0_uv = get(uvmap, prof.v1_v0());
                auto v0_vR_uv = get(uvmap, prof.v0_vR());

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
                v0ds.clear();
                v1ds.clear();
                v0Seams.clear();
                v1Seams.clear();
                v0P_2.clear();
                v1P_2.clear();
                v0_hd_newUV.clear();
                v1_hd_newUV.clear();

                bool v0_is_on_seam = get(vSeamMap, v0);
                bool v1_is_on_seam = get(vSeamMap, v1);
                if (stats.processed_edges == 1136) {
                    int x = 1;
                }

                int count = 0;
                for (auto hd : halfedges_around_source(v0, mesh)) {
                    bool edge_is_on_seam = get(eSeamMap, mesh.edge(hd));

                    auto v0_uv = get(uvmap, hd);
                    auto vnext_uv = get(uvmap, mesh.next(hd));

                    if (v0_is_on_seam && v1_is_on_seam) {
                        glm::vec2 v0_uv_glm = Utils::toGLM(v0_uv);
                        glm::vec2 vnext_uv_glm = Utils::toGLM(vnext_uv);

                        auto vec = vnext_uv_glm - v0_uv_glm;
                        glm::vec2 newUV = v0_uv_glm + (vec * pp0Ratio);
                        v0_hd_newUV.insert({mesh.target(hd), {newUV.x, newUV.y}});
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
                        v0Seams.insert({mesh.target(hd)});
                    }
                    count++;
                }

                count = 0;
                for (auto hd : halfedges_around_source(v1, mesh)) {
                    bool edge_is_on_seam = get(eSeamMap, mesh.edge(hd));

                    auto v1_uv = get(uvmap, hd);
                    auto vnext_uv = get(uvmap, mesh.next(hd));

                    if (v0_is_on_seam && v1_is_on_seam) {
                        glm::vec2 v1_uv_glm = Utils::toGLM(v1_uv);
                        glm::vec2 vnext_uv_glm = Utils::toGLM(vnext_uv);

                        auto vec = vnext_uv_glm - v1_uv_glm;
                        glm::vec2 newUV = v1_uv_glm + (vec * pp0Ratio);
                        v1_hd_newUV.insert({mesh.target(hd), {newUV.x, newUV.y}});
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
                        v1Seams.insert({mesh.target(hd)});
                    }
                    count++;
                }


//                if (get(vSeamMap, v0) && get(vSeamMap, v1)) {
//                    for (auto hd : halfedges_around_source(v0, mesh)) {
//                        if (get(eSeamMap, mesh.edge(hd))) {
//                            v0Seams.insert(mesh.target(hd));
//                        }
//                        auto v0_p2 = get(uvmap, hd);
//                        auto vnext_p2 = get(uvmap, mesh.next(hd));
//
//                        glm::vec2 glmP0_2 = Utils::toGLM(v0_p2);
//                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);
//
//                        auto p0Vec = glmP1_2 - glmP0_2;
//                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
//                        v0P_2.insert({mesh.target(hd), {newPoint_2.x, newPoint_2.y}});
//                    }
//                    for (auto hd : halfedges_around_source(v1, mesh)) {
//                        if (get(eSeamMap, mesh.edge(hd))) {
//                            v1Seams.insert(mesh.target(hd));
//                        }
//                        auto v1_p2 = get(uvmap, hd);
//                        auto vnext_p2 = get(uvmap, mesh.next(hd));
//
//                        glm::vec2 glmP0_2 = Utils::toGLM(v1_p2);
//                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);
//
//                        auto p0Vec = glmP1_2 - glmP0_2;
//                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
//                        v1P_2.insert({mesh.target(hd), {newPoint_2.x, newPoint_2.y}});
//                    }
//                }
//                else if (get(vSeamMap, v0)) {
//                    for (auto hd: halfedges_around_source(v1, mesh)) {
//                        auto edge = mesh.edge(prof.vL_v0());
//                        auto edgeOnSeam = get(eSeamMap, edge);
//
//                        if ((hd != prof.v1_vL() || !get(vSeamMap, prof.vL())) || (hd == prof.v1_vL() && !edgeOnSeam)) {
//                            v0ds.insert(mesh.target(hd));
//                        }
//                    }
//                    for (auto hd : halfedges_around_source(v0, mesh)) {
//                        if (get(eSeamMap, mesh.edge(hd))) {
//                            v0Seams.insert(mesh.target(hd));
//                        }
//
//                        if (mesh.target(hd) != v1) {
//                            auto v0_p2 = get(uvmap, hd);
//                            v0P_2.insert({mesh.target(hd), v0_p2});
//                        }
//                    }
//                }
//                else if (get(vSeamMap, v1)) {
//                    for (auto hd: halfedges_around_source(v0, mesh)) {
//                        auto edge = mesh.edge(prof.vR_v1());
//                        auto edgeOnSeam = get(eSeamMap, edge);
//
//                        if ((hd != prof.v0_vR() || !get(vSeamMap, prof.vR())) || (hd == prof.v0_vR() && !edgeOnSeam)) {
//                            v0ds.insert(mesh.target(hd));
//                        }
//
//                        if (mesh.target(hd) != v1) {
//                            auto v0_p2 = get(uvmap, hd);
//                            v0P_2.insert({mesh.target(hd), v0_p2});
//                        }
//                    }
//                    for (auto hd : halfedges_around_source(v1, mesh)) {
//                        if (get(eSeamMap, mesh.edge(hd))) {
//                            v1Seams.insert(mesh.target(hd));
//                        }
//
//                        if (mesh.target(hd) != v0) {
//                            auto v1_p2 = get(uvmap, hd);
//                            v1P_2.insert({mesh.target(hd), v1_p2});
//                        }
//
////                        auto vnext_p2 = get(uvmap, mesh.next(hd));
////
////                        glm::vec2 glmP0_2 = Utils::toGLM(v1_p2);
////                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);
////
////                        auto p0Vec = glmP1_2 - glmP0_2;
////                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
//
//                    }
//                }
            }
        }

        // Called after each edge has been collapsed
        void OnCollapsed(const EdgeProfile& prof, vertex_descriptor vd)
        {
            const auto& mesh = prof.surface_mesh();
            const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
            const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;

            auto v0Seam = get(vSeamMap, v0);
            auto v1Seam = get(vSeamMap, v1);

            if (v0Seam && !v1Seam) {
                stats.total_seam_inner++;
                if (vd == v1) {
                    put(vSeamMap, v0, false);
                    put(vSeamMap, v1, true);
                    for (auto seamVd : v0Seams) {
                        if (seamVd != vd) {
                            auto new_hd = mesh.halfedge(vd, seamVd);
                            if (new_hd != SurfaceMesh::null_halfedge()) {
                                put(eSeamMap, mesh.edge(new_hd), true);
                            }
                        }
                    }
                }
//                auto p = mesh.point(vd);
                if (mesh.point(vd) != p0) {
                    stats.seam_inner_collapses++;
//                    std::stringstream press;
//                    press << "../out/beast-collapse" << stats.seam_inner_collapses << ".obj";
//                    std::ofstream preOut(press.str());
//                    IO::toOBJ(mesh, preOut);
                }
            }
            else if (v1Seam && !v0Seam) {
                stats.total_seam_inner++;
                if (vd == v0) {
                    put(vSeamMap, v1, false);
                    put(vSeamMap, v0, true);
                    for (auto seamVd : v1Seams) {
                        if (seamVd != vd) {
                            auto new_hd = mesh.halfedge(vd, seamVd);
                            if (new_hd != SurfaceMesh::null_halfedge()) {
                                put(eSeamMap, mesh.edge(new_hd), true);
                            }
                        }
                    }
                }
//                auto p = mesh.point(vd);
                if (mesh.point(vd) != p1) {
                    stats.seam_inner_collapses++;
//                    std::stringstream press;
//                    press << "../out/beast-collapse" << stats.seam_inner_collapses << ".obj";
//                    std::ofstream preOut(press.str());
//                    IO::toOBJ(mesh, preOut);
                }
            }
            else if (v0Seam && v1Seam) {
                stats.seam_pairs++;
                auto correctVDs = vd == v0 ? v1Seams : v0Seams;
                for (auto seamVd : correctVDs) {
                    if (seamVd != vd) {
                        auto new_hd = mesh.halfedge(vd, seamVd);
                        if (new_hd != SurfaceMesh::null_halfedge()) {
                            put(eSeamMap, mesh.edge(new_hd), true);
                        }
                    }
                }
            }

            if (v0Seam && v1Seam) {
                int count = 0;
//                auto& correctSide = vd == v0 ? v1Seams : v0Seams;
                auto& correctUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;

                for (auto hd : halfedges_around_source(vd, mesh)) {
                    std::stringstream press;
                    press << "../out/beast-step" << count << "-pe.obj";
                    std::ofstream preOut(press.str());
                    IO::toOBJ(mesh, preOut);

                    std::cout << count << " found in map? ";
                    if (correctUVs.find(mesh.target(hd)) != correctUVs.end()) {
                        auto p = correctUVs.at(mesh.target(hd));
                        std::cout << "yes. " << p.x() << ", " << p.y();
                        put(uvmap, hd, p);
                    }
                    std::cout << std::endl;

                    std::stringstream poss;
                    poss << "../out/beast-step" << count << "-post.obj";
                    std::ofstream posOut(poss.str());
                    IO::toOBJ(mesh, posOut);
                    count++;
                }
                int x = 1;
            }
            else if (get(vSeamMap, vd)) {
                int count = 0;
                for (auto hd: halfedges_around_source(vd, mesh)) {
                    auto& correctUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;

//                    if (stats.processed_edges == 1136) {
//                        std::stringstream press;
//                        press << "../out/beast-step" << count << "-pe.obj";
//                        std::ofstream preOut(press.str());
//                        IO::toOBJ(mesh, preOut);
//                    }

                    if (correctUVs.find(mesh.target(hd)) != correctUVs.end()) {
                        put(uvmap, hd, correctUVs.at(mesh.target(hd)));
                    }
//                    if (stats.processed_edges == 1136) {
//                        std::stringstream press;
//                        press << "../out/beast-step" << count++ << "-post.obj";
//                        std::ofstream preOut(press.str());
//                        IO::toOBJ(mesh, preOut);
//                    }
                }
            }
            else {
                for (auto hd: halfedges_around_source(vd, mesh)) {
                    put(uvmap, hd, v0v1_p_2);
                }
            }
           stats.processed_edges++;
//            if (stats.processed_edges % 1 == 0 && stats.processed_edges >= 1130 && stats.processed_edges <= 1140) {
//                std::stringstream press;
//                press << "../out/beast-collapse" << stats.processed_edges << ".obj";
//                std::ofstream preOut(press.str());
//                IO::toOBJ(mesh, preOut);
//            }
        }

        UV_pmap uvmap;
        CollapseStats& stats;
//        Vertex_bary_map vertBaryMap;
        SM_vertex_descriptor v0, v1, vR, vL;
        SM_halfedge_descriptor v0_v1, v1_v0;
        Point_3 p0, p1, pR, pL;
        Point_2 p0_2, p1_2, v0v1_p_2, v1v0_p_2;
        std::unordered_map<SM_vertex_descriptor, Point_2> v0P_2, v1P_2, v0_hd_newUV, v1_hd_newUV;

        std::unordered_set<SM_vertex_descriptor> v0ds, v1ds;
        std::unordered_set<SM_vertex_descriptor> v0Seams, v1Seams;
        std::vector<std::pair<SM_vertex_descriptor, SM_halfedge_descriptor>> v0vdhds, v1vdhds;
        std::unordered_set<SM_halfedge_descriptor> v0hds, v1hds;
    };

    template <typename GHPolicies>
    void collapseMesh(const SeamMeshPtr& seamMesh, const SurfaceMeshPtr& mesh, const double ratio)
    {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        SMS::Count_ratio_stop_predicate<SeamMesh> stop(ratio);
        auto vSeamMap = mesh->property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto eSeamMap = mesh->property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        Seam_is_constrained_edge_map seam_edge_map(*seamMesh, vSeamMap, eSeamMap);

        // Garland&Heckbert simplification policies
        typedef typename GHPolicies::Get_cost                                        GH_cost;
        typedef typename GHPolicies::Get_placement                                   GH_placement;
        typedef SMS::Constrained_placement<GH_placement, Seam_is_constrained_edge_map > Constrained_GH_placement;

        GHPolicies gh_policies(*mesh);
        const GH_cost& gh_cost = gh_policies.get_cost();
        const GH_placement& gh_placement = gh_policies.get_placement();
        Constrained_GH_placement placement(seam_edge_map, gh_placement);
        SMS::Bounded_normal_change_filter<> filter;

        auto uvmap = mesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        CollapseStats stats;
        CollapseVisitor vis(uvmap, stats);

        int r = SMS::edge_collapse(*mesh, stop,
                                   CGAL::parameters::get_cost(gh_cost)
//                                           .edge_is_constrained_map(seam_edge_map)
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

    FeatureVerts extractFeatureVertices(Point2Set& pSet, const ProcessFacePtr& tri, const SurfaceMesh& highResMesh, Stats& stats);

    void minBiGraphMatch(const ProcessFacePtr& processFace, const FeatureVerts& featureVerts, Stats& stats);

    void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig,
                         const Gauss_vertex_pmap& gaussMap, Stats& stats);

    void projectAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig, Tree& aabbTree,
                            VertSet& interpolateVerts, Stats& stats);
};


#endif //TESSELLATION_STRATEGY_H
