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

                v0 = prof.v0();
                v1 = prof.v1();
                v0ds.clear();
                v1ds.clear();
                v0Seams.clear();
                v1Seams.clear();
                v0P_2.clear();
                v1P_2.clear();


                if (get(vSeamMap, v0) && get(vSeamMap, v1)) {
                    for (auto hd : halfedges_around_source(v0, mesh)) {
                        if (get(eSeamMap, mesh.edge(hd))) {
                            v0Seams.insert(mesh.target(hd));
                        }
                        auto v0_p2 = get(uvmap, hd);
                        auto vnext_p2 = get(uvmap, mesh.next(hd));

                        glm::vec2 glmP0_2 = Utils::toGLM(v0_p2);
                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);

                        auto p0Vec = glmP1_2 - glmP0_2;
                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
                        v0P_2.insert({mesh.target(hd), {newPoint_2.x, newPoint_2.y}});
                    }
                    for (auto hd : halfedges_around_source(v1, mesh)) {
                        if (get(eSeamMap, mesh.edge(hd))) {
                            v1Seams.insert(mesh.target(hd));
                        }
                        auto v1_p2 = get(uvmap, hd);
                        auto vnext_p2 = get(uvmap, mesh.next(hd));

                        glm::vec2 glmP0_2 = Utils::toGLM(v1_p2);
                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);

                        auto p0Vec = glmP1_2 - glmP0_2;
                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
                        v1P_2.insert({mesh.target(hd), {newPoint_2.x, newPoint_2.y}});
                    }
                }
                else if (get(vSeamMap, v0)) {
                    for (auto hd: halfedges_around_source(v1, mesh)) {
                        auto edge = mesh.edge(prof.vL_v0());
                        auto edgeOnSeam = get(eSeamMap, edge);

                        if ((hd != prof.v1_vL() || !get(vSeamMap, prof.vL())) || (hd == prof.v1_vL() && !edgeOnSeam)) {
                            v0ds.insert(mesh.target(hd));
                        }
                    }
                    for (auto hd : halfedges_around_source(v0, mesh)) {
                        if (get(eSeamMap, mesh.edge(hd))) {
                            v0Seams.insert(mesh.target(hd));
                        }

                        if (mesh.target(hd) != v1) {
                            auto v0_p2 = get(uvmap, hd);
                            v0P_2.insert({mesh.target(hd), v0_p2});
                        }
//                        auto vnext_p2 = get(uvmap, mesh.next(hd));
//
//                        glm::vec2 glmP0_2 = Utils::toGLM(v0_p2);
//                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);
//
//                        auto p0Vec = glmP1_2 - glmP0_2;
//                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
                    }
                } else if (get(vSeamMap, v1)) {
                    for (auto hd: halfedges_around_source(v0, mesh)) {
                        auto edge = mesh.edge(prof.vR_v1());
                        auto edgeOnSeam = get(eSeamMap, edge);

                        if ((hd != prof.v0_vR() || !get(vSeamMap, prof.vR())) || (hd == prof.v0_vR() && !edgeOnSeam)) {
                            v0ds.insert(mesh.target(hd));
                        }

                        if (mesh.target(hd) != v1) {
                            auto v0_p2 = get(uvmap, hd);
                            v0P_2.insert({mesh.target(hd), v0_p2});
                        }
                    }
                    for (auto hd : halfedges_around_source(v1, mesh)) {
                        if (get(eSeamMap, mesh.edge(hd))) {
                            v1Seams.insert(mesh.target(hd));
                        }

                        if (mesh.target(hd) != v0) {
                            auto v1_p2 = get(uvmap, hd);
                            v1P_2.insert({mesh.target(hd), v1_p2});
                        }

//                        auto vnext_p2 = get(uvmap, mesh.next(hd));
//
//                        glm::vec2 glmP0_2 = Utils::toGLM(v1_p2);
//                        glm::vec2 glmP1_2 = Utils::toGLM(vnext_p2);
//
//                        auto p0Vec = glmP1_2 - glmP0_2;
//                        glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);

                    }
                }

                auto v0hd = prof.v0_v1();
                auto v1hd = prof.v1_vL();
                p0_2 = get(uvmap, v0hd);
                p1_2 = get(uvmap, v1hd);


                glm::vec2 glmP0_2 = {p0_2.x(), p0_2.y()};
                glm::vec2 glmP1_2 = {p1_2.x(), p1_2.y()};

                auto p0Vec = glmP1_2 - glmP0_2;
                glm::vec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
                v0v1_p_2 = {newPoint_2.x, newPoint_2.y};
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
            } else if (v1Seam && !v0Seam) {
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
            } else if (v0Seam && v1Seam) {
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
                auto& correctSide = vd == v0 ? v1Seams : v0Seams;
                auto& correctSeamUVs = vd == v0 ? v1P_2 : v0P_2;
//                std::stringstream press;
//                press << "../out/beast-pre-seam-collapse" << stats.seam_pairs << ".obj";
//                std::ofstream preOut(press.str());
//                IO::toOBJ(mesh, preOut);

                for (auto hd : halfedges_around_source(vd, mesh)) {
                    if (correctSeamUVs.find(mesh.target(hd)) != correctSeamUVs.end()) {
                        auto p = correctSeamUVs.at(mesh.target(hd));
                        put(uvmap, hd, p);
                    }
//                    if (correctSide.find(mesh.target(hd)) != correctSide.end()) {
//
//                        put(uvmap, hd, v0v1_p_2);
//                        auto opposite_hd = mesh.next(mesh.opposite(hd));
//                        put(uvmap, opposite_hd);
//                    }
                }
//                std::stringstream ss;
//                ss << "../out/beast-post-seam-collapse" << stats.seam_pairs << ".obj";
//                std::ofstream postOut(ss.str());
//                IO::toOBJ(mesh, postOut);

//                if (stats.processed_edges > 1000) {
//                    int x = 0;
//                }
            }
            else if (get(vSeamMap, vd)) {
                int count = 0;
                for (auto hd: halfedges_around_source(vd, mesh)) {
                    auto& correctSeamUVs = vd == v0 ? v1P_2 : v0P_2;

                    if (stats.processed_edges == 1136) {
                        std::stringstream press;
                        press << "../out/beast-step" << count << "-pe.obj";
                        std::ofstream preOut(press.str());
                        IO::toOBJ(mesh, preOut);
                    }
                    if (v0ds.find(mesh.target(hd)) != v0ds.end() && correctSeamUVs.find(mesh.target(hd)) == correctSeamUVs.end()) {
                        put(uvmap, hd, v0v1_p_2);
                    }
                    if (correctSeamUVs.find(mesh.target(hd)) != correctSeamUVs.end()) {
                        put(uvmap, hd, correctSeamUVs.at(mesh.target(hd)));
                    }
                    if (stats.processed_edges == 1136) {
                        std::stringstream press;
                        press << "../out/beast-step" << count++ << "-post.obj";
                        std::ofstream preOut(press.str());
                        IO::toOBJ(mesh, preOut);
                    }
                }
            }
            else {
                for (auto hd: halfedges_around_source(vd, mesh)) {
                    put(uvmap, hd, v0v1_p_2);
                }
            }
            stats.processed_edges++;
            if (stats.processed_edges % 1 == 0 && stats.processed_edges >= 1130 && stats.processed_edges <= 1140) {
                std::stringstream press;
                press << "../out/beast-collapse" << stats.processed_edges << ".obj";
                std::ofstream preOut(press.str());
                IO::toOBJ(mesh, preOut);
            }
        }

        UV_pmap uvmap;
        CollapseStats& stats;
//        Vertex_bary_map vertBaryMap;
        SM_vertex_descriptor v0, v1, vR, vL;
        SM_halfedge_descriptor v0_v1, v1_v0;
        Point_3 p0, p1, pR, pL;
        Point_2 p0_2, p1_2, v0v1_p_2, v1v0_p_2;
        std::unordered_map<SM_vertex_descriptor, Point_2> v0P_2, v1P_2;

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
