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
                if (stats.processed_edges == 4985) {
                    int x = 1;
                    auto uv0 = get(uvmap, prof.v0_v1());
                    auto uv1 = get(uvmap, prof.v1_v0());
                    std::cout << "v0: " << v0 << " - " << uv0.x() << ", " << uv0.y() << "\n";
                    std::cout << "v1: " << v1 << " - " << uv1.x() << ", " << uv1.y() << "\n";
                }

                int count = 0;
                for (auto hd : halfedges_around_source(v0, mesh)) {
                    bool edge_is_on_seam = get(eSeamMap, mesh.edge(hd));

                    auto v0_uv = get(uvmap, hd);
                    auto vnext_uv = get(uvmap, mesh.next(hd));

                    if (v0_is_on_seam && v1_is_on_seam) {
                        if (v0_uv == v0_vR_uv) {
                            v0_hd_newUV.insert({mesh.target(hd), v1_v0_uv});
                        }
                        else if (v0_uv == v0_v1_uv) {
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

                    if (v0_is_on_seam && v1_is_on_seam) {
                        if (v1_uv == v1_vL_uv) {
                            v1_hd_newUV.insert({mesh.target(hd), v0_v1_uv});
                        }
                        else if (v1_uv == v1_v0_uv) {
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

            bool v0Seam = get(vSeamMap, v0);
            bool v1Seam = get(vSeamMap, v1);

            bool using_non_seam_vd = false;

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
                    using_non_seam_vd = true;
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
                    using_non_seam_vd = true;
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
                int aCount = 0, bCount = 0;
//                auto& correctSide = vd == v0 ? v1Seams : v0Seams;
                auto& otherUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;
                auto& vdUVs = vd == v0 ? v0_hd_newUV : v1_hd_newUV;

                auto& otherSeamUVs = vd == v0 ? v1_seam_uvs : v0_seam_uvs;
                auto& vdSeamUVs = vd == v0 ? v0_seam_uvs : v1_seam_uvs;

                for (auto hd : halfedges_around_source(vd, mesh)) {
//                    std::stringstream press;
//                    press << "../out/beast-step" << count << "-pe.obj";
//                    std::ofstream preOut(press.str());
//                    IO::toOBJ(mesh, preOut);

//                    std::cout << count << " found in map? ";
//                    auto targetVD = mesh.target(hd);
//                    auto vR_uv = vdUVs.at(vR);
//                    auto vL_uv = vdUVs.at(vL);
//                    Point_2 hd_uv;
//                    bool target_in_other = vdUVs.find(targetVD) != vdUVs.end();
//                    if (target_in_other) {
//                        hd_uv = vdUVs.at(targetVD);
//                    }

//                    if (target_in_other && otherSeamUVs.size() == 3 && hd_uv != vR_uv && hd_uv != vL_uv) {
//                        put(uvmap, hd, hd_uv);
//                    }
                    auto targetVD = mesh.target(hd);
                    bool target_in_vd = vdUVs.find(targetVD) != vdUVs.end();
                    bool target_in_other = otherUVs.find(targetVD) != otherUVs.end();
                    if (target_in_vd /*&& !target_in_other*/) {
                        aCount++;
                        auto p = vdUVs.at(targetVD);
//                        std::cout << "in first. " << p.x() << ", " << p.y();
                        put(uvmap, hd, p);
                    }
/*                    if (target_in_other*//* && !target_in_vd*//*) {
                        bCount++;
                        auto p = otherUVs.at(targetVD);

                        put(uvmap, hd, p);
                    }*/ else if (targetVD == vR){
                        put(uvmap, hd, vd == v0 ? v0_vR_uv : v1_v0_uv);
                    } else if (targetVD == vL) {
                        put(uvmap, hd, vd == v0 ? v0_v1_uv : v1_vL_uv);
                    } else {
//                        std::cout << "None match" << std::endl;
                    }

//                    std::stringstream poss;
//                    poss << "../out/beast-step" << count << "-post.obj";
//                    std::ofstream posOut(poss.str());
//                    IO::toOBJ(mesh, posOut);
//                    count++;
                }
                int x = 1;
            }
            else if (get(vSeamMap, vd)) {
                int count = 0;
                auto& seamVertexUVs = v0Seam ? v0_hd_newUV : v1_hd_newUV;
                for (auto hd: halfedges_around_source(vd, mesh)) {
                    auto& otherUVs = vd == v0 ? v1_hd_newUV : v0_hd_newUV;
                    auto& vdUVs = vd == v0 ? v0_hd_newUV : v1_hd_newUV;

//                    if (stats.processed_edges == 4470) {
//                        auto next = get(uvmap, mesh.next(hd));
//                        std::cout << "Vd: " << mesh.target(hd) << " - " << next.x() << ", " << next.y() << std::endl;
//                        std::stringstream press;
//                        press << "../out/beast-step" << count << "-pe.obj";
//                        std::ofstream preOut(press.str());
//                        IO::toOBJ(mesh, preOut);
//                    }

                    if (using_non_seam_vd && seamVertexUVs.find(mesh.target(hd)) != seamVertexUVs.end()) {
                        put(uvmap, hd, seamVertexUVs.at(mesh.target(hd)));
                    }
                    else if (otherUVs.find(mesh.target(hd)) != otherUVs.end()) {
                        put(uvmap, hd, otherUVs.at(mesh.target(hd)));
                    }
                    else if (vdUVs.find(mesh.target(hd)) != vdUVs.end()) {
                        put(uvmap, hd, vdUVs.at(mesh.target(hd)));
                    }
//                    if (stats.processed_edges == 4470) {
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
//            if (stats.processed_edges % 500 == 0) {
//            if (stats.processed_edges % 1 == 0 && stats.processed_edges >= 4985 && stats.processed_edges <= 5000) {
            if (stats.processed_edges % 1 == 0 && stats.processed_edges == 4985) {
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
        Point_2 v0_v1_uv, v1_vL_uv, v1_v0_uv, v0_vR_uv;
        std::unordered_map<SM_vertex_descriptor, Point_2> v0P_2, v1P_2, v0_hd_newUV, v1_hd_newUV;
        std::vector<std::pair<SM_vertex_descriptor, Point_2>> v0_seam_uvs, v1_seam_uvs;

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
