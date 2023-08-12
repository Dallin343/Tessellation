//
// Created by dallin on 6/1/23.
//

#ifndef TESSELLATION_DECIMATOR_H
#define TESSELLATION_DECIMATOR_H

#include "MeshImpl.h"


class Decimator {
public:
    Decimator(SurfaceMeshPtr mesh, Edge_cost_pmap &edgeCostMap, Feature_face_pmap &featureFaceMap,
              Vertex_Q_pmap &vertexQMap, Normal_face_pmap &faceNormalMap,
              Normal_vertex_pmap &vertexNormalMap);

    void init();
    void decimate(double targetRatio);
private:
    static double vertex_error(SymmetricMatrix q, glm::vec3 v);
    void project_features(std::vector<FeaturePointPtr>& freeFeatures, SM_vertex_descriptor& target);
    bool check_num_features(SM_edge_descriptor& edge);
    double calculate_error(const SM_vertex_descriptor& v0, const SM_vertex_descriptor& v1, glm::vec3 &p_result);
    bool edge_collapse_causes_flip(SM_vertex_descriptor& v0, SM_vertex_descriptor& v1, glm::vec3 target);
    SM_vertex_descriptor collapse_and_update_patch(SM_edge_descriptor& e, Point_3 point);
    SM_face_descriptor project_vertex(const SM_vertex_descriptor& vd, const FeaturePointPtr& feature);
    static glm::vec3 barycentric(const glm::vec3 &p, const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c);
    void update_costs(const SM_vertex_descriptor &target, const SM_vertex_descriptor &removeVd);
    void remove_degenerate_edges(const SM_edge_descriptor& e);

    SurfaceMeshPtr mesh;
    Feature_face_pmap& featureFaceMap;
    Edge_cost_pmap& edgeCostMap;
    Vertex_Q_pmap& vertexQMap;
    Normal_face_pmap& faceNormalMap;
    Normal_vertex_pmap& vertexNormalMap;
    EdgeCostHeap edgeCostHeap;
    std::unordered_set<SM_edge_descriptor> constrained_edges;


    const int FEATURE_THRESHOLD = 10;
};


#endif //TESSELLATION_DECIMATOR_H
