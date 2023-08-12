//
// Created by dallin on 6/1/23.
//

#include "Decimator.h"

#include "Utils.h"
#include "IO.h"
#include "Debug.h"

Decimator::Decimator(SurfaceMeshPtr mesh, Edge_cost_pmap &edgeCostMap, Feature_face_pmap &featureFaceMap,
                     Vertex_Q_pmap &vertexQMap, Normal_face_pmap &faceNormalMap,
                     Normal_vertex_pmap &vertexNormalMap)
        : edgeCostMap(edgeCostMap), featureFaceMap(featureFaceMap), vertexQMap(vertexQMap),
          faceNormalMap(faceNormalMap), vertexNormalMap(vertexNormalMap) {
    this->mesh = mesh;
}

void Decimator::init() {
    for (const auto& f : mesh->faces()) {
        std::array<glm::vec3, 3> p {};
        std::array<SM_vertex_descriptor, 3> vds;
        int i = 0;
        for (const auto& vd : mesh->vertices_around_face(mesh->halfedge(f))) {
            p.at(i) = Utils::toGLM(mesh->point(vd));
            vds.at(i) = vd;
            i++;
        }

        // Calculate initial vertex Q
        auto n = glm::normalize(glm::cross(p[1]-p[0], p[2]-p[0]));
        for (const auto& vd : vds) {
            auto vQ = get(vertexQMap, vd);
            put(vertexQMap, vd, vQ + SymmetricMatrix(n.x, n.y, n.z, -glm::dot(n, p[0])));
        }
    }

    // Calculate Edge Error
    for (const auto& e: mesh->edges()) {
        auto v0 = mesh->source(mesh->halfedge(e));
        auto v1 = mesh->target(mesh->halfedge(e));

        glm::vec3 p;
        auto error = calculate_error(v0, v1, p);
        put(edgeCostMap, e, error);
    }
}

void Decimator::decimate(double targetRatio) {
    DEBUG_PROFILE_BEGIN_SESSION("Decimate", "DecimateResults.json");
    this->init();

    // Load heap
    for (const auto& e : mesh->edges()) {
        auto src = mesh->source(mesh->halfedge(e));
        auto tgt = mesh->target(mesh->halfedge(e));
        edgeCostHeap.emplace(src, tgt, get(edgeCostMap, e));
    }

    //Main loop
    // Stops when target face ratio is reached, or all faces have max number of faces.
    auto totalNum = double(mesh->number_of_faces());
    for (int i = 0; i < mesh->number_of_edges(); i++) {
        std::cout << "Ratio: " << ((totalNum - mesh->number_of_removed_faces()) / totalNum) << "\n";
        //Break if we are out of edges or reached target ratio
        if (edgeCostHeap.empty() || ((totalNum - mesh->number_of_removed_faces()) / totalNum) <= targetRatio) {
            break;
        }

        auto edgeCandidate = edgeCostHeap.top();
        edgeCostHeap.pop();
        auto pair = edgeCandidate.pair;
        auto edge = mesh->edge(mesh->halfedge(pair.first, pair.second));

        if (mesh->is_removed(edge)) continue;
        if (!check_num_features(edge)) continue;

        auto v0 = mesh->source(mesh->halfedge(edge));
        auto v1 = mesh->target(mesh->halfedge(edge));

        //Calculate collapse position
        glm::vec3 target;
        calculate_error(v0, v1, target);

        //Don't collapse this edge if it flips a triangle.
        if (edge_collapse_causes_flip(v0, v1, target)) continue;
        if (edge_collapse_causes_flip(v1, v0, target)) continue;

        //Gather free feature points
        std::vector<FeaturePointPtr> freeFeatures;
        auto v0Feature = std::make_shared<FeaturePoint>(mesh->point(v0), get(vertexNormalMap, v0));
        auto v1Feature = std::make_shared<FeaturePoint>(mesh->point(v1), get(vertexNormalMap, v1));
        auto& features_L = get(featureFaceMap, mesh->face(mesh->halfedge(edge)));
        auto& features_R = get(featureFaceMap, mesh->face(mesh->opposite(mesh->halfedge(edge))));

        std::copy(features_L.begin(), features_L.end(), std::back_inserter(freeFeatures));
        std::copy(features_R.begin(), features_R.end(), std::back_inserter(freeFeatures));
        freeFeatures.push_back(v0Feature);
        freeFeatures.push_back(v1Feature);

        //Reduce memory used for degenerate faces
        features_L.clear();
        features_R.clear();

        //Collapse the edge and update normals for incident faces and the target vertex
        this->remove_degenerate_edges(edge);

        auto targetVd = this->collapse_and_update_patch(edge, Utils::toPoint3(target));

        //Project all free features into the simplified triangles of this patch
        this->project_features(freeFeatures, targetVd);

        //Update targetVd edge costs in heap
        this->update_costs(targetVd, v0);
    }

    DEBUG_PROFILE_END_SESSION();
}

void Decimator::project_features(std::vector<FeaturePointPtr> &freeFeatures, SM_vertex_descriptor &target) {
    DEBUG_PROFILE_FUNCTION();
    for (auto f : mesh->faces_around_target(mesh->halfedge(target))) {
        auto& feats = get(featureFaceMap, f);
        std::copy(feats.begin(), feats.end(), std::back_inserter(freeFeatures));
        feats.clear();
    }

    for (const auto& feat : freeFeatures) {
        auto fd = this->project_vertex(target, feat);
        auto& faceFeats = get(featureFaceMap, fd);
        faceFeats.insert(feat);
    }
}

int count = 0;
SM_vertex_descriptor Decimator::collapse_and_update_patch(SM_edge_descriptor& e, Point_3 point) {
    DEBUG_PROFILE_FUNCTION();
    //Collapse edge

    auto src = mesh->source(mesh->halfedge(e));
    auto tgt = mesh->target(mesh->halfedge(e));
    if (e.halfedge().idx() == 48916) {
        std::ofstream out("../out/ERROR-PRE.obj");
        IO::toOBJ(*mesh, out);
        int x = 1;
    }


    auto target = CGAL::Euler::collapse_edge(e, *mesh);
    if (e.halfedge().idx() == 48916) {
//        mesh->is_valid(true);
        std::ofstream out("../out/ERROR-POST.obj");
        IO::toOBJ(*mesh, out);
        int x = 1;
    }
//    if (!CGAL::is_triangle_mesh(*mesh)) {
//        std::ofstream out("../out/ERROR.obj");
////                IO::toOBJ(*mesh, out);
////        mesh->collect_garbage();
////        CGAL::IO::write_OBJ(out, *mesh);
//        int x = 1;
//    }

    //Update vertex position
    mesh->point(target) = point;

    Vector vertexNormal {};
    //Update all face normals around collapsed vertex
    for (const auto& f : mesh->faces_around_target(mesh->halfedge(target))) {
        std::array<Point_3, 3> p;
        int i = 0;
        for (const auto& vd : mesh->vertices_around_face(mesh->halfedge(f))) {
            if (i > 2) {
                std::cout << "Mesh Is Triangular: " << CGAL::is_triangle_mesh(*mesh) << std::endl;
//                std::ofstream out("../out/ERROR.obj");
////                IO::toOBJ(*mesh, out);
//                CGAL::IO::write_OBJ(out, *mesh);
                int x = 1;
            }
            p.at(i++) = mesh->point(vd);
        }

        auto fNorm = Utils::compute_face_normal(p.at(0), p.at(1), p.at(2));
        put(faceNormalMap, f, fNorm);
        vertexNormal += fNorm;
    }

    //Update vertex normal
    vertexNormal = Utils::normalize(vertexNormal);
    put(vertexNormalMap, target, vertexNormal);

    return target;
}

SM_face_descriptor Decimator::project_vertex(const SM_vertex_descriptor &vd, const FeaturePointPtr &feature) {
    SM_face_descriptor closest;
    float min_dist = std::numeric_limits<float>::max();

    for (const auto f : mesh->faces_around_target(mesh->halfedge(vd))) {
        std::array<Point_3, 3> vertices;
        int i = 0;
        for (auto v : mesh->vertices_around_face(mesh->halfedge(f))) {
            vertices.at(i++) = mesh->point(v);
        }
        auto fNorm = Utils::toGLM(get(faceNormalMap, f));

        auto a = Utils::toGLM(vertices.at(0));
        auto b = Utils::toGLM(vertices.at(1));
        auto c = Utils::toGLM(vertices.at(2));

        auto d = glm::dot(fNorm, a);
        auto p = Utils::toGLM(feature->p);
        auto vN = Utils::toGLM(feature->n);

        auto t = (d - (glm::dot(fNorm, p))) / glm::dot(fNorm, vN);
        auto proj = p + t*vN;
        auto neg_proj = p + -t*vN;
        auto bary = barycentric(proj, a, b, c);
        auto neg_bary = barycentric(neg_proj, a, b, c);
        if ((glm::all(glm::greaterThanEqual(bary, {0.0, 0.0, 0.0})) &&
             glm::all(glm::lessThanEqual(bary, {1.0, 1.0, 1.0})))
            || (glm::all(glm::greaterThanEqual(neg_bary, {0.0, 0.0, 0.0})) &&
                glm::all(glm::lessThanEqual(neg_bary, {1.0, 1.0, 1.0})))) {
            return f;
        }

        auto bary_dist = glm::length(bary);
        if (glm::length(neg_bary) < bary_dist) {
            bary_dist = glm::length(neg_bary);
        }

        if (bary_dist < min_dist) {
            min_dist = bary_dist;
            closest = f;
        }
    }
    return closest;
}


void Decimator::update_costs(const SM_vertex_descriptor &target, const SM_vertex_descriptor &removeVd) {
    DEBUG_PROFILE_FUNCTION();
    std::vector<std::pair<SM_vertex_descriptor, SM_vertex_descriptor>> pairsToUpdate;
    std::vector<PairCost> newCosts;
    glm::vec3 p;
    for (const auto& hd : halfedges_around_source(target, *mesh)) {
        auto v1 = mesh->target(hd);

        pairsToUpdate.emplace_back(target, v1);
        newCosts.emplace_back(target, v1, calculate_error(target, v1, p));
    }

    //Remove edges for deleted vertex
    edgeCostHeap.remove(removeVd, true);
    //Remove edges that we are updating, then insert new costs for them.
    edgeCostHeap.remove(pairsToUpdate);
    for (auto& cost : newCosts) {
        edgeCostHeap.push(cost);
    }
}

/////// HELPER FUNCTIONS

bool Decimator::edge_collapse_causes_flip(SM_vertex_descriptor &v0, SM_vertex_descriptor &v1, glm::vec3 target) {
    for (const auto& hd : halfedges_around_source(v0, *mesh)) {
        auto f = mesh->face(hd);

        auto vd1 = mesh->target(hd);
        if (vd1 == v1) continue;

        auto vd2 = mesh->target(mesh->next(hd));

        auto p0 = Utils::toGLM(mesh->point(v0));
        auto p1 = Utils::toGLM(mesh->point(vd1));
        auto p2 = Utils::toGLM(mesh->point(vd2));

        glm::vec3 d1 = glm::normalize(Utils::toGLM(mesh->point(vd1))-target);
        glm::vec3 d2 = glm::normalize(Utils::toGLM(mesh->point(vd2))-target);

        if(fabs(glm::dot(d1, d2)) > 0.999) {
            return true;
        }

        glm::vec3 n = glm::normalize(glm::cross(d1, d2));
//        glm::vec3 faceNorm = Utils::toGLM(get(faceNormalMap, f));
        glm::vec3 faceNorm = glm::normalize(glm::cross(p1-p0, p2-p0));
        if(glm::dot(n, faceNorm) < 0.1) {
            return true;
        }
    }
    return false;
}

glm::vec3 Decimator::barycentric(const glm::vec3 &p, const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c){
    glm::vec3 v0 = b-a;
    glm::vec3 v1 = c-a;
    glm::vec3 v2 = p-a;
    double d00 = glm::dot(v0, v0);
    double d01 = glm::dot(v0, v1);
    double d11 = glm::dot(v1, v1);
    double d20 = glm::dot(v2, v0);
    double d21 = glm::dot(v2, v1);
    double denom = d00*d11-d01*d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return {u,v,w};
}

double Decimator::vertex_error(SymmetricMatrix q, glm::vec3 v) {
    auto x = v.x, y = v.y, z = v.z;
    return q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x + q[4]*y*y
           + 2*q[5]*y*z + 2*q[6]*y + q[7]*z*z + 2*q[8]*z + q[9];
}

//Returns true if the edge is contractible
bool Decimator::check_num_features(SM_edge_descriptor &edge) {
//    auto src = mesh->source(mesh->halfedge(edge));
//    auto tgt = mesh->target(mesh->halfedge(edge));
//
//    for (const auto& f : mesh->faces_around_target(mesh->halfedge(src))) {
//        auto featureSet = get(featureFaceMap, f);
//        if (featureSet.size() >= FEATURE_THRESHOLD) {
//            return false;
//        }
//    }
//
//    for (const auto& f : mesh->faces_around_target(mesh->halfedge(tgt))) {
//        auto featureSet = get(featureFaceMap, f);
//        if (featureSet.size() >= FEATURE_THRESHOLD) {
//            return false;
//        }
//    }

    //contractible
    return true;
}

double Decimator::calculate_error(const SM_vertex_descriptor& v0, const SM_vertex_descriptor& v1, glm::vec3 &p_result) {
    // compute interpolated vertex

    SymmetricMatrix q = get(vertexQMap, v0) + get(vertexQMap, v1);
//    bool   border = vertices[id_v1].border & vertices[id_v2].border;
    double error=0;
    double det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);
    if ( det != 0 )
    {

        // q_delta is invertible
        p_result.x = -1/det*(q.det(1, 2, 3, 4, 5, 6, 5, 7 , 8));	// vx = A41/det(q_delta)
        p_result.y =  1/det*(q.det(0, 2, 3, 1, 5, 6, 2, 7 , 8));	// vy = A42/det(q_delta)
        p_result.z = -1/det*(q.det(0, 1, 3, 1, 4, 6, 2, 5,  8));	// vz = A43/det(q_delta)

        error = vertex_error(q, p_result);
    }
    else
    {
        // det = 0 -> try to find best result
        glm::vec3 p0 = Utils::toGLM(mesh->point(v0));
        glm::vec3 p1 = Utils::toGLM(mesh->point(v1));
        glm::vec3 p2 = (p0+p1) / 2.0f;
        double error1 = vertex_error(q, p0);
        double error2 = vertex_error(q, p1);
        double error3 = vertex_error(q, p2);
        error = std::min(error1, std::min(error2, error3));

        if (error1 == error) p_result=p0;
        if (error2 == error) p_result=p1;
        if (error3 == error) p_result=p2;
    }
    return error;
}

void Decimator::remove_degenerate_edges(const SM_edge_descriptor &e) {
//    auto hfwd = mesh->halfedge(e);
//    auto hop = mesh->opposite(hfwd);
//
//    std::vector<SM_edge_descriptor> degenerateEdges;
//    degenerateEdges.reserve(4);
//    degenerateEdges.push_back(mesh->edge(mesh->next(hfwd)));
//    degenerateEdges.push_back(mesh->edge(mesh->prev(hfwd)));
//    degenerateEdges.push_back(mesh->edge(mesh->next(hfwd)));
//    degenerateEdges.push_back(mesh->edge(mesh->prev(hfwd)));
//
//    edgeCostHeap.remove(degenerateEdges);
}
