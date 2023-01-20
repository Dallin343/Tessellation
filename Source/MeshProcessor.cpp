//
// Created by dallin on 5/4/22.
//

#include "MeshProcessor.h"
#include <glm/glm.hpp>

void MeshProcessor::DecimateInit(std::shared_ptr<Mesh> hdMesh) {
    this->mesh = hdMesh;

    // Step 1
    for (auto& vertex : this->mesh->GetVertices()) {
        auto faces = vertex->GetFaces();

        //Compute initial Q matrices
        for (const auto& face : faces) {
            glm::dmat4 Kp = glm::outerProduct(face->GetPlane(), face->GetPlane());
            vertex->Q += Kp;
        }
    }

    // Step 2
    this->SelectPairs();

    // Step 3
    std::for_each(this->validPairs.begin(), this->validPairs.end(), [this](PairPtr& p) {
        this->ComputeTarget(p);
        // Step 4
        this->pairHeap.push(p);
    });

    // Step 5
    std::cout << "Here";
}

glm::dmat4 MeshProcessor::ComputeQ(const VertexPtr& vertex) {

}

// Simple edge contraction
// TODO: Update to include threshold contraction
void MeshProcessor::SelectPairs() {
    std::for_each(this->mesh->GetEdges().begin(), this->mesh->GetEdges().end(), [this](const EdgePtr& e){
        this->validPairs.emplace_back(new Pair(e->GetVertices()[0], e->GetVertices()[1]));
    });
}

void MeshProcessor::ComputeTarget(PairPtr &pair) {
    VertexPtr v1 = pair->GetVertices()[0];
    VertexPtr v2 = pair->GetVertices()[1];

    //Q-bar
    glm::dmat4 Q = v1->Q + v2->Q;
    Q[3] = {0, 0, 0, 1};

    std::optional<glm::dvec3> candidate = FindOptimal(Q);
    if (!candidate.has_value()) candidate = FitLine(Q, v1->GetPosition(), v2->GetPosition());
    if (!candidate.has_value()) candidate = FitLocal(Q, v1->GetPosition(), v2->GetPosition());

    pair->SetCandidate(candidate.value());
    pair->SetCost(VertexError(candidate.value(), Q));
}

std::optional<glm::dvec3> MeshProcessor::FindOptimal(const glm::dmat4 &Q) {
    glm::dmat4 K = Q;
    K[3] = {0, 0, 0, 1};
    if (glm::determinant(K) == 0.0) return {};

    return glm::dvec3(glm::inverse(Q)[3]);
}

std::optional<glm::dvec3> MeshProcessor::FitLine(const glm::dmat4 &Q, const glm::dvec3 &v1, const glm::dvec3 &v2) {
    glm::dvec3 d = v1 - v2;
    glm::dvec3 Qv2 = Q * glm::vec4(v2, 1);
    glm::dvec3 Qd = Q * glm::dvec4(d, 1);

    double denom = 2.0 * glm::dot(d, Qd);
    if (denom == 0.0) return {};

    double a = (glm::dot(d, Qv2) + glm::dot(v2, Qd)) / denom;
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;

    return a * d + v2;
}

std::optional<glm::dvec3> MeshProcessor::FitLocal(const glm::dmat4 &Q, const glm::dvec3 &v1, const glm::dvec3 &v2) {
    glm::dvec3 v3 = (v1 + v2) / 2.0;

    double cost1 = VertexError(v1, Q);
    double cost2 = VertexError(v2, Q);
    double cost3 = VertexError(v3, Q);

    double min = std::min(cost1, std::min(cost2, cost3));
    if (min == cost1) return v1;
    else if (min == cost2) return v2;
    else return v3;
}

double MeshProcessor::VertexError(const glm::dvec3 &v, const glm::dmat4 &Q) {
    glm::dvec4 vec = glm::dvec4{v, 1};
    return glm::dot(vec, Q * vec);
}
