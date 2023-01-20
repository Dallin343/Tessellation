//
// Created by dallin on 5/4/22.
//

#ifndef TESSELLATION_MESHPROCESSOR_H
#define TESSELLATION_MESHPROCESSOR_H

#include "Model.h"
#include <queue>
#include <optional>

/**
 * Simplification Algorithm
 *  M. Garland, P. S. Heckbert, Surface simplification using quadric error metrics
 *  https://www.cs.cmu.edu/~./garland/Papers/quadrics.pdf
 * 1. Compute the Q matrices for all the initial vertices.
    2. Select all valid pairs.
    3. Compute the optimal contraction target v¯ for each valid pair
    (v1, v2 ). The error v¯T(Q1 +Q2 )v¯ of this target vertex becomes
    the cost of contracting that pair.
    4. Place all the pairs in a heap keyed on cost with the minimum
    cost pair at the top.
    5. Iteratively remove the pair (v1, v2 ) of least cost from the heap,
    contract this pair, and update the costs of all valid pairs involving v1.
 */

class Comparator {
    bool Reverse;
public:
    Comparator(const bool& reverse = false){
        Reverse=reverse;
    }
    bool operator() (const PairPtr& a, const PairPtr& b) const {
        if (Reverse) return (a->GetCost() > b->GetCost());
        else return (a->GetCost() < b->GetCost());
    }
};

class MeshProcessor {
public:

    void DecimateInit(std::shared_ptr<Mesh> hdMesh);

private:
    const double threshold = 0.0;

    std::priority_queue<PairPtr, std::vector<PairPtr>, Comparator> pairHeap;

    std::shared_ptr<Mesh> mesh;
    std::vector<PairPtr> validPairs;

    glm::dmat4 ComputeQ(const VertexPtr& vertex);
    void SelectPairs();
    void ComputeTarget(PairPtr &pair);

    static double VertexError(const glm::dvec3& v, const glm::dmat4& Q);
    std::optional<glm::dvec3> FindOptimal(const glm::dmat4&);
    std::optional<glm::dvec3> FitLine(const glm::dmat4 &Q, const glm::dvec3 &v1, const glm::dvec3 &v2);
    std::optional<glm::dvec3> FitLocal(const glm::dmat4 &Q, const glm::dvec3 &v1, const glm::dvec3 &v2);
};


#endif //TESSELLATION_MESHPROCESSOR_H
