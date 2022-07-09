//
// Created by dallin on 5/4/22.
//

#ifndef TESSELLATION_MESHPROCESSOR_H
#define TESSELLATION_MESHPROCESSOR_H

#include "Model.h"

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

class MeshProcessor {

    void DecimateInit(Mesh *hdMesh);

private:
//
//    double quadricError(vertices)
    Mesh *mesh;

    glm::dmat4 ComputeQ(const VPtr& vertex);
};


#endif //TESSELLATION_MESHPROCESSOR_H
