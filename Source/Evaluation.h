//
// Created by dallin on 9/2/23.
//

#ifndef TESSELLATION_EVALUATION_H
#define TESSELLATION_EVALUATION_H

#include "MeshImpl.h"

struct EvalResult {
    double error;
    double renderTime; // milliseconds
    double memory; // MB
};

class Evaluation {
public:
    static std::vector<double> error(const SurfaceMeshPtr& original, const std::vector<SurfaceMeshPtr>& tests);
};


#endif //TESSELLATION_EVALUATION_H
