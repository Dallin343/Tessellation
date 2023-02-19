//
// Created by dallin on 2/14/23.
//

#ifndef TESSELLATION_GLOBALPARAMETERIZATION_H
#define TESSELLATION_GLOBALPARAMETERIZATION_H


#include "MeshImpl.h"
#include <complex>
#include <glm/glm.hpp>

typedef std::complex<double> dcomplex;
typedef std::array<dcomplex, 3> complexface;
typedef std::unordered_map<SM_face_descriptor, complexface> ApproxZMap;

class GlobalParameterization {
public:

//private:
    static ApproxZMap conformalMap(const SM_vertex_descriptor& vd, const SurfaceMesh& mesh);
};


#endif //TESSELLATION_GLOBALPARAMETERIZATION_H
