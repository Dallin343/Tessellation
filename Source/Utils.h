//
// Created by dallin on 3/4/23.
//

#ifndef TESSELLATION_UTILS_H
#define TESSELLATION_UTILS_H
#include "MeshImpl.h"
#include <glm/glm.hpp>
#include <optional>

namespace Utils {
    std::optional<glm::vec3> barycentric(glm::vec3 point, glm::vec3 t0, glm::vec3 t1, glm::vec3 t2);
    glm::vec3 barycentric(glm::vec2 p, glm::vec2 a, glm::vec2 b, glm::vec2 c);

    double barycentric_distance(glm::vec3 p, glm::vec3 q, double t1t2, double t0t2, double t0t1);
    Vector compute_face_normal(const TessFacePtr& face, const SurfaceMesh& sm);
    Vector compute_face_normal(const Point_3& v0, const Point_3& v1, const Point_3& v2);

    Vector lerp(const Vector& a, const Vector& b, double t);
    Vector lerp(const Vector& A, const Vector& B, const Vector&C, double tA, double tB, double tC);
    Vector normalize(const Vector& V);

    glm::vec3 toGLM(Kernel::Vector_3 p);
    glm::vec3 toGLM(Kernel::Point_3 p);
    glm::vec2 toGLM(Point_2 p);

    Point_3 toPoint3(glm::vec3 p);

    Gauss_vertex_pmap CalculateGaussianCurvature(SurfaceMesh& mesh);
    std::optional<Point_3> findIntersection(const Point_3& p, const Vector& nrm, const Tree& tree);

    std::string SectionString(AssigningSection section);


    //Not used right now
    SeamMeshPtr UnwrapMesh(const SurfaceMeshPtr& sm, const std::string& selectionsFile);



};


#endif //TESSELLATION_UTILS_H
