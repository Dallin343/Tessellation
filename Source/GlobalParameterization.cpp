//
// Created by dallin on 2/14/23.
//

#include "GlobalParameterization.h"
#include <cmath>


//http://multires.caltech.edu/pubs/maps.pdf
//Flattening
ApproxZMap GlobalParameterization::conformalMap(const SM_vertex_descriptor &vd, const SurfaceMesh &mesh) {
    Point_3 p_i = mesh.point(vd);
    std::vector<SM_vertex_descriptor> j;
    std::unordered_map<SM_vertex_descriptor, dcomplex> u_i;
    u_i.insert({vd, 0.0});

    std::unordered_map<SM_vertex_descriptor, Point_3> p;
    std::vector<double> theta;

    //Create list of 1-ring points around vd;
    for (auto hd : halfedges_around_source(vd, mesh)) {
        auto jk = mesh.target(hd);
        j.push_back(jk);
        p.insert({jk, mesh.point(jk)});
    }

    //Calculate sum of angles for each k
    double angle_sum = 0.0;
    for (unsigned int l = 0; l < j.size(); l++) {
        // p_j_{l-1}
        Point_3 pl1;
        if (l == 0) {
            pl1 = p.at(j.back());
        } else {
            pl1 = p.at(j.at(l-1));
        }

        // p_j_l
        Point_3 pl = p.at(j.at(l));
        angle_sum += CGAL::approximate_angle(pl1, p_i, pl);
        theta.push_back(angle_sum);
    }

    double theta_k = theta.back();
    double a = (2.0 * M_PIf64) / theta.back();
    for (auto hd : halfedges_around_source(vd, mesh)) {
        auto Jk_vd = mesh.target(hd);

        Point_3 p_j_k = mesh.point(Jk_vd);
        Vector pi_pjk = p_i - p_j_k;

        // r_k = ||p_i - p_j_k||
        double r_k = CGAL::approximate_sqrt(pi_pjk.squared_length());

        dcomplex i = -1;
        i = sqrt(i);

        dcomplex u_pjk = pow(r_k, a) * std::exp(i*theta_k*a);
        u_i.insert({Jk_vd, u_pjk});
    }

    ApproxZMap map;
    for (auto hd : halfedges_around_source(vd, mesh)) {
        auto fd = mesh.face(hd);
        auto face = complexface();
        int idx = 0;
        for (auto faceVD : vertices_around_face(hd, mesh)) {
            face.at(idx++) = u_i.at(faceVD);
        }
        map.insert({fd, face});
    }
    return map;
}

std::optional<glm::vec<3, dcomplex, glm::defaultp>> zMapBarycentric(const ApproxZMap& map, glm::dvec3 p) {

}
