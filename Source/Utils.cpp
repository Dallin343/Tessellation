//
// Created by dallin on 3/4/23.
//

#include "Utils.h"
#include <igl/gaussian_curvature.h>

namespace Utils {
    bool nearly_equal(float a, float b, float epsilon) {
        if (a == b) return true;
        return std::abs(a-b) < epsilon;
    }

    glm::vec3 barycentric(glm::vec3 point, glm::vec3 t0, glm::vec3 t1, glm::vec3 t2) {
        glm::vec3 u = t1 - t0;
        glm::vec3 v = t2 - t0;
        glm::vec3 n = glm::cross(u, v);
        glm::vec3 w = point - t0;

        double gamma = glm::dot(glm::cross(u, w), n) / glm::dot(n, n);
        double beta = glm::dot(glm::cross(w, v), n) / glm::dot(n, n);
        double alpha = 1.0 - gamma - beta;
        return {alpha, beta, gamma};
    }

    glm::vec3 barycentric(glm::vec2 p, glm::vec2 a, glm::vec2 b, glm::vec2 c) {
        glm::vec2 v0 = b - a;
        glm::vec2 v1 = c - a;
        glm::vec2 v2 = p - a;
        double d00 = glm::dot(v0, v0);
        double d01 = glm::dot(v0, v1);
        double d11 = glm::dot(v1, v1);
        double d20 = glm::dot(v2, v0);
        double d21 = glm::dot(v2, v1);
        double denom = d00 * d11 - d01 * d01;
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;
        return {u, v, w};
    }

    double barycentric_distance(glm::vec3 p, glm::vec3 q) {
        auto d = p - q;
        return glm::sqrt(glm::pow(d.x,2) + glm::pow(d.y,2) + glm::pow(d.z,2));
    }

    double barycentric_distance(glm::vec3 p, glm::vec3 q, double t1t2, double t0t2, double t0t1) {
        auto PQ = p - q;
        auto dist2 = -(t1t2*t1t2*PQ.y*PQ.z) - (t0t2*t0t2*PQ.z*PQ.x) - (t0t1*t0t1*PQ.x*PQ.y);
        auto dist = glm::sqrt(dist2);
        return dist;
    }

    Vector compute_face_normal(const TessFacePtr& face, const SurfaceMesh& sm) {
        auto v0 = face->vertices.at(0);
        auto v1 = face->vertices.at(1);
        auto v2 = face->vertices.at(2);
        auto p0 = toGLM(sm.point(v0->vd));
        auto p1 = toGLM(sm.point(v1->vd));
        auto p2 = toGLM(sm.point(v2->vd));
        glm::vec3 nrm = glm::normalize(glm::cross(p1 - p0, p2 - p0));
        return {nrm.x, nrm.y, nrm.z};
    }

    Vector compute_face_normal(const Point_3& v0, const Point_3& v1, const Point_3& v2) {
        Vector p0 = v1 - v0;
        Vector p1 = v2 - v0;
        Vector fNrm = normalize(CGAL::cross_product(p0, p1));

        return fNrm;
    }

    Vector compute_vertex_normal(const SurfaceMesh& sm, const SM_vertex_descriptor& vd) {
        Vector vertexNorm = {0.0, 0.0, 0.0};
        for (const auto f : sm.faces_around_target(sm.halfedge(vd))) {
            std::array<Point_3, 3> vertices;
            int i = 0;
            for (auto v : sm.vertices_around_face(sm.halfedge(f))) {
                vertices.at(i++) = sm.point(v);
            }
            vertexNorm += compute_face_normal(vertices.at(0), vertices.at(1), vertices.at(2));
        }
        return normalize(vertexNorm);
    }

    Vector lerp(const Vector& a, const Vector& b, double t) {
        return a*t + b*(1.0-t);
    }

    Vector lerp(const Vector& A, const Vector& B, const Vector&C, double tA, double tB, double tC) {
        return A*tA + B*tB + C*tC;
    }

    Vector normalize(const Vector& V)
    {
        auto const slen = V.squared_length();
        auto const d = CGAL::approximate_sqrt(slen);
        return V / d;
    }

    glm::vec4 toGLM4(Kernel::Vector_3 p) {return {p.x(), p.y(), p.z(), 1.0f};}
    glm::vec3 toGLM(Kernel::Vector_3 p) {return {p.x(), p.y(), p.z()};}
    glm::vec3 toGLM(Kernel::Point_3 p) {return {p.x(), p.y(), p.z()};}
    glm::vec2 toGLM(Point_2 p) { return {p.x(), p.y()}; }

    Point_3 toPoint3(glm::vec3 p) { return {p.x, p.y, p.z}; }

    Gauss_vertex_pmap CalculateGaussianCurvature(SurfaceMesh& mesh) {
        Eigen::MatrixX3d V(mesh.num_vertices(), 3);
        Eigen::MatrixX3i F(mesh.num_faces(), 3);

        for (auto vd : mesh.vertices()) {
            Point_3 p = mesh.point(vd);
            V(vd.idx(), 0) = p.x();
            V(vd.idx(), 1) = p.y();
            V(vd.idx(), 2) = p.z();
        }

        for (auto fd: mesh.faces()) {
            unsigned int idx = 0;
            for (auto vd: mesh.vertices_around_face(mesh.halfedge(fd))) {
                F(fd.idx(), idx++) = vd.idx();
            }
        }

        Eigen::Matrix<double, Eigen::Dynamic, 1> K(mesh.num_vertices(), 1);
        igl::gaussian_curvature(V, F, K);

        auto gaussMap = mesh.add_property_map<SM_vertex_descriptor, double>("v:curvature").first;
        for (unsigned int i = 0; i < mesh.num_vertices(); i++) {
            put(gaussMap, SM_vertex_descriptor(i), K(i,0));
        }

        return gaussMap;
    }

    std::optional<std::pair<Point_3, Vector>> findIntersection(const Point_3& p, const Vector& nrm, const Tree& tree, SurfaceMesh& mesh) {
        Ray forwardRay(p, nrm);
        Ray reverseRay(p, -nrm);

        Ray_intersection forwardIntersection = tree.first_intersection(forwardRay);
        Ray_intersection reverseIntersection = tree.first_intersection(reverseRay);
        Point_3 forwardPoint, reversePoint;
        Primitive_id forwardPrimitive, reversePrimitive;
        bool foundForward = false, foundReverse = false;

        if (forwardIntersection && boost::get<Point_3>(&(forwardIntersection->first))) {
            forwardPoint = *boost::get<Point_3>(&(forwardIntersection->first));
            forwardPrimitive = boost::get<Primitive_id>(forwardIntersection->second);
            foundForward = true;
        }
        if (reverseIntersection && boost::get<Point_3>(&(reverseIntersection->first))) {
            reversePoint = *boost::get<Point_3>(&(reverseIntersection->first));
            reversePrimitive = boost::get<Primitive_id>(reverseIntersection->second);
            foundReverse = true;
        }

        Vector forwardNorm, reverseNorm;
        if (foundForward) {
            std::array<Point_3, 3> points;
            int i = 0;
            for (auto vd : mesh.vertices_around_face(mesh.halfedge(forwardPrimitive))) {
                points.at(i++) = mesh.point(vd);
            }
            forwardNorm = compute_face_normal(points.at(0), points.at(1), points.at(2));
        }
        if (foundReverse) {
            std::array<Point_3, 3> points;
            int i = 0;
            for (auto vd : mesh.vertices_around_face(mesh.halfedge(reversePrimitive))) {
                points.at(i++) = mesh.point(vd);
            }
            reverseNorm = compute_face_normal(points.at(0), points.at(1), points.at(2));
        }

        if (foundForward && foundReverse) {
            if (CGAL::squared_distance(p, forwardPoint) < CGAL::squared_distance(p, reversePoint)) {
                return std::pair<Point_3, Vector>(forwardPoint, forwardNorm);
            }
            else {
                return std::pair<Point_3, Vector>(reversePoint, reverseNorm);
            }
        } else if (foundForward) {
            return std::pair<Point_3, Vector>(forwardPoint, forwardNorm);
        } else if (foundReverse) {
            return std::pair<Point_3, Vector>(reversePoint, reverseNorm);
        }
        return std::nullopt;
    }

    std::string SectionString(AssigningSection section) {
        switch (section) {
            case MoveValidate:
                return "MoveValidate";
            case ProjectEdge:
                return "ProjectEdge";
            case MVEdgeCase:
                return "MVEdgeCase";
            case PVEdgeCase:
                return "PVEdgeCase";
            case ProjectValidate:
                return "ProjectValidate";
            case InterpolateUnmatched:
                return "InterpolateUnmatched";
            case Undone:
                return "Undone";
            case Never:
                return "Never";
            default:
                return "Error!";
        }
    };


    //Not used right now
    SeamMeshPtr UnwrapMesh(const SurfaceMeshPtr& sm, const std::string& selectionsFile) {
        std::vector<SM_vertex_descriptor> cone_sm_vds;
        SMP::read_cones(*sm, selectionsFile.c_str(), std::back_inserter(cone_sm_vds));
//    cone_sm_vds.emplace_back(4033);
//    cone_sm_vds.emplace_back(172);
//    cone_sm_vds.emplace_back(1176);
        // Two property maps to store the seam edges and vertices

        Seam_edge_pmap seam_edge_pm = sm->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
        Seam_vertex_pmap seam_vertex_pm = sm->add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;

        // The seam mesh
        auto mesh = std::make_shared<SeamMesh>(*sm, seam_edge_pm, seam_vertex_pm);

        std::cout << "Computing the shortest paths between consecutive cones" << "\n";
        std::list<SM_edge_descriptor> seam_edges;
        SMP::compute_shortest_paths_between_cones(*sm, cone_sm_vds.begin(), cone_sm_vds.end(), seam_edges);

        // Add the seams to the seam mesh
        for(SM_edge_descriptor e : seam_edges) {
            mesh->add_seam(source(e, *sm), target(e, *sm));
        }

        std::cout << mesh->number_of_seam_edges() << " seam edges in input" << "\n";

        // Index map of the seam mesh (assuming a single connected component so far)
        typedef std::unordered_map<vertex_descriptor, int> Indices;
        Indices indices;
        boost::associative_property_map<Indices> vimap(indices);
        int counter = 0;
        for(vertex_descriptor vd : vertices(*mesh)) {
            put(vimap, vd, counter++);
        }

        // Mark the cones in the seam mesh
        std::unordered_map<vertex_descriptor, SMP::Cone_type> cmap;
        SMP::locate_cones(*mesh, cone_sm_vds.begin(), cone_sm_vds.end(), cmap);

        // The 2D points of the uv parametrisation will be written into this map
        // Note that this is a halfedge property map, and that uv values
        // are only stored for the canonical halfedges representing a vertex
        UV_pmap uvmap = sm->add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

        // Parameterizer
        typedef SMP::Orbifold_Tutte_parameterizer_3<SeamMesh>         Parameterizer;
        Parameterizer parameterizer(SMP::Triangle, SMP::Mean_value);


        // a halfedge on the (possibly virtual) border
        // only used in output (will also be used to handle multiple connected components in the future)
        halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(*mesh).first;

        parameterizer.parameterize(*mesh, bhd, cmap, uvmap, vimap);
//    std::ofstream out("../Models/lucy-uv.off");
//    SMP::IO::output_uvmap_to_off(*mesh, bhd, uvmap, out);

        return mesh;
    }

    glm::vec3
    align_barycentric(const BarycentricPoint &point, const SM_vertex_descriptor &alignVd) {
        if (alignVd.idx() == point.v0) {
            return {point.alpha, point.beta, point.gamma};
        }
        else if (alignVd.idx() == point.v1) {
            return {point.beta, point.gamma, point.alpha};
        }
        else {
            return {point.gamma, point.alpha, point.beta};
        }
    }

    glm::vec3
    align_barycentric(const SMBarycentricPoint &point, const SM_vertex_descriptor &alignVd) {
        if (alignVd == point.v0) {
            return {point.alpha, point.beta, point.gamma};
        }
        else if (alignVd == point.v1) {
            return {point.beta, point.gamma, point.alpha};
        }
        else {
            return {point.gamma, point.alpha, point.beta};
        }
    }
}