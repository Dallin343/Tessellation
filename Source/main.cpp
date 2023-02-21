#include <iostream>
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <utility>
//#include "Model.h"
//#include "Camera.h"
//#include "MeshProcessor.h"
//#include "OMesh.h"
#include "GlobalParameterization.h"
#include "MeshImpl.h"

#include "Tessellator.h"

#include <Hungarian.h>
#include <igl/gaussian_curvature.h>

//#define DRAW_CGAL

const int WIDTH = 800;
const int HEIGHT = 600;

//float deltaTime = 0.0f;	// Time between current frame and last frame
//float lastFrame = 0.0f; // Time of last frame
//Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
//float lastX = WIDTH / 2.0f;
//float lastY = HEIGHT / 2.0f;
//bool firstMouse = true;


//void processInput(GLFWwindow *window);
//void mouse_callback(GLFWwindow* window, double xpos, double ypos);
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

typedef std::unordered_map<unsigned int, Point_2> VertexUVMap;
//Cartesian coords first, barycentric second
typedef std::unordered_map<SM_halfedge_descriptor, std::pair<glm::dvec3, glm::dvec3>> Vertex_bary_map;
typedef SurfaceMesh::Property_map<SM_face_descriptor, Vertex_bary_map> Bary_coord_map;

static void GLFWErrorCallback(int error, const char* description) {
    std::cout << "GLFW Error " << error << " " << description << std::endl;
}

std::optional<glm::dvec3> barycentric(glm::dvec3 point, glm::dvec3 t0, glm::dvec3 t1, glm::dvec3 t2) {
    glm::dvec3 u = t1 - t0;
    glm::dvec3 v = t2 - t0;
    glm::dvec3 n = glm::cross(u, v);
    glm::dvec3 w = point - t0;

    double gamma = glm::dot(glm::cross(u, w), n) / glm::dot(n, n);
    double beta = glm::dot(glm::cross(w, v), n) / glm::dot(n, n);
    double alpha = 1.0 - gamma - beta;
    bool a_check = alpha >= 0.0 && alpha <= 1.0;
    bool b_check = beta >= 0.0 && beta <= 1.0;
    bool g_check = gamma >= 0.0 && gamma <= 1.0;
    if (!a_check || !b_check || !g_check) {
        return std::nullopt;
    }
    return std::make_optional<glm::dvec3>(alpha, beta, gamma);
}
glm::dvec3 toGLM(Kernel::Vector_3 p) {return {p.x(), p.y(), p.z()};}
glm::dvec3 toGLM(Kernel::Point_3 p) {return {p.x(), p.y(), p.z()};}
glm::dvec2 toGLM(Point_2 p) { return {p.x(), p.y()}; }

Point_3 toPoint3(glm::dvec3 p) { return {p.x, p.y, p.z}; }

struct Stats {
    unsigned int removed_verts = 0;
    unsigned int overwritten_verts = 0;
    unsigned int unmatched_verts = 0;
    std::unordered_set<SM_halfedge_descriptor> removed_hds {};

    void print() const {
        std::cout << "\n== Collapse Stats ==\n";
        std::cout << "Removed Vertices: " << removed_verts << "\n";
        std::cout << "Unique Halfedges: " << removed_hds.size() << "\n";
        std::cout << "Overwritten Vertices: " << overwritten_verts << "\n";
        std::cout << "Unmatched Vertices: " << unmatched_verts << "\n";
        std::cout << "====\n\n";
    }
};

struct CollapseVisitor : SMS::Edge_collapse_visitor_base<SurfaceMesh> {
    CollapseVisitor(UV_pmap& uvmap, Stats& stats) : uvmap(uvmap), stats(stats) {}

    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(const EdgeProfile& prof,
                      boost::optional<Point> placement)
    {
        if (placement) {
            auto newPoint = placement.value();
            const auto& mesh = prof.surface_mesh();
//            auto seamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
//            Bary_coord_map baryMap = mesh.property_map<SM_face_descriptor, Vertex_bary_map>("f:bary").first;
//            v0 = prof.v0();
//            v1 = prof.v1();
//            v0_v1 = prof.v0_v1();
//            v1_v0 = prof.v1_v0();

//            auto hdL = prof.v1_vL();
//            auto hdR = prof.v0_vR();
//
//            auto faceL = mesh.face(hdL);
//            auto faceR = mesh.face(hdR);
//
//            auto vBaryMapL = get(baryMap, faceL);
//            auto vBaryMapR = get(baryMap, faceR);
//            this->vertBaryMap.clear();
//            this->vertBaryMap.merge(vBaryMapL);
//            this->vertBaryMap.merge(vBaryMapR);


            p0 = prof.p0();
            p1 = prof.p1();

            glm::dvec3 glmP0 = {p0.x(), p0.y(), p0.z()};
            glm::dvec3 glmP1 = {p1.x(), p1.y(), p1.z()};
            glm::dvec3 glmP = {newPoint.x(), newPoint.y(), newPoint.z()};

            double modelEdgeLen = glm::length(glmP1 - glmP0);
            double p_p0Len = glm::length(glmP - glmP0);
            double pp0Ratio = p_p0Len / modelEdgeLen;

            v0 = prof.v0();
            v1 = prof.v1();
            v0ds.clear();
            v1ds.clear();
            for (auto hd: halfedges_around_source(v0, mesh)) {
                v0ds.push_back(mesh.target(hd));
            }
            for (auto hd: halfedges_around_source(v1, mesh)) {
                v1ds.push_back(mesh.target(hd));
            }

            p0_2 = get(uvmap, prof.v0_v1());
            p1_2 = get(uvmap, prof.v1_v0());


            glm::dvec2 glmP0_2 = {p0_2.x(), p0_2.y()};
            glm::dvec2 glmP1_2 = {p1_2.x(), p1_2.y()};
//            auto uvEdgeLen = glm::length(glmP0_2 - glmP1_2);
            auto p0Vec = glmP1_2 - glmP0_2;
            glm::dvec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
            p_2 = {newPoint_2.x, newPoint_2.y};
        }
    }

    // Called after each edge has been collapsed
    void OnCollapsed(const EdgeProfile& prof, vertex_descriptor vd)
    {
        const auto& mesh = prof.surface_mesh();
        const auto& seamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;

        if (get(seamMap, vd)) {
//            std::cout.precision(std::numeric_limits<double>::max_digits10);
            if (vd == v1) {
                for (auto vertDesc : v0ds) {
                    if (vertDesc != vd) {
                        auto hd = mesh.halfedge(vd, vertDesc);
                        put(uvmap, hd, p_2);
                    }
                }
            } else if (vd == v0) {
                for (auto vertDesc : v1ds) {
                    if (vertDesc != vd) {
                        auto hd = mesh.halfedge(vd, vertDesc);
                        put(uvmap, hd, p_2);
                    }
                }
            }
        } else {
            for (auto hd: halfedges_around_source(vd, mesh)) {
                put(uvmap, hd, p_2);
            }
        }


//        Bary_coord_map baryMap = mesh.property_map<SM_face_descriptor, Vertex_bary_map>("f:bary").first;
//        SM_halfedge_descriptor collapsedHD;
//        if (vd == v0) {
//            collapsedHD = v1_v0;
//        } else if (vd == v1) {
//            collapsedHD = v0_v1;
//        }
//
//        std::vector<SM_face_descriptor> fds;
//        for (auto hd : halfedges_around_source(vd, mesh)) {
//            auto fd = mesh.face(hd);
//            fds.push_back(fd);
//            Vertex_bary_map faceBaryMap = get(baryMap, fd);
//            if (faceBaryMap.find(collapsedHD) != faceBaryMap.end()) {
//                stats.overwritten_verts += 1;
//            }
//            vertBaryMap.merge(faceBaryMap);
//        }
//
//        for (auto fd : fds) {
//            std::array<glm::dvec3, 3> ts{};
//            int idx = 0;
//            for (auto tVD : vertices_around_face(mesh.halfedge(fd), mesh)) {
//                ts.at(idx++) = toGLM(mesh.point(tVD));
//            }
//
//            Vertex_bary_map newBaryCoords;
//            for (auto it = vertBaryMap.cbegin(); it != vertBaryMap.cend();) {
//                auto vertDesc = it->first;
//                auto coords = it->second;
//                auto baryProj = barycentric(coords.first, ts.at(0), ts.at(1), ts.at(2));
//                if (baryProj.has_value()) {
//                    newBaryCoords.insert({vertDesc, std::make_pair(coords.first, baryProj.value())});
//                    vertBaryMap.erase(it++);
//                } else {
//                    ++it;
//                }
//            }
//
//            //Add just remvoed vertex to barycentric map if its in this face.
//            if (vd == v0 || vd == v1) {
//                auto baryProj = barycentric(toGLM(p0), ts.at(0), ts.at(1), ts.at(2));
//                if (baryProj.has_value()) {
//                    newBaryCoords.insert({collapsedHD, std::make_pair(toGLM(p0), baryProj.value())});
//                    stats.removed_hds.insert(collapsedHD);
//                }
//                v0 = SM_vertex_descriptor(-1);
//                v1 = SM_vertex_descriptor(-1);
//            }
//            put(baryMap, fd, newBaryCoords);
//        }
//        stats.removed_verts += 1;
//        stats.unmatched_verts += vertBaryMap.size();
//        if (vd.idx() == 26622 || vd.idx() == 26624 || vd.idx() == 37978) {
//            std::ofstream meshStream("../Models/dragon-processing.obj");
//            CGAL::IO::write_OBJ(meshStream, mesh);
//
//            std::ofstream out("../Models/dragon-processing-uv.off");
//            halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
//            SMP::IO::output_uvmap_to_off(mesh, bhd, &uvmap, out);
//        }
    }

    UV_pmap uvmap;
    Stats& stats;
    Vertex_bary_map vertBaryMap;
    SM_vertex_descriptor v0, v1, vR, vL;
    SM_halfedge_descriptor v0_v1, v1_v0;
    Point_3 p0, p1, pR, pL;
    Point_2 p0_2, p1_2, p_2;
    std::vector<SM_vertex_descriptor> v0ds, v1ds;
};

template <typename GHPolicies>
void collapse_gh(const std::shared_ptr<SeamMesh>& seamMesh,
                 const std::shared_ptr<SurfaceMesh>& mesh,
                 const double ratio)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    SMS::Count_ratio_stop_predicate<SeamMesh> stop(ratio);
    Seam_is_constrained_edge_map seam_edge_map(*seamMesh);

    // Garland&Heckbert simplification policies
    typedef typename GHPolicies::Get_cost                                        GH_cost;
    typedef typename GHPolicies::Get_placement                                   GH_placement;
    typedef SMS::Constrained_placement<GH_placement, Seam_is_constrained_edge_map > Constrained_GH_placement;

    GHPolicies gh_policies(*mesh);
    const GH_cost& gh_cost = gh_policies.get_cost();
    const GH_placement& gh_placement = gh_policies.get_placement();
    Constrained_GH_placement placement(seam_edge_map, gh_placement);
    SMS::Bounded_normal_change_filter<> filter;

    auto uvmap = mesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
    Stats stats;
    CollapseVisitor vis(uvmap, stats);

    int r = SMS::edge_collapse(*mesh, stop,
                               CGAL::parameters::get_cost(gh_cost)
                               .edge_is_constrained_map(seam_edge_map)
                               .visitor(vis)
                               .filter(filter)
                               .get_placement(placement));

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::cout << "Time elapsed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
              << "ms" << std::endl;
    std::cout << "\nFinished!\n" << r << " edges removed.\n" << edges(*mesh).size() << " final edges.\n";

    stats.print();
}

std::shared_ptr<SurfaceMesh> LoadMesh(const std::string& filename) {
    auto mesh = std::make_shared<SurfaceMesh>();
    if(!CGAL::IO::read_OBJ(filename, *mesh))
    {
        std::cerr << "Invalid input file." << std::endl;
    }

    return mesh;
}

void ReadUV(const std::shared_ptr<SurfaceMesh>& mesh, const std::string& filename) {
    std::ifstream infile(filename);

    std::string line;
    std::vector<Point_2> texCoords;
    texCoords.reserve(mesh->num_vertices());

    UV_pmap uvmap = mesh->add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

    // Get texcoords from file.
    // There MAY be more texcoords than vertices
    while (std::getline(infile, line)) {
        if (line.substr(0, 2) == "vt") {
            std::istringstream iss(line.substr(3, line.length()-3));
            double u, v;
            iss >> u >> v;
            texCoords.emplace_back(u, v);

        } else if (line.substr(0, 1) == "f") {
            line = line.substr(2, line.length() - 2);
            std::string s0, s1, s2;
            unsigned int w0, w1;
            //Split 1/1/1 2/2/2 3/3/3 into separate strings like "1/1/1"
            w0 = line.find(' ');
            w1 = line.find(' ', w0+1);
            s0 = line.substr(0, w0);
            s1 = line.substr(w0+1, w1 - w0-1);
            s2 = line.substr(w1+1, line.length() - w1-1);

            SM_vertex_descriptor v0, v1, v2;
            Point_2 vt0, vt1, vt2;
            unsigned int v, vt;
            w0 = s0.find('/');
            w1 = s0.find('/', w0+1);
            v = std::stoul(s0.substr(0, w0)) - 1;
            v0 = SM_vertex_descriptor(v);
            vt = std::stoul(s0.substr(w0+1, w1 - w0-1)) - 1;
            vt0 = texCoords.at(vt);

            w0 = s1.find('/');
            w1 = s1.find('/', w0+1);
            v = std::stoul(s1.substr(0, w0)) - 1;
            v1 = SM_vertex_descriptor(v);
            vt = std::stoul(s1.substr(w0+1, w1 - w0-1)) - 1;
            vt1 = texCoords.at(vt);

            w0 = s2.find('/');
            w1 = s2.find('/', w0+1);
            v = std::stoul(s2.substr(0, w0)) - 1;
            v2 = SM_vertex_descriptor(v);
            vt = std::stoul(s2.substr(w0+1, w1 - w0-1)) - 1;
            vt2 = texCoords.at(vt);

            put(uvmap, mesh->halfedge(v0, v1), vt0);
//            put(uvmap, mesh->halfedge(v0, v2), vt0);
//            put(uvmap, mesh->halfedge(v1, v0), vt1);
            put(uvmap, mesh->halfedge(v1, v2), vt1);
            put(uvmap, mesh->halfedge(v2, v0), vt2);
//            put(uvmap, mesh->halfedge(v2, v1), vt2);
        }
    }

    auto highResUVMap = mesh->add_property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
    std::copy(uvmap.begin(), uvmap.end(), highResUVMap.begin());
}

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

std::shared_ptr<SeamMesh> AddSeams(const std::shared_ptr<SurfaceMesh>& sm, const std::string& selectionsFile) {
    Seam_edge_pmap seam_edge_pm = sm->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
    Seam_vertex_pmap seam_vertex_pm = sm->add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;
    // The seam mesh
    auto mesh = std::make_shared<SeamMesh>(*sm, seam_edge_pm, seam_vertex_pm);

    // Add the seams to the seam mesh
    SM_halfedge_descriptor bhd = mesh->add_seams(selectionsFile.c_str());

    if (bhd == SurfaceMesh::null_halfedge()) {
        std::cerr << "There was an error while adding seams." << std::endl;
    }

    return mesh;
}

std::shared_ptr<SeamMesh> UnwrapMesh(const std::shared_ptr<SurfaceMesh>& sm, const std::string& selectionsFile) {
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

    std::cout << "Computing the shortest paths between consecutive cones" << std::endl;
    std::list<SM_edge_descriptor> seam_edges;
    SMP::compute_shortest_paths_between_cones(*sm, cone_sm_vds.begin(), cone_sm_vds.end(), seam_edges);

    // Add the seams to the seam mesh
    for(SM_edge_descriptor e : seam_edges) {
        mesh->add_seam(source(e, *sm), target(e, *sm));
    }

    std::cout << mesh->number_of_seam_edges() << " seam edges in input" << std::endl;

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


struct BaryData {
    glm::dvec3 p0, p1, p2, u, v, n;
    double a, b, c;
};
BaryData get_bary_vars(glm::dvec3 p0, glm::dvec3 p1, glm::dvec3 p2) {
    BaryData data{};
    data.p0 = p0;
    data.p1 = p1;
    data.p2 = p2;
    data.u = p1 - p0;
    data.v = p2 - p0;
    data.n = glm::cross(data.u, data.v);
    data.a = glm::length(p1 - p0);
    data.b = glm::length(p2 - p0);
    data.c = glm::length(p2 - p1);
    return data;
}
std::optional<glm::dvec3> project(const BaryData& d, glm::dvec3 w) {
    double gamma = glm::dot(glm::cross(d.u, w), d.n) / glm::dot(d.n, d.n);
    double beta = glm::dot(glm::cross(w, d.v), d.n) / glm::dot(d.n, d.n);
    double alpha = 1.0 - gamma - beta;
    bool a_check = alpha >= 0.0 && alpha <= 1.0;
    bool b_check = beta >= 0.0 && beta <= 1.0;
    bool g_check = gamma >= 0.0 && gamma <= 1.0;
    if (!a_check || !b_check || !g_check) {
        return std::nullopt;
    }
    return std::make_optional<glm::dvec3>(alpha, beta, gamma);
}

glm::dvec3 barycentric(glm::dvec2 p, glm::dvec2 a, glm::dvec2 b, glm::dvec2 c) {
    glm::dvec2 v0 = b - a;
    glm::dvec2 v1 = c - a;
    glm::dvec2 v2 = p - a;
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

double barycentric_distance(BaryData d, glm::dvec3 p, glm::dvec3 q) {
    auto PQ = p - q;
    auto dist2 = -(d.a*d.a*PQ.y*PQ.z) - (d.b*d.b*PQ.z*PQ.x) - (d.c*d.c*PQ.x*PQ.y);
    auto dist = glm::sqrt(dist2);
    return dist;
}

double barycentric_distance(glm::dvec3 p, glm::dvec3 q, double t1t2, double t0t2, double t0t1) {
    auto PQ = p - q;
    auto dist2 = -(t1t2*t1t2*PQ.y*PQ.z) - (t0t2*t0t2*PQ.z*PQ.x) - (t0t1*t0t1*PQ.x*PQ.y);
    auto dist = glm::sqrt(dist2);
    return dist;
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

struct TessellationData {
    unsigned int ol0{1}, ol1{1}, ol2{1}, il{2};
    std::vector<glm::dvec3> tessBaryCoords, featureBaryCoords, feature3DCoords;
    std::vector<int> assignment;
    std::shared_ptr<TessellatedTriangle> tessTri;
};


class ProcessFace {
public:
    ProcessFace() {}

    ProcessFace(const SM_face_descriptor &fd, const array<SM_vertex_descriptor, 3> &vds, const array<Point_2, 3> &uvs,
                const array<Point_3, 3> &coords) : fd(fd), vds(vds), uvs(uvs), coords(coords) {}

    SM_face_descriptor fd {};
    std::array<SM_vertex_descriptor, 3> vds {};
    std::array<Point_2, 3> uvs {};
    std::array<Point_3, 3> coords {};
};
typedef std::shared_ptr<ProcessFace> ProcessFacePtr;

class FeatureVert {
public:
    FeatureVert(const SM_halfedge_descriptor &hd, const glm::dvec3 &cartCoords, const glm::dvec3 &baryCoords,
                const glm::dvec2 &uv) : hd(hd), cartCoords(cartCoords), baryCoords(baryCoords), uv(uv) {}

    SM_halfedge_descriptor hd;
    glm::dvec3 cartCoords;
    glm::dvec3 baryCoords;
    glm::dvec2 uv;
};
typedef std::shared_ptr<FeatureVert> FeatureVertPtr;

class TessellatedVert {
public:
    TessellatedVert() {
        undoMove();
    }

    TessellatedVert(ProcessFacePtr face, FeatureVertPtr matchingFeature,
                    const glm::dvec3 &origCoords, const glm::dvec3 &baryCoords) :
                    face(std::move(face)), matchingFeature(std::move(matchingFeature)), origCoords(origCoords),
                    baryCoords(baryCoords) {
        undoMove();
    }

    TessellatedVert(ProcessFacePtr face, const glm::dvec3 &origCoords, const glm::dvec3 &baryCoords) :
                    face(std::move(face)), origCoords(origCoords), baryCoords(baryCoords) {
        undoMove();
    }

    void undoMove() {
        newCoords = {minDbl, minDbl, minDbl};
    }

    bool isAssigned() {
        return newCoords != glm::dvec3(minDbl, minDbl, minDbl);
    }

    ProcessFacePtr face;
    FeatureVertPtr matchingFeature = nullptr;
    glm::dvec3 origCoords;
    glm::dvec3 newCoords{};
    glm::dvec3 baryCoords;
    glm::dvec2 uv{};
    bool isInner = false;
    bool anchored = false;
    SM_vertex_descriptor vd;

private:
    double minDbl = std::numeric_limits<double>::min();
};
typedef std::shared_ptr<TessellatedVert> TessVertPtr;
typedef std::vector<FeatureVertPtr> FeatureVerts;
typedef std::vector<TessVertPtr> TessellatedVerts;
typedef std::unordered_map<SM_vertex_descriptor, TessVertPtr> VDToTessVert;
typedef std::shared_ptr<TessellatedTriangle> TessTriPtr;

FeatureVerts extractFeatureVertices(Point2Set& pSet, const ProcessFacePtr& tri, const SurfaceMesh& highResMesh) {
    std::list<Vertex_handle> vertexList;
    pSet.range_search(tri->uvs.at(0), tri->uvs.at(1), tri->uvs.at(2), std::back_inserter(vertexList));

    FeatureVerts foundVerts;
    foundVerts.reserve(vertexList.size());

    auto t0 = toGLM(tri->uvs.at(0)); // (1, 0, 0)
    auto t1 = toGLM(tri->uvs.at(1)); // (0, 1, 0)
    auto t2 = toGLM(tri->uvs.at(2)); // (0, 0, 1)

    for (const auto vHandle : vertexList) {
        auto hd = SM_halfedge_descriptor(vHandle->info());
        Point_3 point = highResMesh.point(highResMesh.source(hd));
        glm::dvec3 vertex = {point.x(), point.y(), point.z()};
        glm::dvec2 uv ={vHandle->point().x(), vHandle->point().y()};
        glm::dvec3 baryCoords = barycentric(uv, t0, t1, t2);

        FeatureVertPtr feature = std::make_shared<FeatureVert>(hd, vertex, baryCoords, uv);
        foundVerts.emplace_back(feature);
    }

    return foundVerts;
}

TessellatedVerts minBiGraphMatch(const ProcessFacePtr& triRange, const FeatureVerts& featureVerts, TessTriPtr& tessTri) {
    int ol0 = 5, ol1 = 5, ol2 = 5, il = 5;

    auto t0 = toGLM(triRange->uvs.at(0)); // (1, 0, 0)
    auto t1 = toGLM(triRange->uvs.at(1)); // (0, 1, 0)
    auto t2 = toGLM(triRange->uvs.at(2)); // (0, 0, 1)

    // find barycentric coords for tessellated vertices
    Triangle tri = {glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0)};
    tessTri = Tessellator::TessellateTriangle(tri, ol0, ol1, ol2, il);
    std::vector<glm::dvec3> innerVertBaryCoords;

    TessellatedVerts tessellatedVerts;
    tessellatedVerts.reserve(tessTri->vertices.size());

    innerVertBaryCoords.reserve(tessTri->innerVertexIndices.size());

    //TODO: Optimize this potentially
    //Add tessellated vertices to map;
    for (auto bary : tessTri->vertices) {
        auto c = triRange->coords;
        auto origCoords = bary.x * toGLM(c.at(0)) + bary.y * toGLM(c.at(1)) + bary.z * toGLM(c.at(2));
        TessVertPtr vert = std::make_shared<TessellatedVert>(triRange, origCoords, bary);
        tessellatedVerts.push_back(vert);
    }

    // Mark inner vertices in map
    for (auto idx : tessTri->innerVertexIndices) {
        auto vertex = tessTri->vertices.at(idx);
        innerVertBaryCoords.push_back(vertex);
        tessellatedVerts.at(idx)->isInner = true;
    }

    //Create Distance matrix for hungarian algo
    std::vector<std::vector<double>> distMatrix(innerVertBaryCoords.size());
    for (auto& col : distMatrix) {
        col.reserve(featureVerts.size());
    }

    //Generate edges between sides of the graph
    for (int i = 0; i < innerVertBaryCoords.size(); i++) {
        for (const auto& fVert : featureVerts) {
            auto tVert = innerVertBaryCoords.at(i);
            double t0t1 = glm::length(t1 - t0);
            double t0t2 = glm::length(t2 - t0);
            double t1t2 = glm::length(t2 - t1);
            double dist = barycentric_distance(tVert, fVert->baryCoords, t1t2, t0t2, t0t1);
            distMatrix.at(i).push_back(dist);
        }
    }

    HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    hungarian.Solve(distMatrix, assignment);

    for (int i = 0; i < assignment.size(); i++) {
        auto assign = assignment.at(i);

        auto bary = innerVertBaryCoords.at(i);
        auto c = triRange->coords;
        auto origCoords = bary.x * toGLM(c.at(0)) + bary.y * toGLM(c.at(1)) + bary.z * toGLM(c.at(2));

        if (assign != -1) {
            tessellatedVerts.at(assign)->matchingFeature = featureVerts.at(assign);
        }
    }
    return tessellatedVerts;
}

std::optional<Point_3> findIntersection(const Point_3& p, const Vector& nrm, const Tree& tree) {
    Ray forwardRay(p, nrm);
    Ray reverseRay(p, -nrm);

    Ray_intersection forwardIntersection = tree.first_intersection(forwardRay);
    Ray_intersection reverseIntersection = tree.first_intersection(reverseRay);
    Point_3 forwardPoint, reversePoint;
    bool foundForward = false, foundReverse = false;

    if (forwardIntersection && boost::get<Point_3>(&(forwardIntersection->first))) {
        forwardPoint = *boost::get<Point_3>(&(forwardIntersection->first));
        foundForward = true;
    }
    if (reverseIntersection && boost::get<Point_3>(&(reverseIntersection->first))) {
        reversePoint = *boost::get<Point_3>(&(reverseIntersection->first));
        foundReverse = true;
    }

    if (foundForward && foundReverse) {
        return CGAL::squared_distance(p, forwardPoint) < CGAL::squared_distance(p, reversePoint)
               ? forwardPoint : reversePoint;
    } else if (foundForward) {
        return forwardPoint;
    } else if (foundReverse) {
        return reversePoint;
    }
    return std::nullopt;
}

SurfaceMesh moveAndValidate(TessellatedVerts& tessVerts, const TessTriPtr& tessTri, const SurfaceMesh& sm, Tree& aabbTree,
                            const Gauss_vertex_pmap& gaussMap) {
    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

    SurfaceMesh tessTriMesh;
    auto triVNorms = tessTriMesh.add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
    auto triFNorms = tessTriMesh.add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;

    ProcessFacePtr face = tessVerts.at(0)->face;
    glm::dvec3 t0 = toGLM(face->coords.at(0));
    glm::dvec3 t1 = toGLM(face->coords.at(1));
    glm::dvec3 t2 = toGLM(face->coords.at(2));

    std::vector<SM_vertex_descriptor> tessVertIdx;
    tessVertIdx.reserve(tessTri->vertices.size());

    // Construct temp tessellated triangle surfaceMesh
    for (auto& tessVert : tessVerts) {
        auto bary = tessVert->baryCoords;
        glm::dvec3 coord = bary.x * t0 + bary.y * t1 + bary.z * t2;
        auto vd = tessTriMesh.add_vertex({coord.x, coord.y, coord.z});
        if (bary.x == 1.0) {
            //At p0
            put(triVNorms, vd, get(vNorms, face->vds.at(0)));
        } else if (bary.y == 1.0) {
            //At p1
            put(triVNorms, vd, get(vNorms, face->vds.at(1)));
        } else if (bary.z == 1.0) {
            //At p2
            put(triVNorms, vd, get(vNorms, face->vds.at(2)));
        } else if (bary.x == 0.0) {
            // On p1 - p2 edge
            auto a = get(vNorms, face->vds.at(1));
            auto b = get(vNorms, face->vds.at(2));
            auto nrm = normalize(lerp(a, b, bary.y));
            put(triVNorms, vd, nrm);
        } else if (bary.y == 0.0) {
            // On p0 - p2 edge
            auto a = get(vNorms, face->vds.at(0));
            auto b = get(vNorms, face->vds.at(2));
            auto nrm = normalize(lerp(a, b, bary.x));
            put(triVNorms, vd, nrm);
        } else if (bary.z == 0.0) {
            // On p0 - p1 edge
            auto a = get(vNorms, face->vds.at(0));
            auto b = get(vNorms, face->vds.at(1));
            auto nrm = normalize(lerp(a, b, bary.x));
            put(triVNorms, vd, nrm);
        } else {
            // Interior
            auto a = get(vNorms, face->vds.at(0));
            auto b = get(vNorms, face->vds.at(1));
            auto c = get(vNorms, face->vds.at(2));
            auto nrm = normalize(lerp(a, b, c, bary.x, bary.y, bary.z));
            put(triVNorms, vd, nrm);
        }
        tessVert->vd = vd;
    }

    for (auto fIdx : tessTri->faces) {
        auto vd0 = tessVerts.at(fIdx.at(0))->vd;
        auto vd1 = tessVerts.at(fIdx.at(1))->vd;
        auto vd2 = tessVerts.at(fIdx.at(2))->vd;
        auto fd = tessTriMesh.add_face(vd0, vd1, vd2);
        put(triFNorms, fd, get(fNorms, face->fd));
    }
#ifdef DRAW_CGAL
    CGAL::draw(tessTriMesh);
#endif

    //Project edges along normal to intersection
    for (const auto& vertex : tessVerts) {
        if (!vertex->isInner) {
            Point_3 vPoint = tessTriMesh.point(vertex->vd);
            Vector norm = get(triVNorms, vertex->vd);
            auto projOnNormal = findIntersection(vPoint, norm, aabbTree);
            if (projOnNormal.has_value()) {
                vertex->newCoords = toGLM(projOnNormal.value());
                vertex->anchored = true;
                tessTriMesh.point(vertex->vd) = projOnNormal.value();
            }
        }
    }

    //Move
    std::unordered_map<SM_vertex_descriptor, TessVertPtr> vdToTessVert;
    for (auto& tessVert : tessVerts) {
        vdToTessVert.insert({tessVert->vd, tessVert});
        if (!tessVert->isInner) {
            continue;
        }

        if (tessVert->matchingFeature != nullptr) {
            glm::dvec3 newCoord = tessVert->matchingFeature->cartCoords;
            tessVert->newCoords = newCoord;
            tessTriMesh.point(tessVert->vd) = {newCoord.x, newCoord.y, newCoord.z};
        }
    }

    //Validate
    for (auto tessFace : tessTriMesh.faces()) {
        Vector norm = get(triFNorms, tessFace);
        double dot;
        do {
            Vector newNorm = CGAL::Polygon_mesh_processing::compute_face_normal(tessFace, tessTriMesh);
            dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

            if (dot < 0) {
                SM_vertex_descriptor minVD;
                double minCurvature = std::numeric_limits<double>::max();
                for (auto vd : tessTriMesh.vertices_around_face(tessTriMesh.halfedge(tessFace))) {
                    auto tessVert = vdToTessVert.at(vd);
                    if (!tessVert->isInner) {
                        //This is an edge vertex, skip it
                        continue;
                    }

                    auto feature = tessVert->matchingFeature;
                    if (feature != nullptr) {
                        auto featureVD = sm.source(feature->hd);
                        if (get(gaussMap, featureVD) < minCurvature) {
                            minCurvature = get(gaussMap, featureVD);
                            minVD = vd;
                        }
                    }
                }

                auto vert = vdToTessVert.at(minVD);
                tessTriMesh.point(minVD) = toPoint3(vert->origCoords);
                vert->matchingFeature = nullptr;
                vert->undoMove();
            }
        } while(dot < 0);
    }
#ifdef DRAW_CGAL
    CGAL::draw(tessTriMesh);
#endif
    for (const auto& tessVert : tessVerts) {
        if (tessVert->isInner && tessVert->matchingFeature != nullptr) {
            tessVert->anchored = true;
        } else if (!tessVert->isInner && tessVert->isAssigned()) {
            tessVert->anchored = true;
        }
    }
    return tessTriMesh;
}

TessellatedVerts projectAndValidate(TessellatedVerts& tessVerts, const SurfaceMesh& sm, Tree& aabbTree,
                                    SurfaceMesh& tessTriMesh) {

    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

    auto triVNorms = tessTriMesh.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto triFNorms = tessTriMesh.property_map<SM_face_descriptor, Vector>("f:normal").first;

    std::unordered_map<SM_vertex_descriptor, TessVertPtr> vdToTessVert;
    //Project unmatched inner verts along normal to intersection
    for (auto& tessVert : tessVerts) {
        vdToTessVert.insert({tessVert->vd, tessVert});
        if (tessVert->isInner && !tessVert->isAssigned()) {
            Point_3 vPoint = tessTriMesh.point(tessVert->vd);
            Vector norm = get(triVNorms, tessVert->vd);
            auto projOnNormal = findIntersection(vPoint, norm, aabbTree);
            if (projOnNormal.has_value()) {
                auto newCoords = toGLM(projOnNormal.value());
                tessVert->newCoords = newCoords;
                tessTriMesh.point(tessVert->vd) = projOnNormal.value();
            }
        }
    }

    TessellatedVerts interpolateVerts;
    //Validate
    for (auto tessFace : tessTriMesh.faces()) {
        Vector norm = get(triFNorms, tessFace);
        double dot;
        do {
            Vector newNorm = CGAL::Polygon_mesh_processing::compute_face_normal(tessFace, tessTriMesh);
            dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

            if (dot < 0) {
                SM_vertex_descriptor undoVD;
                for (auto vd : tessTriMesh.vertices_around_face(tessTriMesh.halfedge(tessFace))) {
                    auto tessVert = vdToTessVert.at(vd);
                    if (!tessVert->isInner || tessVert->anchored) {
                        //This is an edge or anchored vertex, skip it
                        continue;
                    }

                    undoVD = vd;
                }

                auto vert = vdToTessVert.at(undoVD);
                tessTriMesh.point(undoVD) = toPoint3(vert->origCoords);
                vert->matchingFeature = nullptr;
                vert->undoMove();
            }
        } while(dot < 0);
    }

    //Add unmatched edges to interpolation list
    for (const auto& tessVert : tessVerts) {
        if (!tessVert->isInner && !tessVert->isAssigned()) {
            interpolateVerts.push_back(tessVert);
        }
    }

    return interpolateVerts;
}

int main(int argc, char** argv)
{

    const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path("../Models/beast.obj");
    const std::string simplifiedFilename = CGAL::data_file_path("../Models/beast-simplified.obj");
    const std::string selectionsFilename = CGAL::data_file_path("../Models/beast.selection.txt");

    std::cout << "Loading model from file..." << std::endl;
    std::shared_ptr<SurfaceMesh> sm = LoadMesh(filename);
    std::shared_ptr<SurfaceMesh> highResMesh = LoadMesh(filename);
    auto gaussMap = CalculateGaussianCurvature(*highResMesh);
//    auto baryMap = sm->add_property_map<SM_face_descriptor, Vertex_bary_map>("f:bary").first;


    std::cout << "Adding seams to model from selections file..." << std::endl;
    std::shared_ptr<SeamMesh> mesh = AddSeams(sm, selectionsFilename);

    std::cout << "Reading UV coords from file..." << std::endl;
    ReadUV(sm, filename);
    ReadUV(highResMesh, filename);

//    std::cout << "Copying high res model..." << std::endl;
//    auto highResMesh = std::make_shared<SurfaceMesh>(*sm);
//    std::shared_ptr<SeamMesh> highReshSeamMesh = AddSeams(highResMesh, selectionsFilename);

//    std::cout << "Unwrapping model with Orbifold-Tutte..." << std::endl;
//    std::shared_ptr<SeamMesh> mesh = UnwrapMesh(sm, selectionsFilename);

    auto highResUVMap = sm->property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
    auto uvmap = sm->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

    std::vector<std::pair<Point_2, SM_halfedge_descriptor>> points;
    points.reserve(sm->num_halfedges());

    for (auto hd : sm->halfedges()) {
        points.emplace_back(get(uvmap, hd), hd);
    }

    std::cout << "Simplifying model with Garland-Heckbert..." << std::endl;
    collapse_gh<Classic_plane>(mesh, sm, 0.1);

    std::cout << "Calculating normals..." << std::endl;
    auto fNorms = sm->add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;
    auto vNorms = sm->add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
    CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);

    std::cout << "Creating AABB Tree..." << std::endl;
    Tree aabbTree(highResMesh->faces().begin(), highResMesh->faces().end(), *highResMesh);

    std::cout << "Finding feature vertices in simplified triangles..." << std::endl;
    Point2Set pointSet;
    pointSet.insert(points.begin(), points.end());

    auto edgeProcessed = sm->add_property_map<SM_edge_descriptor, bool>("e:processed", false).first;

    SurfaceMesh sm_copy = SurfaceMesh(*sm);
    unsigned int count = 0;
    for (SM_face_descriptor fd : sm->faces()) {
        auto triRange = std::make_shared<ProcessFace>();
        triRange->fd = fd;

        unsigned int arrIdx = 0;
        for (SM_halfedge_descriptor hd: sm->halfedges_around_face(sm->halfedge(fd))) {
            triRange->uvs.at(arrIdx) = get(uvmap, hd);
            triRange->coords.at(arrIdx) = sm->point(sm->source(hd));
            triRange->vds.at(arrIdx) = sm->source(hd);
            arrIdx++;
        }

        auto featureVerts = extractFeatureVertices(pointSet, triRange, *highResMesh);

        TessTriPtr tessellatedTriangle;
        auto tessVerts = minBiGraphMatch(triRange, featureVerts, tessellatedTriangle);
        auto tessTriMesh = moveAndValidate(tessVerts, tessellatedTriangle, *sm, aabbTree, gaussMap);
        auto interpolateVerts = projectAndValidate(tessVerts, *sm, aabbTree, tessTriMesh);
        CGAL::draw(tessTriMesh);
//
//        auto p0 = triPoints.at(0).second;
//        auto p1 = triPoints.at(1).second;
//        auto p2 = triPoints.at(2).second;
//        BaryData bary = get_bary_vars(p0, p1, p2);
//
//        std::vector<glm::dvec3> featurePoints;
//        std::vector<glm::dvec3> featureBaryCoords;
//        std::vector<SM_vertex_descriptor> featureVDs;
//        // project vertices into barycentric coords, only keeping ones that are actually inside.
//        for (auto kv : featureVerts) {
//            auto vertex = kv.second;
//            auto proj = project(bary, vertex - p0);
//
//            if (proj.has_value()) {
//                featureBaryCoords.emplace_back(proj.value());
//                featurePoints.emplace_back(vertex);
//                featureVDs.push_back(highResMesh->source(kv.first));
//            }
//        }
//
//        Triangle tri = {glm::dvec3(0.0, 0.0, 1.0), glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0)};
//        auto tessTri = Tessellator::TessellateTriangle(tri, 2, 2, 2, 3);
//        std::vector<SM_vertex_descriptor> tessVertIdx;
//        SurfaceMesh tessTriMesh;
//        auto triVNorms = tessTriMesh.add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
//        auto triFNorms = tessTriMesh.add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;
//
//        // Construct temp tessellated triangle surfaceMesh
//        for (auto tessVert : tessTri->vertices) {
//            glm::dvec3 coord = tessVert.x * p0 + tessVert.y * p1 + tessVert.z * p2;
//            auto vd = tessTriMesh.add_vertex({coord.x, coord.y, coord.z});
//            if (tessVert.x == 1.0) {
//                //At p0
//                put(triVNorms, vd, get(vNorms, sm->source(triPoints.at(0).first)));
//            } else if (tessVert.y == 1.0) {
//                //At p1
//                put(triVNorms, vd, get(vNorms, sm->source(triPoints.at(1).first)));
//            } else if (tessVert.z == 1.0) {
//                //At p2
//                put(triVNorms, vd, get(vNorms, sm->source(triPoints.at(2).first)));
//            } else if (tessVert.x == 0.0) {
//                // On p1 - p2 edge
//                auto a = get(vNorms, sm->source(triPoints.at(1).first));
//                auto b = get(vNorms, sm->source(triPoints.at(2).first));
//                auto nrm = normalize(lerp(a, b, tessVert.y));
//                put(triVNorms, vd, nrm);
//            } else if (tessVert.y == 0.0) {
//                // On p0 - p2 edge
//                auto a = get(vNorms, sm->source(triPoints.at(0).first));
//                auto b = get(vNorms, sm->source(triPoints.at(2).first));
//                auto nrm = normalize(lerp(a, b, tessVert.x));
//                put(triVNorms, vd, nrm);
//            } else if (tessVert.z == 0.0) {
//                // On p0 - p1 edge
//                auto a = get(vNorms, sm->source(triPoints.at(0).first));
//                auto b = get(vNorms, sm->source(triPoints.at(1).first));
//                auto nrm = normalize(lerp(a, b, tessVert.x));
//                put(triVNorms, vd, nrm);
//            } else {
//                // Interior
//                auto a = get(vNorms, sm->source(triPoints.at(0).first));
//                auto b = get(vNorms, sm->source(triPoints.at(1).first));
//                auto c = get(vNorms, sm->source(triPoints.at(2).first));
//                auto nrm = normalize(lerp(a, b, c, tessVert.x, tessVert.y, tessVert.z));
//                put(triVNorms, vd, nrm);
//            }
//            tessVertIdx.push_back(vd);
//        }
//
//        for (auto fIdx : tessTri->faces) {
//            auto vd0 = tessVertIdx.at(fIdx.at(0));
//            auto vd1 = tessVertIdx.at(fIdx.at(1));
//            auto vd2 = tessVertIdx.at(fIdx.at(2));
//            auto fd0 = tessTriMesh.add_face(vd0, vd1, vd2);
//            put(triFNorms, fd0, get(fNorms, fd));
//        }
//
////        CGAL::draw(tessTriMesh);
//
//        std::vector<SM_vertex_descriptor> tessBaryIdxToVD;
//        std::vector<glm::dvec3> tessBaryCoords;
//        tessBaryCoords.reserve(tessTri->innerVertexIndices.size());
//
//        std::unordered_map<SM_vertex_descriptor, Point_3> newCoords;
//        std::vector<SM_vertex_descriptor> interpolateVerts;
//        // Only look at inner vertices, not edge vertices
//        for (auto idx : tessTri->innerVertexIndices) {
//            auto vertex = tessTri->vertices.at(idx);
//            tessBaryCoords.push_back(vertex);
//            tessBaryIdxToVD.push_back(tessVertIdx.at(idx));
//        }
//
//        std::unordered_map<SM_vertex_descriptor, Point_3> origPoints;
//        //Project edges along normal to intersection
//        for (auto vertex : tessTriMesh.vertices()) {
//            if (tessTriMesh.is_border(vertex)) {
//                Point_3 vPoint = tessTriMesh.point(vertex);
//                origPoints.insert({vertex, vPoint});
//                Vector norm = get(triVNorms, vertex);
//                auto projOnNormal = findIntersection(vPoint, norm, aabbTree);
//                if (projOnNormal.has_value()) {
//                    newCoords.insert({vertex, projOnNormal.value()});
//                    tessTriMesh.point(vertex) = projOnNormal.value();
//                } else {
//                    interpolateVerts.push_back(vertex);
//                }
//            }
//        }
//
//        //Create Distance matrix for hungarian algo
//        std::vector<std::vector<double>> distMatrix(tessBaryCoords.size());
//        for (auto& col : distMatrix) {
//            col.reserve(featureBaryCoords.size());
//        }
//
//        //Generate edges between sides of the graph
//        for (int i = 0; i < tessBaryCoords.size(); i++) {
//            for (auto fVert : featureBaryCoords) {
//                auto tVert = tessBaryCoords.at(i);
//                double dist = barycentric_distance(bary, tVert, fVert);
//                distMatrix.at(i).push_back(dist);
//            }
//        }
//
//        HungarianAlgorithm hungarian;
//        std::vector<int> assignment;
//        hungarian.Solve(distMatrix, assignment);
//
//        std::unordered_map<SM_vertex_descriptor, SM_vertex_descriptor> tessToFeatureVD;
//        //Move
//        std::vector<SM_vertex_descriptor> unmatchedVerts;
//        for (int i = 0; i < assignment.size(); i++) {
//            auto vd = tessBaryIdxToVD.at(i);
//            int ass = assignment.at(i);
//            origPoints.insert({vd, tessTriMesh.point(vd)});
//            if (ass != -1) {
//                tessToFeatureVD.insert({vd, featureVDs.at(ass)});
//                //Move to matching feature
//                auto newCoord = featurePoints.at(ass);
//                newCoords.insert({vd, Point_3(newCoord.x, newCoord.y, newCoord.z)});
//                tessTriMesh.point(vd) = {newCoord.x, newCoord.y, newCoord.z};
//            } else {
//                tessToFeatureVD.insert({vd, SM_vertex_descriptor(-1)});
//                unmatchedVerts.push_back(vd);
//            }
//        }
//        //Validate
//        for (auto face : tessTriMesh.faces()) {
//            Vector norm = get(triFNorms, face);
//            double dot = 0;
//            do {
//                Vector newNorm = CGAL::Polygon_mesh_processing::compute_face_normal(face, tessTriMesh);
//                dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();
//
//                if (dot < 0) {
//                    SM_vertex_descriptor minVD;
//                    double minCurvature = std::numeric_limits<double>::max();
//                    for (auto vd : tessTriMesh.vertices_around_face(tessTriMesh.halfedge(face))) {
//                        auto featureVD = tessToFeatureVD.at(vd);
//                        if (featureVD.idx() != -1 && get(gaussMap, featureVD) < minCurvature) {
//                            minCurvature = get(gaussMap, featureVD);
//                            minVD = vd;
//                        }
//                    }
//
//                    tessTriMesh.point(minVD) = origPoints.at(minVD);
//                }
//            } while(dot < 0);
//        }
//
//        CGAL::draw(tessTriMesh);
//
////        auto featureVerts = extractFeatureVertices(pointSet, triRange, *highResMesh);
////        TessellationData matching = minBiGraphMatch(triPoints, featureVerts);
//
//
//        std::cout << "tessellated face" << std::endl;
    }

    std::cout << count << std::endl;

    std::cout << "Done";
    return 0;
//    glfwInit();
//    glfwSetErrorCallback(GLFWErrorCallback);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//
//    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Tessellation", nullptr, nullptr);
//    if (window == nullptr)
//    {
//        std::cout << "Failed to create GLFW window" << std::endl;
//        glfwTerminate();
//        return -1;
//    }
//    glfwMakeContextCurrent(window);
//    glfwSetCursorPosCallback(window, mouse_callback);
//    glfwSetScrollCallback(window, scroll_callback);
//
//    // tell GLFW to capture our mouse
//    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//
//    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//    {
//        std::cout << "Failed to initialize GLAD" << std::endl;
//        return -1;
//    }
//
//    glViewport(0, 0, WIDTH, HEIGHT);
//
//    // Load Shader
//    auto default_shader = Shader("../Shaders/default_vertex.glsl", "../Shaders/default_frag.glsl");
//
//    // Load Model
//    // Dragon
//    auto dragon_model = new Model("../Models/dragon.obj");
//    auto meshProcessor = MeshProcessor();
//    meshProcessor.DecimateInit(dragon_model->GetMeshes().at(0));
//    // Bunny
//    //auto bunny_model = new Model("../Models/cube.obj");
//
//
//    glm::mat4 model = glm::mat4(1.0f);
//    model = glm::rotate(glm::scale(model, glm::vec3{0.3f, 0.3f, 0.3f}), 45.0f, glm::vec3{0.0f, 1.0f, 0.0f});
//
//    glm::mat4 projection;
//    projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
//
//    // Render Loop
//    while(!glfwWindowShouldClose(window))
//    {
//        float currentFrame = glfwGetTime();
//        deltaTime = currentFrame - lastFrame;
//        lastFrame = currentFrame;
//
//        processInput(window);
//        glEnable(GL_CULL_FACE);
//        glEnable(GL_DEPTH_TEST);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        //glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
//
//
//        default_shader.use();
//        default_shader.setMat4("model", model);
//        default_shader.setMat4("view", camera.GetViewMatrix());
//        default_shader.setMat4("projection", projection);
//
//        //dragon_model->Draw(default_shader);
//        //bunny_model->Draw(default_shader);
//
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//
//    }
//
//    glfwTerminate();
//    return 0;
}

//void processInput(GLFWwindow *window)
//{
//    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
//        glfwSetWindowShouldClose(window, true);
//
//    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
//        camera.ProcessKeyboard(FORWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
//        camera.ProcessKeyboard(BACKWARD, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
//        camera.ProcessKeyboard(LEFT, deltaTime);
//    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
//        camera.ProcessKeyboard(RIGHT, deltaTime);
//}
//
//void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
//{
//    float xpos = static_cast<float>(xposIn);
//    float ypos = static_cast<float>(yposIn);
//
//    if (firstMouse)
//    {
//        lastX = xpos;
//        lastY = ypos;
//        firstMouse = false;
//    }
//
//    float xoffset = xpos - lastX;
//    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
//
//    lastX = xpos;
//    lastY = ypos;
//
//    camera.ProcessMouseMovement(xoffset, yoffset);
//}
//
//// glfw: whenever the mouse scroll wheel scrolls, this callback is called
//// ----------------------------------------------------------------------
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
//{
//    camera.ProcessMouseScroll(static_cast<float>(yoffset));
//}

//int main(int argc, char** argv)
//{
//    CGAL::Timer task_timer;
//    task_timer.start();
//
//    const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path("../Models.dradon.obj");
//
//    SurfaceMesh sm;
//    if(!CGAL::IO::read_polygon_mesh(filename, sm))
//    {
//        std::cerr << "Invalid input file." << std::endl;
//        return EXIT_FAILURE;
//    }
//
//    // Selection file that contains the cones and possibly the path between cones
//    // -- the first line for the cones indices
//    // -- the second line must be empty
//    // -- the third line optionally provides the seam edges indices as 'e11 e12 e21 e22 e31 e32' etc.
////    const char* cone_filename = (argc>2) ? argv[2] : "data/bear.selection.txt";
//
//    // Read the cones and compute their corresponding vertex_descriptor in the underlying mesh 'sm'
//    std::vector<SM_vertex_descriptor> cone_sm_vds;
////    SMP::read_cones<SurfaceMesh>(sm, cone_filename, std::back_inserter(cone_sm_vds));
//    cone_sm_vds.emplace_back(0);
//    cone_sm_vds.emplace_back(5000);
//    cone_sm_vds.emplace_back(10000);
//    cone_sm_vds.emplace_back(11000);
//    // Two property maps to store the seam edges and vertices
//    Seam_edge_pmap seam_edge_pm = sm.add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
//    Seam_vertex_pmap seam_vertex_pm = sm.add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;
//
//    // The seam mesh
//    Mesh mesh(sm, seam_edge_pm, seam_vertex_pm);
//
//    // If provided, use the path between cones to create a seam mesh
//    std::cout << "No seams given in input, computing the shortest paths between consecutive cones" << std::endl;
//    std::list<SM_edge_descriptor> seam_edges;
//    SMP::compute_shortest_paths_between_cones(sm, cone_sm_vds.begin(), cone_sm_vds.end(), seam_edges);
//
//    // Add the seams to the seam mesh
//    for(SM_edge_descriptor e : seam_edges) {
//        mesh.add_seam(source(e, sm), target(e, sm));
//    }
//
//    std::cout << mesh.number_of_seam_edges() << " seam edges in input" << std::endl;
//
//    // Index map of the seam mesh (assuming a single connected component so far)
//    typedef std::unordered_map<vertex_descriptor, int> Indices;
//    Indices indices;
//    boost::associative_property_map<Indices> vimap(indices);
//    int counter = 0;
//    for(vertex_descriptor vd : vertices(mesh)) {
//        put(vimap, vd, counter++);
//    }
//
//    // Mark the cones in the seam mesh
//    std::unordered_map<vertex_descriptor, SMP::Cone_type> cmap;
//    SMP::locate_cones(mesh, cone_sm_vds.begin(), cone_sm_vds.end(), cmap);
//
//    // The 2D points of the uv parametrisation will be written into this map
//    // Note that this is a halfedge property map, and that uv values
//    // are only stored for the canonical halfedges representing a vertex
//    UV_pmap uvmap = sm.add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
//
//    // Parameterizer
//    typedef SMP::Orbifold_Tutte_parameterizer_3<Mesh>         Parameterizer;
//    Parameterizer parameterizer(SMP::Triangle, SMP::Cotangent);
//
//    // a halfedge on the (possibly virtual) border
//    // only used in output (will also be used to handle multiple connected components in the future)
//    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
//
//    parameterizer.parameterize(mesh, bhd, cmap, uvmap, vimap);
//
//    std::cout << "Finished in " << task_timer.time() << " seconds" << std::endl;
//    return EXIT_SUCCESS;
//}

//std::unordered_map<Point_2, unsigned int> num_points;
//std::unordered_map<Point_2, std::unordered_set<unsigned int>> vToUV;
//std::vector<Point_2> vec;
//for (auto hd : sm->halfedges()) {
//vec.push_back(uvmap[hd]);
//if (num_points.find(uvmap[hd]) == num_points.end()) {
//num_points.insert({uvmap[hd], 1});
//vToUV.insert({uvmap[hd], {}});
//} else {
//num_points.at(uvmap[hd]) = num_points.at(uvmap[hd]) + 1;
//vToUV.at(uvmap[hd]).insert(sm->source(hd));
//}
//}
//
//std::vector<std::pair<Point_2, unsigned int>> elems(num_points.begin(), num_points.end());
//std::sort(elems.begin(), elems.end(), [](auto a, auto b) {
//return a.second > b.second;
//});
//
//std::vector<std::pair<Point_2, std::unordered_set<unsigned int>>> vertices(vToUV.begin(), vToUV.end());
//std::sort(vertices.begin(), vertices.end(), [](auto a, auto b) {
//return a.second.size() > b.second.size();
//});
//
//unsigned int dups;
//std::unordered_set<unsigned int> dup_v;
//for (auto e : vertices) {
//if (e.second.size() <= 1) {
//continue;
//}
//dups += e.second.size();
//auto pnum = num_points.at(e.first);
//std::cout << e.first << " -> " << pnum << " halfedges, " << e.second.size()<< " vertices, ( ";
//for (auto v : vToUV.at(e.first)) {
//std::cout << v << " ";
//dup_v.insert(v);
//}
//std::cout << ")" << std::endl;
//}
//
//std::cout << dups << " dups, " << dup_v.size() << " unique v" << std::endl;