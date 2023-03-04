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
#include "IO.h"

#define DRAW_STEPS false
#define DRAW_T false
#define DRAW_EP false
#define DRAW_MV false
#define DRAW_PV false

const int WIDTH = 800;
const int HEIGHT = 600;
unsigned int pass_count = 0;

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
    unsigned int processed_edges = 0;

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
            const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
            const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;

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
            v0vdhds.clear();
            v1vdhds.clear();

            if (get(vSeamMap, v0)) {
                for (auto hd: halfedges_around_source(v1, mesh)) {
                    auto edge = mesh.edge(prof.vL_v0());
                    auto edgeOnSeam = get(eSeamMap, edge);

                    if ((hd != prof.v1_vL() || !get(vSeamMap, prof.vL())) || (hd == prof.v1_vL() && !edgeOnSeam)) {
                        v0ds.insert(mesh.target(hd));
                    }
                }
            } else if (get(vSeamMap, v1)) {
                for (auto hd: halfedges_around_source(v0, mesh)) {
                    auto edge = mesh.edge(prof.vR_v1());
                    auto edgeOnSeam = get(eSeamMap, edge);

                    if ((hd != prof.v0_vR() || !get(vSeamMap, prof.vR())) || (hd == prof.v0_vR() && !edgeOnSeam)) {
                        v0ds.insert(mesh.target(hd));
                    }
                }
            }

//            for (auto hd: halfedges_around_source(v0, mesh)) {
////                if (hd != prof.v0_vR()) {
//                    v0ds.insert(mesh.target(hd));
//                    v0vdhds.emplace_back(mesh.target(hd), hd);
////                }
//            }
//            for (auto hd: halfedges_around_source(v1, mesh)) {
////                if (hd != prof.v1_vL()) {
//                    v1ds.insert(mesh.target(hd));
//                    v1vdhds.emplace_back(mesh.target(hd), hd);
////                }
////                v1ds.insert(mesh.target(hd));
//            }

            auto v0hd = prof.v0_v1();
            auto v1hd = prof.v1_vL();
            p0_2 = get(uvmap, v0hd);
            p1_2 = get(uvmap, v1hd);


            glm::dvec2 glmP0_2 = {p0_2.x(), p0_2.y()};
            glm::dvec2 glmP1_2 = {p1_2.x(), p1_2.y()};
//            auto uvEdgeLen = glm::length(glmP0_2 - glmP1_2);
            auto p0Vec = glmP1_2 - glmP0_2;
            glm::dvec2 newPoint_2 = glmP0_2 + (p0Vec * pp0Ratio);
            v0v1_p_2 = {newPoint_2.x, newPoint_2.y};
        }
    }

    // Called after each edge has been collapsed
    void OnCollapsed(const EdgeProfile& prof, vertex_descriptor vd)
    {
        const auto& mesh = prof.surface_mesh();
        const auto& vSeamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        const auto& eSeamMap = mesh.property_map<SM_edge_descriptor, bool>("e:on_seam").first;
        auto p = mesh.point(vd);
        auto v0Seam = get(vSeamMap, v0);
        auto v1Seam = get(vSeamMap, v1);
//        if (stats.processed_edges == 29029) {
//            int  x = 0;
//        }

//        const unsigned int num = 1;
//        const double div = 5000.0;
//        if (stats.processed_edges % num == 0 && stats.processed_edges >= 29000 && stats.processed_edges < 29050) {
//            stringstream ss;
//            ss << "../out/beast-pre-" << stats.processed_edges << ".obj";
//            std::ofstream out(ss.str());
//            IO::toOBJ(mesh, out);
//        }
        //Update seams
        if (v0Seam && vd == v1) {
            put(vSeamMap, v0, false);
            put(vSeamMap, v1, true);
            //Update edge seams
//            for (const auto& kv : v0vdhds) {
//                auto targetVd = kv.first;
//                auto old_hd = kv.second;
//                auto new_hd = mesh.halfedge(vd, targetVd);
//                if (new_hd != SurfaceMesh::null_halfedge() && get(eSeamMap, mesh.edge(old_hd))) {
//                    put(eSeamMap, mesh.edge(old_hd), false);
//                    put(eSeamMap, mesh.edge(new_hd), true);
//                }
//            }
        }
        else if (v1Seam && vd == v0) {
            put(vSeamMap, v1, false);
            put(vSeamMap, v0, true);
            //Update edge seams
//            for (const auto& kv : v1vdhds) {
//                auto targetVd = kv.first;
//                auto old_hd = kv.second;
//                auto new_hd = mesh.halfedge(vd, targetVd);
//                if (new_hd != SurfaceMesh::null_halfedge() && get(eSeamMap, mesh.edge(old_hd))) {
//                    put(eSeamMap, mesh.edge(old_hd), false);
//                    put(eSeamMap, mesh.edge(new_hd), true);
//                }
//            }
        }

        if (get(vSeamMap, vd)) {
//            stringstream ss;
//            ss << "../Models/beast-pre-seam-" << stats.processed_edges << ".obj";
//            std::ofstream seamOut(ss.str());
//            IO::toOBJ(mesh, seamOut);
            int count = 0;
            for (auto hd: halfedges_around_source(vd, mesh)) {
                auto& correctSide = v0ds;//vd == v0 ? v1ds : v0ds;
                if (correctSide.find(mesh.target(hd)) != correctSide.end()) {
//                    auto tgt = mesh.target(hd);
//                    auto src = mesh.source(hd);
//                    if (stats.processed_edges == 29029) {
//                        stringstream ss;
//                        ss << "../out/beast-pre-step-" << count << ".obj";
//                        std::ofstream out(ss.str());
//                        IO::toOBJ(mesh, out);
//                        int x = 0;
//                        std::cout << hd << "\n";
//                        std::cout << count << " " << src << " " << tgt << " " << get(vSeamMap, src) << get(vSeamMap, tgt) << "\n";
//                        std::cout << get(uvmap, hd).x() << ", " << get(uvmap,hd).y() << " => " << v0v1_p_2.x() << ", " << v0v1_p_2.y() << "\n" << std::endl;
//                    }
                    put(uvmap, hd, v0v1_p_2);
//                    if (stats.processed_edges == 29029) {
//                        stringstream ss;
//                        ss << "../out/beast-post-step-" << count++ << ".obj";
//                        std::ofstream out(ss.str());
//                        IO::toOBJ(mesh, out);
//                        int x = 0;
//                    }
                }

            }
//            stringstream ss2;
//            ss2 << "../Models/beast-post-seam-" << stats.processed_edges << ".obj";
//            std::ofstream seamPostOut(ss2.str());
//            IO::toOBJ(mesh, seamPostOut);
        }
        else {
            for (auto hd: halfedges_around_source(vd, mesh)) {
                put(uvmap, hd, v0v1_p_2);
            }
        }
//        if (stats.processed_edges % num == 0  && stats.processed_edges >= 29000 && stats.processed_edges < 29050) {
//            stringstream ss;
//            ss << "../out/beast-post-" << stats.processed_edges << ".obj";
//            std::ofstream out(ss.str());
//            IO::toOBJ(mesh, out);
//            int x = 0;
//        }
        stats.processed_edges++;
    }

    UV_pmap uvmap;
    Stats& stats;
    Vertex_bary_map vertBaryMap;
    SM_vertex_descriptor v0, v1, vR, vL;
    SM_halfedge_descriptor v0_v1, v1_v0;
    Point_3 p0, p1, pR, pL;
    Point_2 p0_2, p1_2, v0v1_p_2, v1v0_p_2;
    std::unordered_set<SM_vertex_descriptor> v0ds, v1ds;
    std::vector<std::pair<SM_vertex_descriptor, SM_halfedge_descriptor>> v0vdhds, v1vdhds;
    std::unordered_set<SM_halfedge_descriptor> v0hds, v1hds;
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

    TessellatedVert(FeatureVertPtr matchingFeature,
                    const glm::dvec3 &origCoords, const glm::dvec3 &baryCoords) :
                    matchingFeature(std::move(matchingFeature)), origCoords(origCoords), baryCoords(baryCoords) {
        undoMove();
    }

    TessellatedVert(const glm::dvec3 &origCoords, const glm::dvec3 &baryCoords) :
                    origCoords(origCoords), baryCoords(baryCoords) {
        undoMove();
    }

    void undoMove() {
        newCoords = {minDbl, minDbl, minDbl};
    }

    bool isAssigned() const {
        return newCoords != glm::dvec3(minDbl, minDbl, minDbl);
    }

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

struct TessellatedFace {
    SM_face_descriptor fd;
    std::array<TessVertPtr, 3> vertices;
};
typedef std::shared_ptr<TessellatedFace> TessFacePtr;
typedef std::unordered_map<SM_vertex_descriptor, TessVertPtr> VDToTessVert;

class ProcessEdge {
public:
    ProcessEdge(const SM_edge_descriptor& ed) : ed(ed) {}

    SM_edge_descriptor ed;
    TessVertPtr v0, v1;
    std::vector<TessVertPtr> tessVerts;
};
typedef std::shared_ptr<ProcessEdge> ProcessEdgePtr;

class ProcessFace {
public:
    ProcessFace() {}

    ProcessFace(const SM_face_descriptor &fd, const array<SM_vertex_descriptor, 3> &vds, const array<Point_2, 3> &uvs,
                const array<Point_3, 3> &coords) : fd(fd), vds(vds), uvs(uvs), coords(coords) {}

    SM_face_descriptor fd {};
    std::array<SM_vertex_descriptor, 3> vds {};
    std::array<Point_2, 3> uvs {};
    std::array<Point_3, 3> coords {};
    std::vector<TessVertPtr> tessVerts;
    std::vector<TessVertPtr> innerVerts;
    std::vector<TessFacePtr> tessFaces;
    ProcessEdgePtr e01, e02, e12;
    VDToTessVert vdToTessVert;
};
typedef std::shared_ptr<ProcessFace> ProcessFacePtr;

typedef std::vector<FeatureVertPtr> FeatureVerts;
typedef std::vector<TessVertPtr> TessellatedVerts;
typedef std::shared_ptr<TessellatedTriangle> TessTriPtr;
typedef std::unordered_set<TessVertPtr> VertSet;
typedef std::unordered_map<SM_edge_descriptor, ProcessEdgePtr> TessEdgeMap;

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

struct TessLevel {
    unsigned int ol0;
    unsigned int ol1;
    unsigned int ol2;
    unsigned int il;
};
const unsigned int NUM_TESS_LEVELS = 3;
typedef std::array<TessLevel, 3> TessLevels;

TessLevels tessLevels {{
                               {1, 1, 1, 1}, // No tessellation
                               {3, 3, 3, 3},
                               {5, 5, 5, 5}
                       }};

Vector compute_face_normal(const TessFacePtr& face, const SurfaceMesh& sm) {
    auto v0 = face->vertices.at(0);
    auto v1 = face->vertices.at(1);
    auto v2 = face->vertices.at(2);
    auto p0 = toGLM(sm.point(v0->vd));
    auto p1 = toGLM(sm.point(v1->vd));
    auto p2 = toGLM(sm.point(v2->vd));
    glm::dvec3 nrm = glm::normalize(glm::cross(p1 - p0, p2 - p0));
    return {nrm.x, nrm.y, nrm.z};
}

Vector compute_face_normal(const Point_3& v0, const Point_3& v1, const Point_3& v2) {
    Vector p0 = v1 - v0;
    Vector p1 = v2 - v0;
    Vector fNrm = normalize(CGAL::cross_product(p0, p1));

    return fNrm;
}

void tessellateFace(const ProcessFacePtr& processFace, SurfaceMesh& sm, const TessLevel& tL, TessEdgeMap& edgeVerts) {
    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;
    auto removed = sm.property_map<SM_vertex_descriptor, bool>("v:removed").first;
    auto e01hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(1)));
    auto e02hd = sm.edge(sm.halfedge(processFace->vds.at(0), processFace->vds.at(2)));
    auto e12hd = sm.edge(sm.halfedge(processFace->vds.at(1), processFace->vds.at(2)));

    bool e01_exists = edgeVerts.find(e01hd) != edgeVerts.end();
    bool e02_exists = edgeVerts.find(e02hd) != edgeVerts.end();
    bool e12_exists = edgeVerts.find(e12hd) != edgeVerts.end();

    std::vector<TessVertPtr> e01Verts, e02Verts, e12Verts;

    if (e01_exists) {
        processFace->e01 = edgeVerts.at(e01hd);
        e01Verts = processFace->e01->tessVerts;
    } else {
        auto e01 = std::make_shared<ProcessEdge>(e01hd);
        processFace->e01 = e01;
        edgeVerts.insert({e01hd, e01});
    }

    if (e02_exists) {
        processFace->e02 = edgeVerts.at(e02hd);
        e02Verts = processFace->e02->tessVerts;
    } else {
        auto e02 = std::make_shared<ProcessEdge>(e02hd);
        processFace->e02 = e02;
        edgeVerts.insert({e02hd, e02});
    }

    if (e12_exists) {
        processFace->e12 = edgeVerts.at(e12hd);
        e12Verts = processFace->e12->tessVerts;
    } else {
        auto e12 = std::make_shared<ProcessEdge>(e12hd);
        processFace->e12 = e12;
        edgeVerts.insert({e12hd, e12});
    }

    glm::dvec3 t0 = toGLM(processFace->coords.at(0));
    glm::dvec3 t1 = toGLM(processFace->coords.at(1));
    glm::dvec3 t2 = toGLM(processFace->coords.at(2));
    auto uv0 = toGLM(processFace->uvs.at(0)); // (1, 0, 0)
    auto uv1 = toGLM(processFace->uvs.at(1)); // (0, 1, 0)
    auto uv2 = toGLM(processFace->uvs.at(2)); // (0, 0, 1)

    // find barycentric coords for tessellated vertices
    Triangle baryTri = {glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0, 0.0, 1.0)};
    auto tessTri = Tessellator::TessellateTriangle(baryTri, tL.ol0, tL.ol1, tL.ol2, tL.il);

    TessellatedVerts tessellatedVerts;
    tessellatedVerts.reserve(tessTri->vertices.size());
    std::unordered_map<unsigned int, SM_vertex_descriptor> tessIdxToVd;

    unsigned int idx = 0;
    for (auto& vert : tessTri->vertices) {
        auto bary = vert;
        glm::dvec3 coord = bary.x * t0 + bary.y * t1 + bary.z * t2;
        TessVertPtr tessVert;
        if (bary.x == 1.0) {
            //At p0
            if (e01_exists) {
                tessVert = processFace->e01->v0->vd == processFace->vds.at(0) ? processFace->e01->v0 : processFace->e01->v1;
            } else if (e02_exists) {
                tessVert = processFace->e02->v0->vd == processFace->vds.at(0) ? processFace->e02->v0 : processFace->e02->v1;
            } else {
                //Neither 0-1 or 0-2 exist yet, create the tessVert
                tessVert = std::make_shared<TessellatedVert>(coord, bary);
                tessVert->vd = processFace->vds.at(0);
            }

            if (!e01_exists) {
                processFace->e01->v0 = tessVert;
            }
            if (!e02_exists) {
                processFace->e02->v0 = tessVert;
            }

            tessIdxToVd.insert({idx, tessVert->vd});
        }
        else if (bary.y == 1.0) {
            //At p1
            if (e01_exists) {
                tessVert = processFace->e01->v0->vd == processFace->vds.at(1) ? processFace->e01->v0 : processFace->e01->v1;
            } else if (e12_exists) {
                tessVert = processFace->e12->v0->vd == processFace->vds.at(1) ? processFace->e12->v0 : processFace->e12->v1;
            } else {
                //Neither 0-1 or 0-2 exist yet, create the tessVert
                tessVert = std::make_shared<TessellatedVert>(coord, bary);
                tessVert->vd = processFace->vds.at(1);
            }

            if (!e01_exists) {
                processFace->e01->v1 = tessVert;
            }
            if (!e12_exists) {
                processFace->e12->v0 = tessVert;
            }

            tessIdxToVd.insert({idx, tessVert->vd});
        }
        else if (bary.z == 1.0) {
            //At p2
            if (e02_exists) {
                tessVert = processFace->e02->v0->vd == processFace->vds.at(2) ? processFace->e02->v0 : processFace->e02->v1;
            } else if (e12_exists) {
                tessVert = processFace->e12->v0->vd == processFace->vds.at(2) ? processFace->e12->v0 : processFace->e12->v1;
            } else {
                //Neither 0-1 or 0-2 exist yet, create the tessVert
                tessVert = std::make_shared<TessellatedVert>(coord, bary);
                tessVert->vd = processFace->vds.at(2);
            }

            if (!e02_exists) {
                processFace->e02->v1 = tessVert;
            }
            if (!e12_exists) {
                processFace->e12->v1 = tessVert;
            }

            tessIdxToVd.insert({idx, tessVert->vd});
        }
        else {
            // Nonexisting vertex, so lets create it.
//            auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
            if (bary.x == 0.0) {
                // On p1 - p2 edge
                if (!e12_exists) {
                    auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                    tessVert = std::make_shared<TessellatedVert>(coord, glm::dvec3(bary.y, 0.0, 0.0));
                    tessVert->vd = vd;
                    processFace->e12->tessVerts.push_back(tessVert);

                    auto a = get(vNorms, processFace->vds.at(1));
                    auto b = get(vNorms, processFace->vds.at(2));
                    auto nrm = normalize(lerp(a, b, bary.y));
                    put(vNorms, vd, nrm);
                } else {
                    //TODO: FIX THIS!!! It'll be nasty
                    //Finds the given barycentric tessellated vert in the existing edges vector of tessVerts
                    double testBary = processFace->e12->v0->vd == processFace->vds.at(1) ? bary.y : bary.z;

                    auto i = std::find_if(processFace->e12->tessVerts.begin(), processFace->e12->tessVerts.end(), [&](const TessVertPtr& a) {
                        return abs(a->baryCoords.x - testBary) <= 1e-6;
                    });
                    tessVert = *i;
                }

            } else if (bary.y == 0.0) {
                // On p0 - p2 edge
                if (!e02_exists) {
                    auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                    tessVert = std::make_shared<TessellatedVert>(coord, glm::dvec3(bary.x, 0.0, 0.0));
                    tessVert->vd = vd;
                    processFace->e02->tessVerts.push_back(tessVert);

                    auto a = get(vNorms, processFace->vds.at(0));
                    auto b = get(vNorms, processFace->vds.at(2));
                    auto nrm = normalize(lerp(a, b, bary.x));
                    put(vNorms, vd, nrm);
                } else {
                    //TODO: FIX THIS!!! It'll be nasty
                    double testBary = processFace->e02->v0->vd == processFace->vds.at(0) ? bary.x : bary.z;

                    auto i = std::find_if(processFace->e02->tessVerts.begin(), processFace->e02->tessVerts.end(), [&](const TessVertPtr& a) {
                        return abs(a->baryCoords.x - testBary) <= 1e-6;
                    });
                    tessVert = *i;
                }
            } else if (bary.z == 0.0) {
                // On p0 - p1 edge
                if (!e01_exists) {
                    auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                    tessVert = std::make_shared<TessellatedVert>(coord, glm::dvec3(bary.x, 0.0, 0.0));
                    tessVert->vd = vd;
                    processFace->e01->tessVerts.push_back(tessVert);

                    auto a = get(vNorms, processFace->vds.at(0));
                    auto b = get(vNorms, processFace->vds.at(1));
                    auto nrm = normalize(lerp(a, b, bary.x));
                    put(vNorms, vd, nrm);
                } else {
                    //TODO: FIX THIS!!! It'll be nasty
                    double testBary = processFace->e01->v0->vd == processFace->vds.at(0) ? bary.x : bary.y;

                    auto i = std::find_if(processFace->e01->tessVerts.begin(), processFace->e01->tessVerts.end(), [&](const TessVertPtr& a) {
                        return abs(a->baryCoords.x - testBary) <= 1e-6;
                    });
                    tessVert = *i;
                }
            } else {
                // Interior
                auto vd = sm.add_vertex({coord.x, coord.y, coord.z});
                tessVert = std::make_shared<TessellatedVert>(coord, bary);
                tessVert->vd = vd;
                tessVert->isInner = true;
                processFace->innerVerts.push_back(tessVert);

                auto a = get(vNorms, processFace->vds.at(0));
                auto b = get(vNorms, processFace->vds.at(1));
                auto c = get(vNorms, processFace->vds.at(2));
                auto nrm = normalize(lerp(a, b, c, bary.x, bary.y, bary.z));
                put(vNorms, vd, nrm);
            }
        }
        tessellatedVerts.push_back(tessVert);
        processFace->vdToTessVert.insert({tessVert->vd, tessVert});
    }
    processFace->tessVerts = tessellatedVerts;

    //TODO: Maybe make new face normals based on vertex normals.
    //Add faces
    CGAL::Euler::remove_face(sm.halfedge(processFace->fd), sm);
    auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
    for (auto fIdx : tessTri->faces) {
        auto v0 = tessellatedVerts.at(fIdx.at(0));
        auto v1 = tessellatedVerts.at(fIdx.at(1));
        auto v2 = tessellatedVerts.at(fIdx.at(2));
        auto fd = sm.add_face(v0->vd, v1->vd, v2->vd);
        if (fd == SurfaceMesh::null_face()) {
            sm.add_face(v0->vd, v1->vd, v2->vd);
            continue;
        }
        auto tessellatedFace = std::make_shared<TessellatedFace>();
        tessellatedFace->fd = fd;
        tessellatedFace->vertices = {v0, v1, v2};
        processFace->tessFaces.push_back(tessellatedFace);

        //TODO: Maybe get rid of this
        //Calc face normal based on interpolated vertex normals;
        auto nrm = compute_face_normal(toPoint3(v0->origCoords), toPoint3(v1->origCoords), toPoint3(v2->origCoords));
        put(fNorms, fd, nrm);
    }
#if DRAW_STEPS || DRAW_T
    CGAL::draw(sm, "Tessellation");
#endif
}

void projectEdgeVerts(const ProcessFacePtr& processFace, SurfaceMesh& sm, Tree& aabbTree, VertSet& interpolateVerts,
                      TessEdgeMap& edgeVerts) {
    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

    //Project edges along normal to intersection
    for (const auto& vertex : processFace->tessVerts) {
        if (!vertex->isInner && !vertex->anchored && interpolateVerts.find(vertex) == interpolateVerts.end()) {
            Point_3 vPoint = sm.point(vertex->vd);
            Vector norm = get(vNorms, vertex->vd);
            auto projOnNormal = findIntersection(vPoint, norm, aabbTree);
            if (projOnNormal.has_value()) {
                vertex->newCoords = toGLM(projOnNormal.value());
                vertex->anchored = true;
                sm.point(vertex->vd) = projOnNormal.value();
            } else {
                interpolateVerts.insert(vertex);
            }
        }
    }

    //TODO: Evaluate if actuatlly necessary
    for (const auto& tessFace : processFace->tessFaces) {
        auto fNorm = compute_face_normal(tessFace, sm);
        put(fNorms, tessFace->fd, fNorm);
    }
#if DRAW_STEPS || DRAW_EP
    CGAL::draw(sm, "Edge Projection");
#endif
}

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

void minBiGraphMatch(const ProcessFacePtr& processFace, const FeatureVerts& featureVerts) {
    int ol0 = 5, ol1 = 5, ol2 = 5, il = 5;

    auto t0 = toGLM(processFace->uvs.at(0)); // (1, 0, 0)
    auto t1 = toGLM(processFace->uvs.at(1)); // (0, 1, 0)
    auto t2 = toGLM(processFace->uvs.at(2)); // (0, 0, 1)

    std::vector<glm::dvec3> innerVertBaryCoords;
    innerVertBaryCoords.reserve(processFace->innerVerts.size());

    // save inner vertex bary coords
    for (const auto& innerVert : processFace->innerVerts) {
        innerVertBaryCoords.push_back(innerVert->baryCoords);
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

        if (assign != -1) {
            processFace->innerVerts.at(i)->matchingFeature = featureVerts.at(assign);
        }
    }
}

void moveAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig, const Gauss_vertex_pmap& gaussMap) {
    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;


    glm::dvec3 t0 = toGLM(processFace->coords.at(0));
    glm::dvec3 t1 = toGLM(processFace->coords.at(1));
    glm::dvec3 t2 = toGLM(processFace->coords.at(2));

    //Move
    for (auto& innerVert : processFace->innerVerts) {
        if (innerVert->matchingFeature != nullptr) {
            glm::dvec3 newCoords = innerVert->matchingFeature->cartCoords;
            innerVert->newCoords = newCoords;
            sm.point(innerVert->vd) = {newCoords.x, newCoords.y, newCoords.z};
        }
    }

    //Validate
    for (const auto& tessFace : processFace->tessFaces) {
        Vector norm = get(fNorms, tessFace->fd);
        double dot;
        do {
            Vector newNorm = compute_face_normal(tessFace, sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
            dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

            if (dot < 0) {
                SM_vertex_descriptor minVD;
                unsigned int numEdges = 0;
                double minCurvature = std::numeric_limits<double>::max();
                for (auto vd : sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
                    auto tessVert = processFace->vdToTessVert.at(vd);
                    if (!tessVert->isInner) {
                        //This is an edge vertex, skip it
                        numEdges++;
                        continue;
                    }

                    auto feature = tessVert->matchingFeature;
                    if (feature != nullptr) {
                        auto featureVD = sm_orig.source(feature->hd);
                        if (get(gaussMap, featureVD) < minCurvature) {
                            minCurvature = get(gaussMap, featureVD);
                            minVD = vd;
                        }
                    }
                }

                if (numEdges == 2 && (minVD == SurfaceMesh::null_vertex() || processFace->vdToTessVert.at(minVD)->matchingFeature ==
                                                                                     nullptr)) {
                    std::cout<<"Edge Case\n";
                    break;
                }
                if (minVD == SurfaceMesh::null_vertex()) {
                    auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
                    put(fColor, tessFace->fd, CGAL::IO::Color(0xAA, 0x00, 0x00));
                    CGAL::draw(sm);
                    break;
                }
                auto vert = processFace->vdToTessVert.at(minVD);
                sm.point(minVD) = toPoint3(vert->origCoords);
                vert->matchingFeature = nullptr;
                vert->undoMove();
            }
        } while(dot < 0);
    }

    //Anchor vertices
    for (const auto& tessVert : processFace->innerVerts) {
        if (tessVert->matchingFeature != nullptr) {
            tessVert->anchored = true;
        }
    }
#if DRAW_STEPS || DRAW_MV
    CGAL::draw(sm, "Move and Validate");
#endif
}

void projectAndValidate(const ProcessFacePtr& processFace, SurfaceMesh& sm, SurfaceMesh& sm_orig, Tree& aabbTree, VertSet& interpolateVerts) {
    auto vNorms = sm.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

    //Project unmatched inner verts along normal to intersection
    for (auto& tessVert : processFace->innerVerts) {
        if (!tessVert->isAssigned()) {
            Point_3 vPoint = sm.point(tessVert->vd);
            Vector norm = get(vNorms, tessVert->vd);
            auto projOnNormal = findIntersection(vPoint, norm, aabbTree);
            if (projOnNormal.has_value()) {
                auto newCoords = toGLM(projOnNormal.value());
                tessVert->newCoords = newCoords;
                sm.point(tessVert->vd) = projOnNormal.value();
            }
        }
    }

    //Validate
    for (const auto& tessFace : processFace->tessFaces) {
        Vector norm = get(fNorms, tessFace->fd);
        double dot;
        do {
            Vector newNorm = compute_face_normal(tessFace, sm);//CGAL::Polygon_mesh_processing::compute_face_normal(tessFace->fd, sm);
            dot = norm.x()*newNorm.x() + norm.y()*newNorm.y() + norm.z()*newNorm.z();

            if (dot < 0) {
                SM_vertex_descriptor undoVD;
                for (auto vd : sm.vertices_around_face(sm.halfedge(tessFace->fd))) {
                    auto tessVert = processFace->vdToTessVert.at(vd);
                    if (!tessVert->isInner || tessVert->anchored) {
                        //This is an edge or anchored vertex, skip it
                        continue;
                    }
                    undoVD = vd;
                }

                if (undoVD == SurfaceMesh::null_vertex() || processFace->vdToTessVert.at(undoVD)->matchingFeature == nullptr) {
                    std::cout<<"Edge Case\n";
                    break;
                }
                if (undoVD == SurfaceMesh::null_vertex()) {
                    auto fColor = sm.property_map<SM_face_descriptor, CGAL::IO::Color>("f:color").first;
                    put(fColor, tessFace->fd, CGAL::IO::Color(0x00, 0xFF, 0x00));
                    CGAL::draw(sm);
                    break;
                } if (undoVD.idx() == 16659) {
                    CGAL::draw(sm);
                }
                auto vert = processFace->vdToTessVert.at(undoVD);
                sm.point(undoVD) = toPoint3(vert->origCoords);
                vert->matchingFeature = nullptr;
                vert->undoMove();
            }
        } while(dot < 0);
    }

    //Add unmatched edges to interpolation list
    for (const auto& innerVert : processFace->innerVerts) {
        if (!innerVert->isAssigned()) {
            interpolateVerts.insert(innerVert);
        }
    }
#if DRAW_STEPS || DRAW_PV
    CGAL::draw(sm, "Project and Validate");
#endif
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

//    std::ofstream testOut("../Models/beast-export-test.obj");
//    IO::toOBJ(*sm, testOut);

    std::cout << "Creating AABB Tree..." << std::endl;
    Tree aabbTree(highResMesh->faces().begin(), highResMesh->faces().end(), *highResMesh);

    std::cout << "Finding feature vertices in simplified triangles..." << std::endl;
    Point2Set pointSet;
    pointSet.insert(points.begin(), points.end());

    SurfaceMesh sm_copy = SurfaceMesh(*sm);
    unsigned int total = sm_copy.number_of_faces();
    auto fColor = sm_copy.add_property_map<SM_face_descriptor, CGAL::IO::Color>("f:color", CGAL::IO::Color(0x44, 0x44, 0x44)).first;

    TessEdgeMap processedEdges;
    VertSet interpolateVerts;
    for (SM_face_descriptor fd : sm->faces()) {
        auto processFace = std::make_shared<ProcessFace>();
        processFace->fd = fd;

        unsigned int arrIdx = 0;
        for (SM_halfedge_descriptor hd: sm->halfedges_around_face(sm->halfedge(fd))) {
            processFace->uvs.at(arrIdx) = get(uvmap, hd);
            processFace->coords.at(arrIdx) = sm->point(sm_copy.source(hd));
            processFace->vds.at(arrIdx) = sm->source(hd);
            arrIdx++;
        }

        auto featureVerts = extractFeatureVertices(pointSet, processFace, *highResMesh);

        tessellateFace(processFace, sm_copy, tessLevels.at(1), processedEdges);
        projectEdgeVerts(processFace, sm_copy, aabbTree, interpolateVerts, processedEdges);
        minBiGraphMatch(processFace, featureVerts);
        moveAndValidate(processFace, sm_copy, *sm, gaussMap);
        projectAndValidate(processFace, sm_copy, *sm, aabbTree, interpolateVerts);
        pass_count++;
        std::cout << pass_count << "/" << total << "\n";
//        if (pass_count % 1000 == 0) {
//            CGAL::draw(sm_copy);
//        }
    }

//    CGAL::draw(sm_copy);
//    std::cout << pass_count << std::endl;
    std::ofstream finalOut("../out/beast-export-pre-norm.obj");
    IO::toOBJ(sm_copy, finalOut);

    auto newFNorms = sm_copy.property_map<SM_face_descriptor, Vector>("f:normal").first;
    auto newVNorms = sm_copy.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_normals(sm_copy, newVNorms, newFNorms);
    std::ofstream finalNormOut("../out/beast-export-post-norm.obj");
    IO::toOBJ(sm_copy, finalNormOut);

    std::cout << "Done\n" << "Number of interpolated Vertices: " << interpolateVerts.size() << std::endl;
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