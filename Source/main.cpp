#include <iostream>
//#include <glad/glad.h>
//#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
//#include "Model.h"
//#include "Camera.h"
//#include "MeshProcessor.h"
//#include "OMesh.h"
//#include "Tessellator.h"
#include "MeshImpl.h"
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>

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

static void GLFWErrorCallback(int error, const char* description) {
    std::cout << "GLFW Error " << error << " " << description << std::endl;
}

struct CollapseVisitor : SMS::Edge_collapse_visitor_base<SurfaceMesh> {
    CollapseVisitor(UV_pmap& uvmap) : uvmap(uvmap) {}

    // Called during the processing phase for each edge being collapsed.
    // If placement is absent the edge is left uncollapsed.
    void OnCollapsing(const EdgeProfile& prof,
                      boost::optional<Point> placement)
    {
        if (placement) {
            auto newPoint = placement.value();
            const auto& mesh = prof.surface_mesh();
            auto seamMap = mesh.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;

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
            p0_2 = get(uvmap, prof.v0_v1());
            p1_2 = get(uvmap, prof.v1_v0());

            // 37978 26624 26622
            if (v0.idx() == 37978 || v0.idx() == 26624 || v0.idx() == 26622) {
                std::cout << "Found";
                if (prof.vR()) {
                    seamVR = prof.vR();
                }
                if (prof.vL()) {
                    seamVL = prof.vL();
                }
            }
            if (v1.idx() == 37978 || v1.idx() == 26624 || v1.idx() == 26622) {
                std::cout << "Found";
                if (prof.vR()) {
                    seamVR = prof.vR();
                }
                if (prof.vL()) {
                    seamVL = prof.vL();
                }
            }

            if (prof.vR() && seamMap[prof.vR()]) {
                seamVR = prof.vR();
            }
            if (prof.vL() && seamMap[prof.vL()]) {
                seamVL = prof.vL();
            }

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
//        if (seamVR != SurfaceMesh::null_vertex()) {
//
//            Point_2 texCoord;
//            if (vd == v0) {
//                auto v_hd = mesh.halfedge(seamVR, vd);
//            } else if (vd == v1) {
//
//            }
//            auto vR1_TexCoord = get(uvmap, vR1_hd);
//            auto vR0_TexCoord = get(uvmap, vR0_hd);
//            put(uvmap, vR2_hd, vR0_TexCoord);
//            seamVR = SurfaceMesh::null_vertex();
//        }
//
//        if (seamVL != SurfaceMesh::null_vertex()) {
//            auto vL0_hd = mesh.halfedge(seamVL, v0);
//            auto vL1_hd = mesh.halfedge(seamVL, v1);
//            auto vL2_hd = mesh.halfedge(seamVL, vd);
//
//            auto vL0_TexCoord = get(uvmap, vL0_hd);
//            auto vL1_TexCoord = get(uvmap, vL1_hd);
//            put(uvmap, vL2_hd, vL0_TexCoord);
//            seamVL = SurfaceMesh::null_vertex();
//        }

        if (vd.idx() == 37978 || vd.idx() == 26624 || vd.idx() == 26622) {
            std::cout << "Found";
        }
        for (auto hd : halfedges_around_source(vd, mesh)) {
            auto vtgt = mesh.target(hd);
            put(uvmap, hd, p_2);
        }
    }

    UV_pmap uvmap;
    SM_vertex_descriptor seamVL, seamVR, v0, v1;
    Point_3 p0, p1;
    Point_2 p0_2, p1_2, p_2;
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
    CollapseVisitor vis(uvmap);

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
            put(uvmap, mesh->halfedge(v0, v2), vt0);
            put(uvmap, mesh->halfedge(v1, v0), vt1);
            put(uvmap, mesh->halfedge(v1, v2), vt1);
            put(uvmap, mesh->halfedge(v2, v0), vt2);
            put(uvmap, mesh->halfedge(v2, v1), vt2);
        }
    }

    auto highResUVMap = mesh->add_property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
    std::copy(uvmap.begin(), uvmap.end(), highResUVMap.begin());
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

//std::shared_ptr<SeamMesh> UnwrapMesh(const std::shared_ptr<SurfaceMesh>& sm, const std::string& selectionsFile) {
//    std::vector<SM_vertex_descriptor> cone_sm_vds;
//    SMP::read_cones(*sm, selectionsFile.c_str(), std::back_inserter(cone_sm_vds));
////    cone_sm_vds.emplace_back(4033);
////    cone_sm_vds.emplace_back(172);
////    cone_sm_vds.emplace_back(1176);
//    // Two property maps to store the seam edges and vertices
//
//    Seam_edge_pmap seam_edge_pm = sm->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
//    Seam_vertex_pmap seam_vertex_pm = sm->add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;
//
//    // The seam mesh
//    auto mesh = std::make_shared<SeamMesh>(*sm, seam_edge_pm, seam_vertex_pm);
//
//    std::cout << "Computing the shortest paths between consecutive cones" << std::endl;
//    std::list<SM_edge_descriptor> seam_edges;
//    SMP::compute_shortest_paths_between_cones(*sm, cone_sm_vds.begin(), cone_sm_vds.end(), seam_edges);
//
//    // Add the seams to the seam mesh
//    for(SM_edge_descriptor e : seam_edges) {
//        mesh->add_seam(source(e, *sm), target(e, *sm));
//    }
//
//    std::cout << mesh->number_of_seam_edges() << " seam edges in input" << std::endl;
//
//    // Index map of the seam mesh (assuming a single connected component so far)
//    typedef std::unordered_map<vertex_descriptor, int> Indices;
//    Indices indices;
//    boost::associative_property_map<Indices> vimap(indices);
//    int counter = 0;
//    for(vertex_descriptor vd : vertices(*mesh)) {
//        put(vimap, vd, counter++);
//    }
//
//    typedef std::unordered_map<vertex_descriptor, bool> Parameterized;
//    Parameterized parameterized;
//    boost::associative_property_map<Parameterized> vpmap(parameterized);
//    for(vertex_descriptor vd : vertices(*mesh)) {
//        put(vpmap, vd, false);
//    }
//
//    // Mark the cones in the seam mesh
////    std::unordered_map<vertex_descriptor, SMP::Cone_type> cmap;
////    SMP::locate_cones(*mesh, cone_sm_vds.begin(), cone_sm_vds.end(), cmap);
//
//    // The 2D points of the uv parametrisation will be written into this map
//    // Note that this is a halfedge property map, and that uv values
//    // are only stored for the canonical halfedges representing a vertex
////    UV_pmap uvmap = sm->add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
//
//    // Parameterizer
////    typedef SMP::Orbifold_Tutte_parameterizer_3<SeamMesh>         Parameterizer;
//
////    Parameterizer parameterizer(twoVParameterizer, solverTraits, 1000, 1, 1e-6);
//
//    // a halfedge on the (possibly virtual) border
//    // only used in output (will also be used to handle multiple connected components in the future)
//    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(*mesh).first;
//
//    typedef SMP::Circular_border_arc_length_parameterizer_3<SeamMesh>  Border_parameterizer;
//    typedef SMP::Discrete_authalic_parameterizer_3<SeamMesh, Border_parameterizer> Parameterizer;
////    SMP::Error_code err = SMP::parameterize(*mesh, Parameterizer(), bhd, uvmap);
////    SMP::parameterize(*mesh, bhd, uvmap);
//
//    std::ofstream out("../Models/lucy-uv.off");
////    SMP::IO::output_uvmap_to_off(*mesh, bhd, uvmap, out);
//
//    return mesh;
//}

bool comp(std::pair<Point_2,int> a, std::pair<Point_2,int> b) {
    return a.second > b.second;
}

int main(int argc, char** argv)
{
    const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path("../Models/dragon-mapped.obj");
    const std::string simplifiedFilename = CGAL::data_file_path("../Models/dragon-mapped-simplified.obj");

    std::cout << "Loading model from file..." << std::endl;
    std::shared_ptr<SurfaceMesh> sm = LoadMesh(filename);

    std::cout << "Adding seams to model from selections file..." << std::endl;
    std::shared_ptr<SeamMesh> mesh = AddSeams(sm, "../Models/dragon-mapped.selection.txt");

    std::cout << "Reading UV coords from file..." << std::endl;
    std::unordered_map<unsigned int, VertexUVMap> seamUVMap;
    ReadUV(sm, filename);

//    std::cout << "Unwrapping model with Orbifold-Tutte..." << std::endl;
//    std::shared_ptr<SeamMesh> mesh = UnwrapMesh(sm, "../Models/dragon-mapped.selections.txt");

    auto highResUVMap = sm->property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
    auto uvmap = sm->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

    std::cout << "Simplifying model with Garland-Heckbert..." << std::endl;
    collapse_gh<Classic_plane>(mesh, sm, 0.5);

    std::vector<std::pair<Point_2, SM_halfedge_descriptor>> points;
    points.reserve(sm->num_vertices());

    for (auto hd : sm->halfedges()) {
        points.emplace_back(get(highResUVMap, hd), hd);
    }

    Point2Set pointSet;
    pointSet.insert(points.begin(), points.end());
    std::cout << "Points in set: " << pointSet.number_of_vertices() << ", Total UV map entries: " << points.size() << std::endl;

    for (SM_face_descriptor fd : sm->faces()) {
        std::vector<Point_2> vds;
        std::vector<SM_halfedge_descriptor> hds;
        std::vector<std::pair<unsigned int, unsigned int>> src_tgt;
        std::vector<SM_vertex_descriptor> vIdx;
        for (SM_halfedge_descriptor hd: sm->halfedges_around_face(sm->halfedge(fd))) {
            vds.push_back(uvmap[hd]);
            hds.push_back(hd);
            src_tgt.emplace_back(sm->source(hd), sm->target(hd));
            vIdx.push_back(sm->source(hd));
        }

        if (vds[0] == vds[1] || vds[1] == vds[2] || vds[0] == vds[2]) {
            std::cout<<"Vertex "<< vIdx.at(0)<<" on seam: "<<mesh->has_on_seam(vIdx.at(0)) << std::endl;
            std::cout<<"Vertex "<< vIdx.at(1)<<" on seam: "<<mesh->has_on_seam(vIdx.at(1)) << std::endl;
            std::cout<<"Vertex "<< vIdx.at(2)<<" on seam: "<<mesh->has_on_seam(vIdx.at(2)) << std::endl;
            std::cout << "Whoopsies";
        }

        std::list<Vertex_handle> vertexList;
        pointSet.range_search(vds.at(0), vds.at(1), vds.at(2), std::back_inserter(vertexList));
        std::cout << "Matches: " << vertexList.size() << std::endl;
//        for (const auto vHandle : vertexList) {
//            auto info = vHandle->info();
//            std::cout << vHandle->point() << std::endl;
//        }

    }



//    std::cout << "Writing simplified model to " << simplifiedFilename << std::endl;
//    CGAL::IO::write_OBJ(simplifiedFilename, *sm);
//
//    std::vector<SM_vertex_descriptor> cone_sm_vds;
////    for(SM_vertex_descriptor vd : vertices_around_face(sm.halfedge(SM_vertex_descriptor(10)), sm)){
////        std::cout << vd << std::endl;
////        cone_sm_vds.push_back(vd);
////    }
//    cone_sm_vds.emplace_back(4033);
//    cone_sm_vds.emplace_back(172);
//    cone_sm_vds.emplace_back(1176);
//
//    // Two property maps to store the seam edges and vertices
//    Seam_edge_pmap seam_edge_pm = sm->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
//    Seam_vertex_pmap seam_vertex_pm = sm->add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;
//
//    // The seam mesh
//    SeamMesh mesh(*sm, seam_edge_pm, seam_vertex_pm);
//
//    std::cout << "Computing the shortest paths between consecutive cones" << std::endl;
//    std::list<SM_edge_descriptor> seam_edges;
//    SMP::compute_shortest_paths_between_cones(*sm, cone_sm_vds.begin(), cone_sm_vds.end(), seam_edges);
//
//    // Add the seams to the seam mesh
//    for(SM_edge_descriptor e : seam_edges) {
//        mesh.add_seam(source(e, *sm), target(e, *sm));
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
//    UV_pmap uvmap = sm->add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
//
//    // Parameterizer
//    typedef SMP::Orbifold_Tutte_parameterizer_3<SeamMesh>         Parameterizer;
//    Parameterizer parameterizer(SMP::Triangle, SMP::Cotangent);
//
//    // a halfedge on the (possibly virtual) border
//    // only used in output (will also be used to handle multiple connected components in the future)
//    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
//
//    parameterizer.parameterize(mesh, bhd, cmap, uvmap, vimap);
//
//    std::ofstream out("result.off");
//    SMP::IO::output_uvmap_to_off(mesh, bhd, uvmap, out);

//    // a halfedge on the border
//    SMHalfedgeDescriptor bhd = CGAL::Polygon_mesh_processing::longest_border(myMesh).first;
//    // The UV property map that holds the parameterized values
//    typedef SurfaceMesh::Property_map<VertexDescriptor, Kernel::Point_2>  UV_pmap;
//    UV_pmap uv_map = myMesh.add_property_map<SMVertexDescriptor, Kernel::Point_2>("h:uv").first;
//    SMP::parameterize(myMesh, bhd, uv_map);
//    std::ofstream out("resultFull.off");
//    SMP::IO::output_uvmap_to_off(myMesh, bhd, uv_map, out);
//
//    if (!CGAL::is_triangle_mesh(myMesh)) {
//        std::cerr << "Mesh is not triangular";
//        return EXIT_FAILURE;
//    }
//
//    collapse_gh<Classic_plane>(myMesh, 0.1);
//    //myMesh.collect_garbage();
//
//    // a halfedge on the border
//    SMHalfedgeDescriptor bhd2 = CGAL::Polygon_mesh_processing::longest_border(myMesh).first;
//
//    UV_pmap uv_map2 = myMesh.add_property_map<VertexDescriptor, Kernel::Point_2>("h:uv").first;
//    SMP::parameterize(myMesh, bhd2, uv_map2);
//    std::ofstream out2("resultSimplified.off");
//    SMP::IO::output_uvmap_to_off(myMesh, bhd2, uv_map2, out2);

//    SMS::
//    glm::dvec3 a = {0.0, 1.0, 0.0};
//    glm::dvec3 b = {0.0, 0.0, 1.0};
//    glm::dvec3 c = {1.0, 0.0, 0.0};
//    Triangle tri = {a, b, c};
//    auto tessellated = Tessellator::TessellateTriangle(tri, 4, 1, 6, 5);

//    auto mesh = new OMesh("../Models/dragon.obj");
//
//    if (!mesh->IsLoaded())
//        return 1;
//
//    auto decimated = mesh->Decimated(0.1);
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
