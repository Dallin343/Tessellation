#define STB_IMAGE_IMPLEMENTATION
#include "stb.h"
#include <iostream>
#include <glad/glad.h>
#include "Shader.h"
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <utility>
#include "Camera.h"
#include "MeshImpl.h"
#include <Hungarian.h>
#include "IO.h"
#include "Utils.h"
#include "Strategy.h"
#include "Prepare.h"
#include "OGLMesh.h"
//#include <indicators.hpp>

#define DRAW_STEPS false
#define DRAW_T false
#define DRAW_EP false
#define DRAW_MV false
#define DRAW_PV false

const int WIDTH = 800;
const int HEIGHT = 600;
unsigned int pass_count = 0;

double deltaTime = 0.0f;	// Time between current frame and last frame
double lastFrame = 0.0f; // Time of last frame
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = WIDTH / 2.0f;
float lastY = HEIGHT / 2.0f;
bool firstMouse = true;


void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

static void GLFWErrorCallback(int error, const char* description) {
    std::cout << "GLFW Error " << error << " " << description << std::endl;
}

constexpr unsigned int NUM_TESS_LEVELS = 3;
typedef std::array<TessLevel, NUM_TESS_LEVELS> TessLevels;
TessLevels tessLevels {{
                               {1, 1, 1, 1}, // No tessellation
                               {10, 10, 10, 10},
                               {5, 5, 5, 5}
                       }};

int main(int argc, char** argv)
{
//    indicators::show_console_cursor(false);

    const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path("../Models/beastcorrected.obj");
    const std::string simplifiedFilename = CGAL::data_file_path("../out/beast-simplified.obj");
    const std::string selectionsFilename = CGAL::data_file_path("../Models/beastcorrected.selection.txt");

    std::cout << "Loading model from file..." << "\n";
    SurfaceMeshPtr sm, highResMesh;
    SeamMeshPtr mesh, highResSeamMesh;
    std::tie(sm, mesh) = IO::fromOBJ(filename, selectionsFilename);
    std::tie(highResMesh, highResSeamMesh) = IO::fromOBJ(filename, selectionsFilename);

    auto gaussMap = Utils::CalculateGaussianCurvature(*highResMesh);
//    auto baryMap = sm->add_property_map<SM_face_descriptor, Vertex_bary_map>("f:bary").first;

//    std::cout << "Unwrapping model with Orbifold-Tutte..." << "\n";
//    SeamMeshPtr mesh = UnwrapMesh(sm, selectionsFilename);

    auto highResUVMap = sm->property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
    auto uvmap = sm->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

    std::vector<std::pair<Point_2, SM_halfedge_descriptor>> points;
    points.reserve(sm->num_halfedges());

    for (auto hd : sm->halfedges()) {
        points.emplace_back(get(uvmap, hd), hd);
    }

    std::cout << "Simplifying model with Garland-Heckbert..." << "\n";
    auto fNorms = sm->add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;
    auto vNorms = sm->add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
    Strategy::collapseMesh<Classic_plane>(mesh, sm, 0.1);

    std::cout << "Calculating normals..." << "\n";
    CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);
    std::ofstream simpleOut(simplifiedFilename);
    IO::toOBJ(*sm, simpleOut);
    return EXIT_SUCCESS;

//    std::ofstream testOut("../Models/beast-export-test.obj");
//    IO::toOBJ(*sm, testOut);

    std::cout << "Creating AABB Tree..." << "\n";
    Tree aabbTree(highResMesh->faces().begin(), highResMesh->faces().end(), *highResMesh);

    std::cout << "Finding feature vertices in simplified triangles..." << "\n";
    Point2Set pointSet;
    pointSet.insert(points.begin(), points.end());

    SurfaceMesh sm_copy = SurfaceMesh(*sm);

    auto fColor = sm_copy.add_property_map<SM_face_descriptor, CGAL::IO::Color>("f:color", CGAL::IO::Color(0x44, 0x44, 0x44)).first;

    TessEdgeMap processedEdges;
    ProcessFaceMap processedFaces;
    VertSet interpolateVerts;
    Strategy::Stats stats;
//    indicators::BlockProgressBar bar {
//            indicators::option::BarWidth{80},
//            indicators::option::ForegroundColor{indicators::Color::white},
//            indicators::option::FontStyles{
//                    std::vector<indicators::FontStyle>{indicators::FontStyle::bold}},
//            indicators::option::MaxProgress{sm->faces().size()}
//    };

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

        processedFaces.insert({fd, processFace});

        auto featureVerts = Strategy::extractFeatureVertices(pointSet, processFace, *highResMesh, stats);


        Strategy::tessellateFace(processFace, sm_copy, tessLevels.at(2), processedEdges, stats);
        Strategy::projectEdgeVerts(processFace, sm_copy, aabbTree, interpolateVerts, processedEdges, stats);
        Strategy::minBiGraphMatch(processFace, featureVerts, stats);
        Strategy::moveAndValidate(processFace, sm_copy, *sm, gaussMap, stats);
        Strategy::projectAndValidate(processFace, sm_copy, *sm, aabbTree, interpolateVerts, stats);

//        bar.tick();
//        if (pass_count % 1000 == 0) {
//            CGAL::draw(sm_copy);
//        }
    }
//    bar.mark_as_completed();

//    CGAL::draw(sm_copy);
    stats.print();

    auto newFNorms = sm_copy.property_map<SM_face_descriptor, Vector>("f:normal").first;
    auto newVNorms = sm_copy.property_map<SM_vertex_descriptor, Vector>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_normals(sm_copy, newVNorms, newFNorms);

    std::ofstream finalOut("../out/beast-export.obj");
    IO::toOBJ(sm_copy, finalOut);

    std::cout << "Done\n" << "Number of interpolated Vertices: " << interpolateVerts.size();
    std::cout << "\nNumber of edges in original: " << sm->number_of_edges() << std::endl;
//    indicators::show_console_cursor(true);

    glfwInit();
    glfwSetErrorCallback(GLFWErrorCallback);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Tessellation", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);

    Prepare::OGLData modelData = Prepare::toOGL(*sm);
    auto oglMesh = new OGLMesh(modelData);

    //Texture Setup
//    int width=4096, height=4096, maxVal;
//    auto tex = Prepare::createTexture(*sm, processedFaces, width, height, maxVal);
//    IO::WriteTexture(tex, width, height, maxVal, "../out/tex.ppm");
    int width, height, nrChannels;
    unsigned char *tex = stbi_load("../Models/testTexture.png", &width, &height, &nrChannels, 0);
    if (!tex) {
        std::cout << "Failed to load texture" << std::endl;
    }

    unsigned int texture;

    glGenTextures(1, &texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
    stbi_image_free(tex);

    // Load Shader
    auto default_shader = Shader(
            "../Shaders/default_vertex.glsl",
            "../Shaders/default_frag.glsl"/*,
            "../Shaders/basic_tessellation.tesc",
            "../Shaders/basic_tessellation.tese"*/
            );

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::rotate(glm::scale(model, glm::vec3{0.8f, 0.8f, 0.8f}), 45.0f, glm::vec3{0.0f, 1.0f, 0.0f});

    glm::mat4 projection;
    projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 10000.0f);

    // Render Loop
    while(!glfwWindowShouldClose(window))
    {
        double currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);
        glEnable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        glPatchParameteri(GL_PATCH_VERTICES, 3);

//        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

        default_shader.use();
        default_shader.setMat4("model", model);
        default_shader.setMat4("view", camera.GetViewMatrix());
        default_shader.setMat4("projection", projection);
        default_shader.setInt("vProjectionMap", 0);
        default_shader.setInt("texWidth", width);
        default_shader.setInt("texHeight", height);

        glBindTexture(GL_TEXTURE_2D, texture);
//        glBindVertexArray(VAO);
//        glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, nullptr);
        oglMesh->draw(default_shader);
        //dragon_model->Draw(default_shader);
        //bunny_model->Draw(default_shader);

        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

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