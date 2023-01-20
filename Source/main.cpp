#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
//#include "Model.h"
#include "Camera.h"
//#include "MeshProcessor.h"
#include "OMesh.h"
#include "Tessellator.h"
#include <CGAL/boost/graph/IO/OBJ.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Polygon_mesh_processing/measure.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> MeshClass;
typedef MeshClass::Vertex_index VIndex;
typedef MeshClass::Face_index FIndex;
typedef boost::graph_traits<MeshClass>::vertex_descriptor     vertex_descriptor;
typedef boost::graph_traits<MeshClass>::halfedge_descriptor   halfedge_descriptor;
typedef boost::graph_traits<MeshClass>::face_descriptor       face_descriptor;
namespace SMP = CGAL::Surface_mesh_parameterization;

namespace SMS = CGAL::Surface_mesh_simplification;

const int WIDTH = 800;
const int HEIGHT = 600;

float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame
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

typedef SMS::GarlandHeckbert_plane_policies<MeshClass, Kernel>                  Classic_plane;
typedef SMS::GarlandHeckbert_probabilistic_plane_policies<MeshClass, Kernel>    Prob_plane;
typedef SMS::GarlandHeckbert_triangle_policies<MeshClass, Kernel>               Classic_tri;
typedef SMS::GarlandHeckbert_probabilistic_triangle_policies<MeshClass, Kernel> Prob_tri;

template <typename GHPolicies>
void collapse_gh(MeshClass& mesh,
                 const double ratio)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    SMS::Count_ratio_stop_predicate<MeshClass> stop(ratio);
    // Garland&Heckbert simplification policies
    typedef typename GHPolicies::Get_cost                                        GH_cost;
    typedef typename GHPolicies::Get_placement                                   GH_placement;
    typedef SMS::Bounded_normal_change_placement<GH_placement>                    Bounded_GH_placement;
    GHPolicies gh_policies(mesh);
    const GH_cost& gh_cost = gh_policies.get_cost();
    const GH_placement& gh_placement = gh_policies.get_placement();
    Bounded_GH_placement placement(gh_placement);
    int r = SMS::edge_collapse(mesh, stop,
                               CGAL::parameters::get_cost(gh_cost)
                                       .get_placement(placement));
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::cout << "Time elapsed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
              << "ms" << std::endl;
    std::cout << "\nFinished!\n" << r << " edges removed.\n" << edges(mesh).size() << " final edges.\n";
}

int main()
{
    MeshClass myMesh;
    CGAL::IO::read_polygon_mesh("../Models/dragon.obj", myMesh);

    // a halfedge on the border
    halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(myMesh).first;
    // The UV property map that holds the parameterized values
    typedef MeshClass::Property_map<vertex_descriptor, Kernel::Point_2>  UV_pmap;
    UV_pmap uv_map = myMesh.add_property_map<vertex_descriptor, Kernel::Point_2>("h:uv").first;
    SMP::parameterize(myMesh, bhd, uv_map);
    std::ofstream out("resultFull.off");
    SMP::IO::output_uvmap_to_off(myMesh, bhd, uv_map, out);

    if (!CGAL::is_triangle_mesh(myMesh)) {
        std::cerr << "Mesh is not triangular";
        return EXIT_FAILURE;
    }

    collapse_gh<Classic_plane>(myMesh, 0.1);
    myMesh.collect_garbage();

    // a halfedge on the border
    halfedge_descriptor bhd2 = CGAL::Polygon_mesh_processing::longest_border(myMesh).first;

    UV_pmap uv_map2 = myMesh.add_property_map<vertex_descriptor, Kernel::Point_2>("h:uv").first;
    SMP::parameterize(myMesh, bhd2, uv_map2);
    std::ofstream out2("resultSimplified.off");
    SMP::IO::output_uvmap_to_off(myMesh, bhd2, uv_map2, out2);

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
    std::cout << "here" << myMesh.num_vertices();
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
