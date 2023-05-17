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
#include <indicators.hpp>

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

struct ViewSettings {
    bool showLines = false;
};

ViewSettings viewSettings;

void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

static void GLFWErrorCallback(int error, const char* description) {
    std::cout << "GLFW Error " << error << " " << description << std::endl;
}

constexpr unsigned int NUM_TESS_LEVELS = 1;
typedef std::array<TessLevel, NUM_TESS_LEVELS> TessLevels;
TessLevels tessLevels {{
//                               {1, 1, 1, 1}, // No tessellation
                               {3, 3, 3, 3},
//                               {10, 10, 10, 10}
                       }};

int main(int argc, char** argv)
{
    indicators::show_console_cursor(false);

    const bool PRE_SIMPLIFIED = true;
    const bool HAS_SEAM_SELECTIONS = true;
    const bool EXPORT_TESSELLATED_MESH = true;
    const bool EXPORT_PROCESSED_FACES = true;
    const bool IMPORT_PROCESSED_FACES = false;
    const bool EXPORT_TEXTURE = false;

    int percent = 10;
    float percentFloat = (float) percent / 100.0f;
    const std::string percentStr = std::to_string(percent);

    const std::string modelsPath = "../Models/";
    const std::string outPath = "../out/";
    const std::string modelName = "beast";
    const std::string modelBase = modelsPath + modelName;
    const std::string outSimplifiedBase = outPath + modelName + "-simplified-" + percentStr;
    const std::string inSimplifiedBase = modelBase + "-simplified-" + percentStr;
    const std::string filename = (argc>1) ? argv[1] : CGAL::data_file_path(modelBase + ".obj");
    const std::string selectionsFilename = !PRE_SIMPLIFIED ? CGAL::data_file_path(modelBase + ".selection.txt") : "";
    const std::string simplifiedSelectionsFilename = CGAL::data_file_path(inSimplifiedBase + ".selection.txt");

    const std::string outSimplifiedFilename = CGAL::data_file_path(outSimplifiedBase + ".obj");
    const std::string outHdFilename = CGAL::data_file_path(outSimplifiedBase + "-hd.txt");
    const std::string outVdFilename = CGAL::data_file_path(outSimplifiedBase + "-vd.txt");
    const std::string outSeamsFilename = CGAL::data_file_path(outSimplifiedBase + "-seams.txt");

    const std::string inSimplifiedFilename = CGAL::data_file_path(inSimplifiedBase + ".obj");
    const std::string inHdFilename = CGAL::data_file_path(inSimplifiedBase + "-hd.txt");
    const std::string inVdFilename = CGAL::data_file_path(inSimplifiedBase + "-vd.txt");
    const std::string inSeamsFilename = CGAL::data_file_path(inSimplifiedBase + "-seams.txt");

    std::cout << "Loading model from file..." << "\n";
    SurfaceMeshPtr sm, highResMesh;
    SeamMeshPtr mesh, highResSeamMesh;
    SurfaceMesh::Property_map<SM_halfedge_descriptor, Point_2> highResUVMap, uvmap;
    SurfaceMesh::Property_map<SM_face_descriptor, Vector> fNorms;
    SurfaceMesh::Property_map<SM_vertex_descriptor, Vector> vNorms;
    Gauss_vertex_pmap gaussMap;
    IO::VdHdMap vdHdMap;
    IO::VdMap vdPairMap;

    std::vector<std::pair<Point_2, SM_halfedge_descriptor>> points;

    if (!PRE_SIMPLIFIED) {
        std::tie(sm, mesh) = IO::fromOBJ(filename, selectionsFilename);
        std::tie(highResMesh, highResSeamMesh) = IO::fromOBJ(filename, selectionsFilename);
    } else {
        if (HAS_SEAM_SELECTIONS) {
            std::tie(sm, mesh) = IO::fromOBJ(inSimplifiedFilename, simplifiedSelectionsFilename);
        } else {
            std::tie(sm, vdPairMap, vdHdMap) = IO::fromOBJ(inSimplifiedFilename, inVdFilename, inHdFilename, inSeamsFilename);
        }
        std::tie(highResMesh, highResSeamMesh) = IO::fromOBJ(filename, selectionsFilename);
    }

    fNorms = sm->property_map<SM_face_descriptor, Vector>("f:normal").first;
    vNorms = sm->property_map<SM_vertex_descriptor, Vector>("v:normal").first;

    struct TessLevelData {
        SurfaceMeshPtr mesh;
        TessEdgeMap processedEdges;
        ProcessFaceMap processedFaces;
        VertSet interpolateVerts;
    };

    std::array<TessLevelData, NUM_TESS_LEVELS> allTessMeshes;
    for (int i = 0; i < NUM_TESS_LEVELS; i++) {
        allTessMeshes.at(i) = {};
    }

    if (!IMPORT_PROCESSED_FACES)
    {
        gaussMap = Utils::CalculateGaussianCurvature(*highResMesh);

        uvmap = sm->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        highResUVMap = highResMesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

        points.reserve(highResMesh->num_halfedges());

        for (auto hd: highResMesh->halfedges()) {
            points.emplace_back(get(highResUVMap, hd), hd);
        }

        if (!PRE_SIMPLIFIED) {
            std::cout << "Simplifying model with Garland-Heckbert..." << "\n";
            Strategy::collapseMesh<Classic_plane>(mesh, sm, percentFloat);

            std::cout << "Calculating normals..." << "\n";
            CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);

            std::ofstream simpleOut(outSimplifiedFilename);
            IO::toOBJ(*sm, simpleOut, outVdFilename, outHdFilename, outSeamsFilename);
            return EXIT_SUCCESS;
        }

        std::cout << "Creating AABB Tree..." << "\n";
        Tree aabbTree(highResMesh->faces().begin(), highResMesh->faces().end(), *highResMesh);

        std::cout << "Optimizing simplified mesh for tessellation..." << "\n";
        Point2Set pointSet;
        pointSet.insert(points.begin(), points.end());

        Strategy::Stats stats;
        using namespace indicators;
        ProgressBar tessLevelsBar{
                option::BarWidth{80},
                option::Start{"["},
                option::Fill{"#"},
                option::ForegroundColor{Color::white},
                option::FontStyles{
                        std::vector<FontStyle>{FontStyle::bold}},
                option::MaxProgress{NUM_TESS_LEVELS}
        };

        DynamicProgress<ProgressBar> bars{tessLevelsBar};
        bars.set_option(option::HideBarWhenComplete(true));

        std::vector<std::string> tessBarStates{
                "Processing Faces",
                "Calculating Norms",
                "Exporting Model",
        };

        for (int i = 0; i < NUM_TESS_LEVELS; i++) {
            ProgressBar faceBar{
                    option::BarWidth{80},
                    option::Start{"["},
                    option::Fill{"#"},
                    option::ForegroundColor{Color::yellow},
                    option::FontStyles{
                            std::vector<FontStyle>{FontStyle::bold}},
                    option::MaxProgress{sm->faces().size()}
            };
            bars.push_back(faceBar);

            auto &currTessLevel = allTessMeshes.at(i);
            stats.clear();

            SurfaceMeshPtr sm_copy = std::make_shared<SurfaceMesh>(*sm);

            int state = 0;
            bars[0].set_option(option::PostfixText{
                    std::to_string(i) + "/" + std::to_string(NUM_TESS_LEVELS) + " [" + tessBarStates.at(state) + "]"
            });

            auto fColor = sm_copy->add_property_map<SM_face_descriptor, CGAL::IO::Color>("f:color",
                                                                                         CGAL::IO::Color(0x44, 0x44,
                                                                                                         0x44)).first;
            int faceCount = 1;
            for (SM_face_descriptor fd: sm->faces()) {
                auto processFace = std::make_shared<ProcessFace>();
                processFace->fd = fd;

                unsigned int arrIdx = 0;
                for (SM_halfedge_descriptor hd: sm->halfedges_around_face(sm->halfedge(fd))) {
                    processFace->uvs.at(arrIdx) = get(uvmap, hd);
                    processFace->coords.at(arrIdx) = sm->point(sm_copy->source(hd));
                    processFace->vds.at(arrIdx) = sm->source(hd);
                    arrIdx++;
                }

                currTessLevel.processedFaces.insert({fd, processFace});

                auto featureVerts = Strategy::extractFeatureVertices(pointSet, processFace, *highResMesh, stats);


                Strategy::tessellateFace(processFace, *sm_copy, tessLevels.at(i), currTessLevel.processedEdges, stats);
                Strategy::projectEdgeVerts(processFace, *sm_copy, aabbTree, currTessLevel.interpolateVerts,
                                           currTessLevel.processedEdges, stats);
                Strategy::minBiGraphMatch(processFace, featureVerts, stats);
                Strategy::moveAndValidate(processFace, *sm_copy, *highResMesh, gaussMap, stats);
                Strategy::projectAndValidate(processFace, *sm_copy, *highResMesh, aabbTree, currTessLevel.interpolateVerts,
                                             stats);

                bars[i + 1].set_option(option::PostfixText{
                        std::to_string(faceCount++) + "/" + std::to_string(sm->faces().size())
                });
                bars[i + 1].tick();
            }

            if (!currTessLevel.interpolateVerts.empty()) {
                Strategy::interpolateUnmatched(currTessLevel.interpolateVerts, *sm_copy);
            }

            bars[i + 1].mark_as_completed();
            bars[0].set_option(option::PostfixText{
                    std::to_string(i) + "/" + std::to_string(NUM_TESS_LEVELS) + " [" + tessBarStates.at(++state) + "]"
            });

            allTessMeshes.at(i).mesh = sm_copy;

            auto newFNorms = sm_copy->property_map<SM_face_descriptor, Vector>("f:normal").first;
            auto newVNorms = sm_copy->property_map<SM_vertex_descriptor, Vector>("v:normal").first;
            CGAL::Polygon_mesh_processing::compute_normals(*sm_copy, newVNorms, newFNorms);

            bars[0].set_option(option::PostfixText{
                    std::to_string(i) + "/" + std::to_string(NUM_TESS_LEVELS) + " [" + tessBarStates.at(++state) + "]"
            });

            Strategy::calculateUVs(*sm_copy, *sm, currTessLevel.processedFaces);

            if (EXPORT_TESSELLATED_MESH) {
                std::ofstream finalOut("../out/beast-export-" + std::to_string(i) + ".obj");
                IO::toOBJ(*sm_copy, finalOut, "", "", "");
            }

            if (EXPORT_PROCESSED_FACES) {
                auto tLevel = tessLevels.at(i);

                std::stringstream tessLevelStr;
                tessLevelStr << i << "-" << tLevel.ol0 << "-" << tLevel.ol1 << "-" << tLevel.ol2 << "-" << tLevel.il;
                std::ofstream processedFacesOut(outSimplifiedBase + "-processed_faces-" + tessLevelStr.str() + ".bin", std::ios::binary);

                cereal::BinaryOutputArchive oarchive(processedFacesOut);
                oarchive(currTessLevel.processedFaces);
            }

            bars[0].tick();
        }
        bars[0].mark_as_completed();

        indicators::show_console_cursor(true);
        stats.print();
    }
    else {

        CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);
        for (int i = 0; i < NUM_TESS_LEVELS; i++) {
            auto tLevel = tessLevels.at(i);
            auto &currTessLevel = allTessMeshes.at(i);

            std::stringstream tessLevelStr;
            tessLevelStr << i << "-" << tLevel.ol0 << "-" << tLevel.ol1 << "-" << tLevel.ol2 << "-" << tLevel.il;
            std::ifstream processedFacesIn(outSimplifiedBase + "-processed_faces-" + tessLevelStr.str() + ".bin", std::ios::binary);

            cereal::BinaryInputArchive iarchive(processedFacesIn);
            iarchive(currTessLevel.processedFaces);
        }

    }

    std::unordered_map<AssigningSection, int> totalAssigned = {
            {ProjectEdge, 0},
            {MoveValidate, 0},
            {MVEdgeCase, 0},
            {ProjectValidate, 0},
            {PVEdgeCase, 0},
            {Undone, 0},
            {Never, 0},
            {InterpolateUnmatched, 0}
    };

    for (auto& [fd, face] : allTessMeshes.at(0).processedFaces) {
        for (auto& vert : face->tessVerts) {
            totalAssigned.at(vert->assignedBy) += 1;
        }
    }

    for (auto [key, val] : totalAssigned) {
        std::cout << Utils::SectionString(key) << ": " << val << "\n";
    }

    Prepare::OGLData modelData = Prepare::toOGL(*sm);
    auto oglMesh = new OGLMesh(modelData);

    Prepare::OGLData highResModelData = Prepare::toOGL(*highResMesh);
    auto highResOglMesh = new OGLMesh(highResModelData);

    //Texture Setup
    int width=8192, height=8192, maxVal;
    float offset;
    auto tex = Prepare::createTexture(*sm, allTessMeshes.at(0).processedFaces, width, height, offset, maxVal);
    if (EXPORT_TEXTURE) {
        IO::WriteTexture(tex, width, height, offset, maxVal, "../out/tex.ppm");
    }

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

    oglMesh->setupMesh();
    highResOglMesh->setupMesh();
//    int width, height, nrChannels;
//    unsigned char *tex = stbi_load("../Models/testTexture.png", &width, &height, &nrChannels, 0);
//    if (!tex) {
//        std::cout << "Failed to load texture" << std::endl;
//    }

//    return EXIT_SUCCESS;
    unsigned int texture;

    glGenTextures(1, &texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
//    stbi_image_free(tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, tex.data());

    // Load Shader
    auto tess_shader = Shader(
            "../Shaders/tess_vertex.glsl",
            "../Shaders/tess_frag.glsl",
            "../Shaders/basic_tessellation.tesc",
            "../Shaders/basic_tessellation.tese"
            );

    auto plain_shader = Shader(
            "../Shaders/default_vertex.glsl",
            "../Shaders/default_frag.glsl"
    );

    glm::mat4 model = glm::mat4(1.0f), plain_model(1.0f);
    model = glm::rotate(glm::scale(model, glm::vec3{0.8f, 0.8f, 0.8f}), 45.0f, glm::vec3{0.0f, 1.0f, 0.0f});
    plain_model = glm::rotate(glm::scale(plain_model, glm::vec3{0.8f, 0.8f, 0.8f}), 45.0f, glm::vec3{0.0f, 1.0f, 0.0f});


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
        glPatchParameteri(GL_PATCH_VERTICES, 3);

        if (viewSettings.showLines) {
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        }
        else {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }


        tess_shader.use();
        tess_shader.setMat4("model", model);
        tess_shader.setMat4("view", camera.GetViewMatrix());
        tess_shader.setMat4("projection", projection);
        tess_shader.setInt("vProjectionMap", 0);
        tess_shader.setInt("texWidth", width);
        tess_shader.setInt("texHeight", height);

        glBindTexture(GL_TEXTURE_2D, texture);
//        glBindVertexArray(VAO);
//        glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, nullptr);
        oglMesh->draw(tess_shader, OGLMesh::Patches);

        plain_shader.use();
        plain_shader.setMat4("model", glm::translate(plain_model, glm::vec3{150.0f, 0.f, 0.f}));
        plain_shader.setMat4("view", camera.GetViewMatrix());
        plain_shader.setMat4("projection", projection);

        oglMesh->draw(plain_shader, OGLMesh::Triangles);

        plain_shader.setMat4("model", glm::translate(plain_model, glm::vec3{-150.0f, 0.f, 0.f}));
        highResOglMesh->draw(plain_shader, OGLMesh::Triangles);
        //dragon_model->Draw(tess_shader);
        //bunny_model->Draw(tess_shader);

        glfwSwapBuffers(window);
        glfwPollEvents();

    }

    glfwTerminate();
    return 0;
}

auto m_key_state = GLFW_RELEASE;
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

    if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
        m_key_state = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_RELEASE && m_key_state == GLFW_PRESS) {
        viewSettings.showLines = !viewSettings.showLines;
        m_key_state = GLFW_RELEASE;
    }


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