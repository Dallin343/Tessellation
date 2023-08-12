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
#include "Decimator.h"
#include "Debug.h"
#include "Mapping.h"
#include <tclap/CmdLine.h>


////// MAPPING
//////

#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/unordered_map.hpp>
#include <filesystem>


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
    float displacementThreshold = 5.0f;
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
                               {5, 5, 5, 5},
//                               {10, 10, 10, 10}
                       }};

//void testMapping(const std::string& data_path, const std::string& out_path, const std::string& name) {
//    using namespace SurfaceMaps;
//    fs::path dpath = data_path;
//    fs::path opath = out_path;
//    fs::path output_dir = opath / "approximation";
//    const fs::path mesh_path_A = dpath / name / "-simple.obj";
//    const fs::path mesh_path_B = dpath / name / ".obj";
//    const fs::path landmarks_path_A = dpath / name / "-simple.pinned";
//    const fs::path landmarks_path_B = dpath / name / ".pinned";
//    const fs::path embedding_path_A = output_dir / "embedding_A.obj";
//    const fs::path embedding_path_B = output_dir / "embedding_B.obj";
//
//    // Init map
//    MapState map_state;
//    init_map(map_state, { mesh_path_A, mesh_path_B }, { landmarks_path_A, landmarks_path_B }, { embedding_path_A, embedding_path_B }, false);
//
//    // Optimize map
//    landmark_phase(map_state);
//    release_landmarks(map_state);
//    coarse_phase(map_state);
//    fine_phase(map_state);
//
////    AdaptiveTriangulationsSettings fine_settings = fine_phase_settings();
////    fine_settings.w_approx = w_approx;
////    optimize_with_remeshing(map_state, fine_settings, "");
//}

void testDecimate(const SurfaceMeshPtr& mesh, double ratio) {
    auto edgeCostMap = mesh->property_map<SM_edge_descriptor, double>("e:collapse_cost").first;
    auto featureFaceMap = mesh->property_map<SM_face_descriptor, std::unordered_set<FeaturePointPtr>>("f:features").first;
    auto vertexQMap = mesh->property_map<SM_vertex_descriptor, SymmetricMatrix>("v:q").first;
    auto faceNormMap = mesh->property_map<SM_face_descriptor, Vector>("f:normal").first;
    auto vertexNormMap = mesh->property_map<SM_vertex_descriptor, Vector>("v:normal").first;

    auto decimator = new Decimator(mesh, edgeCostMap, featureFaceMap, vertexQMap, faceNormMap, vertexNormMap);
    decimator->decimate(ratio);
}

namespace fs = std::filesystem;
int main(int argc, char** argv)
{
    indicators::show_console_cursor(false);

    const bool TEST_DECIMATE = false;
    const bool PRE_SIMPLIFIED = true;
    const bool HAS_SEAM_SELECTIONS = false;
    bool EXPORT_TESSELLATED_MESH = true;
    bool EXPORT_PROCESSED_FACES = true;
    bool IMPORT_PROCESSED_FACES = false;
    bool EXPORT_TEXTURE = true;

    fs::path dataDir;
    try {
        TCLAP::CmdLine cmd("Program Description", ' ', "0.1");
        TCLAP::ValueArg<std::string> dataDirArg("d", "data-dir", "Directory for model data", true, "./data", "string");
        TCLAP::SwitchArg exportTessMeshSwitch("m", "export-mesh", "Export tessellated mesh", cmd, true);
        TCLAP::SwitchArg exportProcessedFacesSwitch("p", "export-faces", "Export processed faces", cmd, false);
        TCLAP::SwitchArg exportTextureSwitch("t", "export-texture", "Export texture", cmd, true);
        cmd.add(dataDirArg);
        cmd.add(exportTessMeshSwitch);
        cmd.add(exportProcessedFacesSwitch);
        cmd.add(exportTextureSwitch);

        cmd.parse(argc, argv);

        dataDir = dataDirArg.getValue();
        EXPORT_TESSELLATED_MESH = exportTessMeshSwitch.getValue();
        EXPORT_PROCESSED_FACES = exportProcessedFacesSwitch.getValue();
        EXPORT_TEXTURE = exportTextureSwitch.getValue();
    } catch (TCLAP::ArgException &e) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }

    int percent = 10;
    float percentFloat = (float) percent / 100.0f;
    const std::string percentStr = std::to_string(percent);

    const std::string modelsPath = "../Models/";
    const std::string outPath = "../out/";
    const std::string modelName = "beast";
    const std::string modelBase = modelsPath + modelName + "/";
    const std::string outSimplifiedBase = outPath + modelName + "-simplifiedtest-" + percentStr;
    const std::string inSimplifiedBase = modelBase + "simple"/* + percentStr*/;
//    const std::string selectionsFilename = !PRE_SIMPLIFIED ? CGAL::data_file_path(modelBase + ".selection.txt") : "";
//    const std::string simplifiedSelectionsFilename = CGAL::data_file_path(inSimplifiedBase + ".selection.txt");

//    const std::string outSimplifiedFilename = CGAL::data_file_path(outSimplifiedBase + ".obj");

    std::string mapFilename = dataDir / "MapBtoA.txt";
    std::string inSimplifiedFilename = dataDir / "simple.obj";
    std::string inOriginalFilename = dataDir / "original.obj";



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

//    if (TEST_DECIMATE) {
//        Debug::Log::Init();
//        auto mesh = IO::fromOBJ(inOriginalFilename);
//        testDecimate(mesh, percentFloat);
//        std::ofstream simpleOut(outSimplifiedFilename);
//        IO::toOBJ(*sm, simpleOut);
//        return EXIT_SUCCESS;
//    }

//    if (!PRE_SIMPLIFIED) {
//        std::tie(sm, mesh) = IO::fromOBJ(filename, selectionsFilename);
//        std::tie(highResMesh, highResSeamMesh) = IO::fromOBJ(filename, selectionsFilename);
//    } else {
//        if (HAS_SEAM_SELECTIONS) {
//            std::tie(sm, mesh) = IO::fromOBJ(inSimplifiedFilename, simplifiedSelectionsFilename);
//        } else {
//            std::tie(sm, vdPairMap, vdHdMap) = IO::fromOBJ(inSimplifiedFilename, inVdFilename, inHdFilename, inSeamsFilename);
//        }
//        std::tie(highResMesh, highResSeamMesh) = IO::fromOBJ(filename, selectionsFilename);
//    }

    sm = IO::fromOBJ(inSimplifiedFilename, true, true);
    highResMesh = IO::fromOBJ(inOriginalFilename);

    FaceToVertsMap faceVertsMap = loadMap(mapFilename, *highResMesh, *sm);

//    FaceVertMapping faceVertMapping;
//    {
//        std::ifstream in("../Models/beast/mapping.bin", std::ios::binary);
//        cereal::BinaryInputArchive iarchive(in);
//        iarchive(faceVertMapping);
//    }

//    auto mappingMap = sm->add_property_map<SM_face_descriptor, std::vector<MapVert>>("f:feature_verts", {}).first;
    auto mappingMap = sm->add_property_map<SM_face_descriptor, std::vector<FaceMapping>>("f:feature_verts", {}).first;

    for (auto [fdIdx, baryVerts] : faceVertsMap) {
        auto fd = SM_face_descriptor(fdIdx);
        auto& featureVec = get(mappingMap, fd);
        featureVec = baryVerts;
    }

//    for (auto [fdIdx, baryVerts] : faceVertsMap) {
//        auto fd = SM_face_descriptor(fdIdx);
//        auto& featureVec = get(mappingMap, fd);
//        featureVec = baryVerts;
//        for (auto& featVert : featureVec) {
//            auto v0 = featVert.bary.v0;
//            auto v1 = featVert.bary.v1;
//            auto hd = sm->halfedge(SM_vertex_descriptor(v0), SM_vertex_descriptor(v1));
//            auto vd2 = sm->target(sm->next(hd));
//            featVert.bary.v2 = vd2;
//            featVert.bary.hd = hd;
//        }
//    }

    fNorms = sm->property_map<SM_face_descriptor, Vector>("f:normal").first;
    vNorms = sm->property_map<SM_vertex_descriptor, Vector>("v:normal").first;

    std::array<TessLevelData, NUM_TESS_LEVELS> allTessMeshes;
    for (int i = 0; i < NUM_TESS_LEVELS; i++) {
        allTessMeshes.at(i) = {};
    }

    if (!IMPORT_PROCESSED_FACES)
    {
        gaussMap = Utils::CalculateGaussianCurvature(*highResMesh);

        uvmap = sm->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        highResUVMap = highResMesh->property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

//        points.reserve(highResMesh->num_halfedges());
//
//        for (auto hd: highResMesh->halfedges()) {
//            points.emplace_back(get(highResUVMap, hd), hd);
//        }

        Point2Set pointSet;
//        pointSet.insert(points.begin(), points.end());

//        if (!PRE_SIMPLIFIED) {
//            std::cout << "Simplifying model with Garland-Heckbert..." << "\n";
//            Strategy::collapseMesh<Classic_plane>(mesh, sm, pointSet, percentFloat);
//
//            std::cout << "Calculating normals..." << "\n";
//            CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);
//
//            std::ofstream simpleOut(outSimplifiedFilename);
//            IO::toOBJ(*sm, simpleOut, outVdFilename, outHdFilename, outSeamsFilename);
//            return EXIT_SUCCESS;
//        }

        std::cout << "Creating AABB Tree..." << "\n";
        Tree aabbTree(highResMesh->faces().begin(), highResMesh->faces().end(), *highResMesh);

        std::cout << "Optimizing simplified mesh for tessellation..." << "\n";

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

                if (faceCount == 1035) {
                    int x = 1;
                }

                auto featureVerts = Strategy::extractFeatureVertices(*sm, processFace, *highResMesh, stats);


                Strategy::tessellateFace(processFace, *sm_copy, tessLevels.at(i), currTessLevel.processedEdges, stats);
                Strategy::projectEdgeVerts(processFace, *sm_copy, aabbTree, currTessLevel.interpolateVerts,
                                           currTessLevel.processedEdges, stats);
                Strategy::minBiGraphMatch(processFace, featureVerts, stats);
                Strategy::moveAndValidate(processFace, *sm_copy, *highResMesh, gaussMap, stats); // Not validating flipped faces well?
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

            std::vector<SM_vertex_descriptor> moveVs;
            auto vColorMap = sm_copy->add_property_map<SM_vertex_descriptor, glm::vec3>("v:color", {}).first;
            for (auto& [fd, face] : currTessLevel.processedFaces) {
                for (auto& vert : face->tessVerts) {
                    glm::vec3 color = {};
                    switch (vert->assignedBy) {
                        case ProjectValidate:
                            color = {0.0, 0.0, 0.0};//{1.0, 0.0, 0.0};
                            break;
                        case MoveValidate:
                            color = {1.0, 0.0, 0.0};
                            moveVs.push_back(vert->vd);
                            break;
                        case ProjectEdge:
                            color = {0.0, 0.0, 0.0};//{0.0, 0.0, 1.0};
                            break;
                        case PVEdgeCase:
                            color = {0.0, 0.0, 0.0};//{1.0, 1.0, 0.0};
                            break;
                         case InterpolateUnmatched:
                            color = {0.0, 1.0, 1.0};
                    }
                    put(vColorMap, vert->vd, color);
                }
            }

            allTessMeshes.at(i).mesh = sm_copy;

            auto newFNorms = sm_copy->property_map<SM_face_descriptor, Vector>("f:normal").first;
            auto newVNorms = sm_copy->property_map<SM_vertex_descriptor, Vector>("v:normal").first;
            CGAL::Polygon_mesh_processing::compute_normals(*sm_copy, newVNorms, newFNorms);

            bars[0].set_option(option::PostfixText{
                    std::to_string(i) + "/" + std::to_string(NUM_TESS_LEVELS) + " [" + tessBarStates.at(++state) + "]"
            });

            Strategy::calculateUVs(*sm_copy, *sm, currTessLevel.processedFaces);

            auto tLevel = tessLevels.at(i);
            std::stringstream tessLevelStr;
            tessLevelStr << i << "-" << tLevel.ol0 << "-" << tLevel.ol1 << "-" << tLevel.ol2 << "-" << tLevel.il;

            if (EXPORT_TESSELLATED_MESH) {
                std::string name = tessLevelStr.str() + ".obj";
                std::ofstream finalOut(dataDir / "out" / name);
                IO::toOBJ(*sm_copy, finalOut, "", "", "");
            }

            if (EXPORT_PROCESSED_FACES) {
                std::string name = "processed_faces_" + tessLevelStr.str() + ".bin";
                std::ofstream processedFacesOut(dataDir / "out" / name, std::ios::binary);

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

    Prepare::OGLData tessModelData = Prepare::toOGL(*allTessMeshes.at(0).mesh);
    auto tessOglMesh = new OGLMesh(tessModelData);

    Prepare::OGLData highResModelData = Prepare::toOGL(*highResMesh);
    auto highResOglMesh = new OGLMesh(highResModelData);

    //Texture Setup
    int width=8192, height=8192, maxVal;
    float offset;
    auto tex = Prepare::createTexture(*sm, allTessMeshes.at(0), width, height, offset, maxVal);
    if (EXPORT_TEXTURE) {
        IO::WriteTexture(tex, width, height, offset, maxVal, dataDir / "out" / "tex.ppm");
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
    tessOglMesh->setupMesh();
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
        tess_shader.setVec3("lightPos", glm::vec3(0.0, 0.0, 1000.0));
        tess_shader.setVec3("viewPos", camera.Position);
        tess_shader.setVec4("tessellationLevel", 5, 5, 5, 5);

        glBindTexture(GL_TEXTURE_2D, texture);
//        glBindVertexArray(VAO);
//        glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, nullptr);
        oglMesh->draw(tess_shader, OGLMesh::Patches);

        plain_shader.use();
        plain_shader.setMat4("model", glm::translate(plain_model, glm::vec3{150.0f, 0.f, 0.f}));
        plain_shader.setMat4("view", camera.GetViewMatrix());
        plain_shader.setMat4("projection", projection);
        plain_shader.setVec3("lightPos", glm::vec3(0, 0.0, 1000.0));
        plain_shader.setVec3("viewPos", camera.Position);
        if (viewSettings.showLines) {
            plain_shader.setBool("inWireframe", true);
        }
        else {
            plain_shader.setBool("inWireframe", false);
        }

        tessOglMesh->draw(plain_shader, OGLMesh::Triangles);

//        oglMesh->draw(plain_shader, OGLMesh::Triangles);

        plain_shader.setMat4("model", glm::translate(plain_model, glm::vec3{-150.0f, 0.f, 0.f}));
        plain_shader.setVec3("lightPos", glm::vec3(0, 0.0, 1000.0));
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
std::unordered_map<int, int> key_states = {
        {GLFW_KEY_M, GLFW_RELEASE},
        {GLFW_KEY_LEFT_BRACKET, GLFW_RELEASE},
        {GLFW_KEY_RIGHT_BRACKET, GLFW_RELEASE},
};
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
        key_states.at(GLFW_KEY_M) = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_RELEASE && key_states.at(GLFW_KEY_M) == GLFW_PRESS) {
        viewSettings.showLines = !viewSettings.showLines;
        key_states.at(GLFW_KEY_M) = GLFW_RELEASE;
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS)
        key_states.at(GLFW_KEY_LEFT_BRACKET) = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_RELEASE && key_states.at(GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS) {
        viewSettings.displacementThreshold -= 0.25f;
        if (viewSettings.displacementThreshold < 0.f) {
            viewSettings.displacementThreshold = 0.0f;
        }
        key_states.at(GLFW_KEY_LEFT_BRACKET) = GLFW_RELEASE;
    }

    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS)
        key_states.at(GLFW_KEY_RIGHT_BRACKET) = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_RELEASE && key_states.at(GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) {
        viewSettings.displacementThreshold += 0.25f;
        key_states.at(GLFW_KEY_RIGHT_BRACKET) = GLFW_RELEASE;
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