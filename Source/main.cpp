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
#include <igl/IO>
#include <igl/gaussian_curvature.h>
#include <igl/massmatrix.h>
#include <igl/invert_diag.h>
#include <Evaluation.h>
#include <glm/gtx/string_cast.hpp>


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

const int WIDTH = 1260;
const int HEIGHT = 720;
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
    bool useTestTex = false;
    bool testView = false;
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
                               {7, 7, 7, 7},
//                               {2, 2, 2, 3},
//                               {3, 3, 3, 3},
//                               {3, 3, 3, 4},
                       }};

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
    bool EXPORT_TESSELLATED_MAPPING = true;

    fs::path dataDir;
    try {
        TCLAP::CmdLine cmd("Program Description", ' ', "0.1");
        TCLAP::ValueArg<std::string> dataDirArg("d", "data-dir", "Directory for model data", true, "./data", "string");
        cmd.add(dataDirArg);

        TCLAP::SwitchArg exportTessMeshSwitch("m", "export-mesh", "Export tessellated mesh", cmd, true);
        TCLAP::SwitchArg exportProcessedFacesSwitch("p", "export-faces", "Export processed faces", cmd, false);
        TCLAP::SwitchArg exportTextureSwitch("t", "export-texture", "Export texture", cmd, true);

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
//    const std::string inSimplifiedBase = modelBase + "simple"/* + percentStr*/;
    const std::string selectionsFilename = !PRE_SIMPLIFIED ? CGAL::data_file_path(modelBase + "edge.selection.txt") : "";
//    const std::string simplifiedSelectionsFilename = CGAL::data_file_path(inSimplifiedBase + ".selection.txt");

//    const std::string outSimplifiedFilename = CGAL::data_file_path(outSimplifiedBase + ".obj");

    std::string mapFilename = dataDir / "MapOrigtoSimple.txt";
    std::string inSimplifiedFilename = dataDir / "simple.obj";
    std::string inOriginalFilename = dataDir / "original.obj";
    std::string inEdgeSelectionsFilename = dataDir / "edge.selection.txt";



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

    sm = IO::fromOBJ(inSimplifiedFilename, true, false);
    IO::AddSeamsFromFile(sm, inEdgeSelectionsFilename);
    highResMesh = IO::fromOBJ(inOriginalFilename);
    Eigen::MatrixXi F;
    Eigen::MatrixXd V;
    igl::readOBJ(inOriginalFilename, V, F);

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

    CGAL::Polygon_mesh_processing::compute_normals(*sm, vNorms, fNorms);

    std::vector<TessLevelData> allTessMeshes;
    for (int i = 0; i < NUM_TESS_LEVELS; i++) {
        allTessMeshes.emplace_back();
    }

    if (!IMPORT_PROCESSED_FACES)
    {
        Eigen::VectorXd K;
        igl::gaussian_curvature(V,F,K);
        Eigen::SparseMatrix<double> M,Minv;
        igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT,M);
        igl::invert_diag(M,Minv);
        // Divide by area to get integral average
        K = (Minv*K).eval();

        auto gaussMap = highResMesh->add_property_map<SM_vertex_descriptor, double>("v:curvature").first;
        for (unsigned int i = 0; i < highResMesh->num_vertices(); i++) {
            put(gaussMap, SM_vertex_descriptor(i), K(i,0));
        }

//        gaussMap = Utils::CalculateGaussianCurvature(*highResMesh);

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
            auto fNormsCopy = sm_copy->property_map<SM_face_descriptor, Vector>("f:normal").first;
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

                auto featureVerts = Strategy::extractFeatureVertices(*sm, processFace, *highResMesh, stats);


                Strategy::tessellateFace(processFace, *sm_copy, tessLevels.at(i), currTessLevel.processedEdges, stats);
                Strategy::projectEdgeVerts(processFace, *sm_copy, *highResMesh, aabbTree, currTessLevel.interpolateVerts,
                                           currTessLevel.processedEdges, stats);

                Strategy::minBiGraphMatch(processFace, featureVerts, stats);
                Strategy::moveAndValidate(processFace, *sm_copy, *highResMesh, gaussMap, stats); // Not validating flipped faces well? Maybe fixed
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

//            Strategy::calculateUVs(*sm_copy, *sm, currTessLevel.processedFaces);

            
            auto tLevel = tessLevels.at(i);
            std::stringstream tessLevelStr;
            tessLevelStr << i << "-" << tLevel.ol0 << "-" << tLevel.ol1 << "-" << tLevel.ol2 << "-" << tLevel.il;

            if (EXPORT_TESSELLATED_MESH) {
                std::string name = tessLevelStr.str() + ".obj";
                std::ofstream finalOut(dataDir / "out" / name);
                IO::toOBJ(*sm_copy, finalOut, "", "", dataDir / "out" / "edge.selection.txt");
            }

            if (EXPORT_PROCESSED_FACES) {
                std::string name = "processed_faces_" + tessLevelStr.str() + ".bin";
                std::ofstream processedFacesOut(dataDir / "out" / name, std::ios::binary);

                cereal::BinaryOutputArchive oarchive(processedFacesOut);
                oarchive(currTessLevel.processedFaces);
            }

            if (EXPORT_TESSELLATED_MAPPING) {
                std::string name = tessLevelStr.str() + "-mapping.txt";
                std::string blenderSelections = tessLevelStr.str() + "-selection.txt";
                std::ofstream finalOut(dataDir / "out" / name);
                std::ofstream blenderOut(dataDir / "out" / blenderSelections);

                blenderOut << "[";
                bool first = true;
                for (const auto& [fd, face] : currTessLevel.processedFaces) {
                    for (const auto& tessVert : face->tessVerts) {
                        if (tessVert->matchingFeature != nullptr) {
                            auto feature = tessVert->matchingFeature;
                            auto targetVD =  highResMesh->source(feature->hd);
                            finalOut << tessVert->vd << " " << targetVD.idx() << "\n";

                            if (first) {
                                first = false;
                                blenderOut << tessVert->vd.idx();
                            }
                            else {
                                blenderOut << "," << tessVert->vd.idx();
                            };
                        }
                    }
                }
                blenderOut << "]";
                blenderOut.flush();
                blenderOut.close();
                finalOut.flush();
                finalOut.close();
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

//    Run evaluation
    std::vector<SurfaceMeshPtr> evalMeshes;
    for (auto tessMesh : allTessMeshes) {
        evalMeshes.push_back(tessMesh.mesh);
    }
    auto errors = Evaluation::error(highResMesh, evalMeshes);

    Prepare::OGLData modelData = Prepare::toOGL(sm);
    auto oglMesh = std::make_shared<OGLMesh>(modelData);

    Prepare::OGLData tessModelData = Prepare::toOGL(allTessMeshes);
    auto testTessOglMesh = std::make_shared<OGLMesh>(tessModelData);

    struct TessTexture {
        int width, height;
        std::vector<glm::vec3> displacement, normal;
        unsigned int displacementID, normalID;
        unsigned int displaceTexUnit, normalTexUnit;
    };

    typedef std::shared_ptr<OGLMesh> OGLMeshPtr;
    typedef std::shared_ptr<TessTexture> TessTexturePtr;
    struct TessOglMesh {
        OGLMeshPtr oglMesh;
        TessTexturePtr tex;
        TessLevel tessLevel;
    };

    std::vector<TessOglMesh> tessOglMeshes;

    // Create No tessellation model
//    auto simpleTex = std::make_shared<TessTexture>(TessTexture{1024, 1024, {}, {}});
//    Prepare::createTextures(sm, {sm}, simpleTex->width, simpleTex->height, simpleTex->displacement, simpleTex->normal);
//    TessOglMesh simplifiedOglMesh {oglMesh, simpleTex, {1, 1, 1, 1}};
//    tessOglMeshes.push_back(simplifiedOglMesh);
//    IO::WriteTexture(simpleTex->normal, simpleTex->width, simpleTex->height, 0.0, 0, dataDir / "out" / "simple_nrm.tiff");

//    int tessLevelIndex = 0;
//    for (auto& tessMesh : allTessMeshes) {
//        auto tessTex = std::make_shared<TessTexture>(TessTexture{1024, 1024, {}, {}});
//        Prepare::createTextures(sm, tessMesh, tessTex->width, tessTex->height, tessTex->displacement, tessTex->normal);
//        TessOglMesh tessOglMesh {oglMesh, tessTex, tessLevels.at(tessLevelIndex)};
//        tessOglMeshes.push_back(tessOglMesh);
//        if (EXPORT_TEXTURE) {
//            auto texstr = std::to_string(tessLevelIndex) + "_tex.tiff";
//            auto nrmstr = std::to_string(tessLevelIndex) + "_nrm.tiff";
//            IO::WriteTexture(tessTex->displacement, tessTex->width, tessTex->height, 0.0, 0, dataDir / "out" / texstr);
//            IO::WriteTexture(tessTex->normal, tessTex->width, tessTex->height, 0.0, 0, dataDir / "out" / nrmstr);
//        }
//
//        tessLevelIndex++;
//    }

    Prepare::OGLData highResModelData = Prepare::toOGL(highResMesh);
    auto highResOglMesh = new OGLMesh(highResModelData);


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

    testTessOglMesh->setupMesh();
    oglMesh->setupMesh();
    highResOglMesh->setupMesh();

    int width, height, nrChannels;
    std::string nrmPath = dataDir / "checkersmall.png";
    stbi_set_flip_vertically_on_load(true);
    unsigned char *tex = stbi_load(nrmPath.c_str(), &width, &height, &nrChannels, 0);
    if (!tex) {
        std::cout << "Failed to load texture" << std::endl;
    }

    unsigned int testTextureID;

    glGenTextures(1, &testTextureID);
    glBindTexture(GL_TEXTURE_2D, testTextureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
    stbi_image_free(tex);

    // TEMPORARY
    nrmPath = dataDir / "blurred_nrm.png";
    stbi_set_flip_vertically_on_load(true);
    tex = stbi_load(nrmPath.c_str(), &width, &height, &nrChannels, 0);
    if (!tex) {
        std::cout << "Failed to load texture" << std::endl;
    }

    ///////// TEMPORARY

    int glTexIndex = 1;
    for (const auto& tessOglMesh : tessOglMeshes) {

        auto tessTex = tessOglMesh.tex;
        tessTex->displaceTexUnit = GL_TEXTURE0 + glTexIndex++;
        glGenTextures(1, &tessTex->displacementID);
        glBindTexture(GL_TEXTURE_2D, tessTex->displacementID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, tessTex->width, tessTex->height, 0, GL_RGB, GL_FLOAT, tessTex->displacement.data());

        tessTex->normalTexUnit = GL_TEXTURE0 + glTexIndex++;
        glGenTextures(1, &tessTex->normalID);
        glBindTexture(GL_TEXTURE_2D, tessTex->normalID);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        if (glTexIndex == 5) {
//            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
//        } else {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, tessTex->width, tessTex->height, 0, GL_RGB, GL_FLOAT, tessTex->normal.data());
//        }
    }


    // Load Shader
    auto standard_tess_shader = Shader(
            "../Shaders/standard/tess_vertex.glsl",
            "../Shaders/standard/tess_frag.glsl",
            "../Shaders/standard/tess_control.tesc",
            "../Shaders/standard/tess_evaluation.tese"
    );

    auto dynamic_tess_shader = Shader(
            "../Shaders/dynamic/tess_vertex.glsl",
            "../Shaders/dynamic/tess_frag.glsl",
            "../Shaders/dynamic/tess_control.tesc",
            "../Shaders/dynamic/tess_evaluation.tese"
            );

    auto plain_shader = Shader(
            "../Shaders/default_vertex.glsl",
            "../Shaders/default_frag.glsl"
    );

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::rotate(glm::scale(model, glm::vec3{0.8f, 0.8f, 0.8f}), 45.0f, glm::vec3{0.0f, 1.0f, 0.0f});


    glm::mat4 projection;
    projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100000.0f);

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

        glm::mat4 tempView = {
                {-0.463278, 0.187274, -0.866200, 0.000000},
                {0.000000, 0.977417, 0.211319, 0.000000},
                {0.886213, 0.097899, -0.452816, 0.000000},
                {20.044111, -153.227295, -64.848488, 1.000000}
        };
        dynamic_tess_shader.use();
        if (viewSettings.testView) {
            dynamic_tess_shader.setMat4("view", tempView);
        } else {
            dynamic_tess_shader.setMat4("view", camera.GetViewMatrix());
        }
        dynamic_tess_shader.setMat4("projection", projection);
        dynamic_tess_shader.setVec3("lightPos", glm::vec3(0.0, 200.0, 1000.0));
        dynamic_tess_shader.setVec3("viewPos", camera.Position);
//        dynamic_tess_shader.setBool("useTestTex", viewSettings.useTestTex);
        dynamic_tess_shader.setInt("testTextureMap", 0);
        dynamic_tess_shader.setInt("numTessLevels", 2);
//        if (viewSettings.useTestTex) {
//        glActiveTexture(GL_TEXTURE0);
//        glBindTexture(GL_TEXTURE_2D, testTextureID);
//        }

//        auto firstMesh = tessOglMeshes.at(0), secondMesh = tessOglMeshes.at(1);
        int projectionMaps[2], normalMaps[2], texResolutions[2];
        glm::ivec4 shaderTessLevels[2];
        for (int i = 0; i < NUM_TESS_LEVELS+1; i++) {
            projectionMaps[i] = tessOglMeshes.at(i).tex->displaceTexUnit;
            normalMaps[i] = tessOglMeshes.at(i).tex->normalTexUnit;
            texResolutions[i] = tessOglMeshes.at(i).tex->height;
            shaderTessLevels[i] = tessOglMeshes.at(i).tessLevel.toGLM();
        }
//        int projectionMaps[4] = {static_cast<int>(firstMesh.tex->displaceTexUnit), static_cast<int>(secondMesh.tex->displaceTexUnit)};
//        int normalMaps[4] = {static_cast<int>(firstMesh.tex->normalTexUnit), static_cast<int>(secondMesh.tex->normalTexUnit)};
//        int texWidths[4] = {firstMesh.tex->width, secondMesh.tex->width};
//        int texHeights[4] = {firstMesh.tex->height, secondMesh.tex->height};

        dynamic_tess_shader.setIntArr("vProjectionMaps", projectionMaps, 2);
        dynamic_tess_shader.setIntArr("vNormalMaps", normalMaps, 2);
        dynamic_tess_shader.setIntArr("texRes", texResolutions, 2);
        dynamic_tess_shader.setMat4("model", model);

//        glm::ivec4 shaderTessLevels[2] = {
//                firstMesh.tessLevel.toGLM(),
//                secondMesh.tessLevel.toGLM(),
//        };
        dynamic_tess_shader.setVec4Arr("tessellationLevels", shaderTessLevels, 2);

        float distances[2] = {200, 00.0};//, 50.0,0.0};
        dynamic_tess_shader.setFloatArr("tessellationLevelDistances", distances, 2);

        for (const auto& tessOglMesh : tessOglMeshes) {
            auto tessTex = tessOglMesh.tex;
            glActiveTexture(tessTex->displaceTexUnit);
            glBindTexture(GL_TEXTURE_2D, tessTex->displacementID);
            glActiveTexture(tessTex->normalTexUnit);
            glBindTexture(GL_TEXTURE_2D, tessTex->normalID);
        }

        oglMesh->draw(dynamic_tess_shader, OGLMesh::Patches);

//        plain_shader.use();
//        plain_shader.setMat4("model", glm::translate(plain_model, glm::vec3{200.0f, 0.f, 0.f}));
//        plain_shader.setMat4("view", camera.GetViewMatrix());
//        plain_shader.setMat4("projection", projection);
//        plain_shader.setVec3("lightPos", glm::vec3(0, 0.0, 1000.0));
//        plain_shader.setVec3("viewPos", camera.Position);
//        plain_shader.setBool("useTestTex", viewSettings.useTestTex);
//        plain_shader.setInt("testTextureMap", 0);
//        if (viewSettings.showLines) {
//            plain_shader.setBool("inWireframe", true);
//        }
//        else {
//            plain_shader.setBool("inWireframe", false);
//        }
//
//        testTessOglMesh->draw(plain_shader, OGLMesh::Triangles);

//        standard_tess_shader.use();
//        standard_tess_shader.setMat4("model", glm::translate(model, glm::vec3{200.0f, 0.f, 0.f}));
//        standard_tess_shader.setMat4("view", camera.GetViewMatrix());
//        standard_tess_shader.setMat4("projection", projection);
//        standard_tess_shader.setVec3("lightPos", glm::vec3(0, 0.0, 1000.0));
//        standard_tess_shader.setVec3("viewPos", camera.Position);
//        standard_tess_shader.setBool("useTestTex", viewSettings.useTestTex);
//        standard_tess_shader.setInt("testTextureMap", 0);
//        standard_tess_shader.setInt("vProjectionMap", firstMesh.tex->displaceTexUnit);
//        standard_tess_shader.setInt("vNormalMap", firstMesh.tex->normalTexUnit);
//        standard_tess_shader.setInt("texRes", firstMesh.tex->width);
//        standard_tess_shader.setBool("inWireframe", viewSettings.showLines);
//
//        oglMesh->draw(standard_tess_shader, OGLMesh::Patches);

        plain_shader.use();
        plain_shader.setMat4("view", camera.GetViewMatrix());
        plain_shader.setMat4("projection", projection);
        plain_shader.setVec3("lightPos", glm::vec3(0, 0.0, 1000.0));
        plain_shader.setVec3("viewPos", camera.Position);
        plain_shader.setBool("useTestTex", viewSettings.useTestTex);
        plain_shader.setInt("testTextureMap", 0);
        plain_shader.setBool("inWireframe", viewSettings.showLines);

        plain_shader.setMat4("model", glm::translate(model, glm::vec3{-200.0f, 0.f, 0.f}));
        highResOglMesh->draw(plain_shader, OGLMesh::Triangles);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

auto m_key_state = GLFW_RELEASE;
std::unordered_map<int, int> key_states = {
        {GLFW_KEY_M, GLFW_RELEASE},
        {GLFW_KEY_T, GLFW_RELEASE},
        {GLFW_KEY_C, GLFW_RELEASE},
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

    if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS)
        key_states.at(GLFW_KEY_T) = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE && key_states.at(GLFW_KEY_T) == GLFW_PRESS) {
        viewSettings.useTestTex = !viewSettings.useTestTex;
        key_states.at(GLFW_KEY_T) = GLFW_RELEASE;
    }

    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        key_states.at(GLFW_KEY_C) = GLFW_PRESS;
    else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_RELEASE && key_states.at(GLFW_KEY_C) == GLFW_PRESS) {
        std::cout << glm::to_string(camera.GetViewMatrix()) << std::endl;
        viewSettings.testView = !viewSettings.testView;
        key_states.at(GLFW_KEY_C) = GLFW_RELEASE;
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