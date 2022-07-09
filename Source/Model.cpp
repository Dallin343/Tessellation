//
// Created by Dallin Hagman on 2/16/22.
//

#include <memory>
#include "Model.h"
#include "Primitives.h"

void Model::loadModel(const std::string& path)
{
    // read file via ASSIMP
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals );//aiProcess_GenSmoothNormals);
    // check for errors
    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
    {
        std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
        return;
    }

    // process ASSIMP's root node recursively
    processNode(scene->mRootNode, scene);
}

void Model::processNode(aiNode *node, const aiScene *scene)
{
    // process each mesh located at the current node
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        // the node object only contains indices to index the actual objects in the scene.
        // the scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes.push_back(processMesh(mesh, scene));
    }
    // after we've processed all of the meshes (if any) we then recursively process each of the children nodes
    for(unsigned int i = 0; i < node->mNumChildren; i++)
    {
        processNode(node->mChildren[i], scene);
    }

}

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
    // data to fill
    std::vector<unsigned int> indices;
    std::vector<VPtr> vertices;
    std::vector<EPtr> edges;
    std::vector<FPtr> faces;

    // walk through each of the mesh's vertices
    for(unsigned int i = 0; i < mesh->mNumVertices; i++)
    {
        auto vertex = std::make_shared<Vertex>();
        glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
        // positions
        vector.x = mesh->mVertices[i].x;
        vector.y = mesh->mVertices[i].y;
        vector.z = mesh->mVertices[i].z;
        vertex->SetPosition(vector);
        // normals
        if (mesh->HasNormals())
        {
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex->SetNormal(vector);
        }

        vertices.push_back(vertex);
    }
    // now wak through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
    for(unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace assimpFace = mesh->mFaces[i];
        // retrieve all indices of the assimpFace and store them in the indices vector
        for(unsigned int j = 0; j < assimpFace.mNumIndices; j+=3) {
            auto i0 = assimpFace.mIndices[j];
            auto i1 = assimpFace.mIndices[j + 1];
            auto i2 = assimpFace.mIndices[j + 2];

            indices.push_back(i0);
            indices.push_back(i1);
            indices.push_back(i2);

            // Create Edge pointers
            auto edgeA = std::make_shared<Edge>(i0, i1);
            auto edgeB = std::make_shared<Edge>(i1, i2);
            auto edgeC = std::make_shared<Edge>(i2, i0);

            // Create Face with refs to edges
            auto face = std::make_shared<Face>(edgeA, edgeB, edgeC);

            // Add Face ref to edges
            edgeA->AddFace(face);
            edgeB->AddFace(face);
            edgeC->AddFace(face);

            // Add edges to vertices
            vertices.at(i0)->AddEdge(edgeA);
            vertices.at(i0)->AddEdge(edgeC);
            vertices.at(i1)->AddEdge(edgeA);
            vertices.at(i1)->AddEdge(edgeB);
            vertices.at(i2)->AddEdge(edgeA);
            vertices.at(i2)->AddEdge(edgeC);
        }

    }

    // return a mesh object created from the extracted mesh data
    return {vertices, indices, edges, faces};
}

void Model::Draw(Shader &shader) {
    for(auto & mesh : meshes)
        mesh.Draw(shader);
}
