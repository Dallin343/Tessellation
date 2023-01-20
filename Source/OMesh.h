//
// Created by dallin on 1/7/23.
//

#ifndef TESSELLATION_OMESH_H
#define TESSELLATION_OMESH_H

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <utility>

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
typedef OpenMesh::Decimater::DecimaterT<MyMesh>          Decimator;
typedef OpenMesh::Decimater::ModQuadricT<MyMesh>::Handle HModQuadric;
typedef std::shared_ptr<MyMesh> MyMeshPtr;

class OMesh {
public:
    explicit OMesh(const std::string& modelFilePath) {
        loadSuccess = this->LoadMesh(modelFilePath);
    }

    explicit OMesh(MyMesh mesh): mesh(std::move(mesh)) {}

    bool LoadMesh(const std::string& filepath);

    [[nodiscard]] bool IsLoaded() const {
        return loadSuccess;
    }

    MyMeshPtr Decimated(double factor);

    [[nodiscard]] const MyMesh &GetMesh() const;

private:
    MyMesh mesh;
    bool loadSuccess = false;
};


#endif //TESSELLATION_OMESH_H
