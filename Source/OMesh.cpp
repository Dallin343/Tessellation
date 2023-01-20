//
// Created by dallin on 1/7/23.
//

#include "OMesh.h"

bool OMesh::LoadMesh(const std::string& filepath) {
    auto success = OpenMesh::IO::read_mesh(this->mesh, filepath);
    if (!success) {
        std::cerr << "Mesh read error" << std::endl;
    }

    return success;
}

MyMeshPtr OMesh::Decimated(double factor) {
    auto decimatedMesh = std::make_shared<MyMesh>(mesh);
    Decimator decimator(*decimatedMesh);
    HModQuadric hModQuadric;

    decimator.add(hModQuadric);

    decimator.module(hModQuadric).unset_max_err();
    decimator.initialize();
    decimator.decimate_to(int(floor(mesh.n_vertices() * factor)));

    decimatedMesh->garbage_collection();
    return decimatedMesh;
}

const MyMesh &OMesh::GetMesh() const {
    return mesh;
}

