//
// Created by dallin on 7/2/22.
//

#include <glm/geometric.hpp>
#include "Primitives.h"

Edge::Edge(VertexPtr& a, VertexPtr& b) {
vertices = {a, b};
}

const std::vector<FacePtr>& Edge::GetFaces() const {
    return this->faces;
}

void Edge::AddFace(FacePtr& face) {
    this->faces.push_back(face);
}

const std::array<VertexPtr, 2> &Edge::GetVertices() const {
    return vertices;
}

glm::dvec3 Edge::GetVector() {
    return vertices[1]->GetPosition() - vertices[0]->GetPosition();
}

Face::Face(EdgePtr& e1, EdgePtr& e2, EdgePtr& e3) {
    this->edges = {e1, e2, e3};

    // Calculate face normal
    auto vertex = edges[0]->GetVertices()[0];
    glm::dvec3 faceNorm = glm::normalize(glm::cross(edges[0]->GetVector(), edges[1]->GetVector()));
    double orientation = glm::dot(faceNorm, glm::normalize(vertex->GetNormal()));

    this->normal = (orientation < 0.0f) ? -faceNorm : faceNorm;
    this->plane = glm::dvec4(this->normal, glm::dot(this->normal, vertex->GetPosition()));
}

const std::array<EdgePtr, 3> &Face::GetEdges() const {
    return edges;
}

const glm::dvec4& Face::GetPlane() const {
    return plane;
}

const std::vector<EdgePtr>& Vertex::GetEdges() const {
    return edges;
}

void Vertex::AddEdge(EdgePtr& edge) {
    this->edges.push_back(edge);
}

VertexData Vertex::GetVertexData() {
    return {Position, Normal};
}

const glm::dvec3& Vertex::GetPosition() const {
    return Position;
}

void Vertex::SetPosition(const glm::vec3 &position) {
    Position = position;
}

const glm::dvec3 &Vertex::GetNormal() const {
    return Normal;
}

void Vertex::SetNormal(const glm::vec3 &normal) {
    Normal = normal;
}

unsigned int Vertex::GetID() const {
    return ID;
}

void Vertex::SetID(unsigned int id) {
    ID = id;
}

const std::vector<FacePtr> &Vertex::GetFaces() const {
    return faces;
}

void Vertex::AddFace(FacePtr &face) {
    this->faces.push_back(face);
}

double Pair::GetCost() const {
    return cost;
}

void Pair::SetCost(double cost) {
    this->cost = cost;
}

Pair::Pair(VertexPtr &a, VertexPtr &b, glm::dvec3 candidate) {
    vertices = {a,b};
    this->candidate = candidate;
}

Pair::Pair(const VertexPtr &a, const VertexPtr &b) {
    vertices = {a,b};
}

const std::array<VertexPtr, 2> &Pair::GetVertices() const {
    return vertices;
}

const glm::dvec3 &Pair::GetCandidate() const {
    return candidate;
}

void Pair::SetCandidate(const glm::dvec3 &candidate) {
    this->candidate = candidate;
}
