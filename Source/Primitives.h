//
// Created by dallin on 6/20/22.
//

#ifndef TESSELLATION_PRIMITIVES_H
#define TESSELLATION_PRIMITIVES_H

#include <utility>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <vector>
#include <memory>
#include <cstdarg>
#include <glm/mat4x4.hpp>
#include <cfloat>

struct VertexData {
    glm::vec3 Position;
    glm::vec3 Normal;
};

class Vertex;
class Edge;
class Face;
class Pair;

typedef typename std::shared_ptr<Edge> EdgePtr;
typedef typename std::shared_ptr<Face> FacePtr;
typedef typename std::shared_ptr<Vertex> VertexPtr;
typedef typename std::shared_ptr<Pair> PairPtr;

typedef typename std::vector<EdgePtr> EBuf;
typedef typename std::vector<FacePtr> FBuf;
typedef typename std::vector<VertexPtr> VBuf;

class Pair {
public:
    Pair(const VertexPtr &a, const VertexPtr &b);
    Pair(VertexPtr &a, VertexPtr &b, glm::dvec3 candidate);
    double GetCost() const;
    void SetCost(double cost);

    const glm::dvec3 &GetCandidate() const;

    void SetCandidate(const glm::dvec3 &candidate);

    const std::array<VertexPtr, 2> &GetVertices() const;

private:
    double cost = DBL_MAX;
    std::array<VertexPtr, 2> vertices;
    glm::dvec3 candidate = {};
};

class Edge {
public:
    Edge(VertexPtr& a, VertexPtr& b);
    const std::vector<FacePtr>& GetFaces() const;
    void AddFace(FacePtr& face);
    const std::array<VertexPtr, 2> &GetVertices() const;
    glm::dvec3 GetVector();


private:
    std::vector<FacePtr> faces = {};
    std::array<VertexPtr, 2> vertices;
};

class Face {
public:
    Face(EdgePtr& e1, EdgePtr& e2, EdgePtr& e3);
    const std::array<EdgePtr, 3> &GetEdges() const;
    const glm::dvec4 & GetPlane() const;

private:
    std::array<EdgePtr, 3> edges;
    glm::dvec3 normal;
    glm::dvec4 plane;
};



class Vertex {
public:
    Vertex(): Position(), Normal() {}
    Vertex(glm::dvec3 position, glm::dvec3 normal): Position(position), Normal(normal) {}

    const std::vector<EdgePtr> &GetEdges() const;
    void AddEdge(EdgePtr& edge);

    const std::vector<FacePtr>& GetFaces() const;
    void AddFace(FacePtr& face);

    VertexData GetVertexData();
    const glm::dvec3 &GetPosition() const;
    void SetPosition(const glm::vec3 &position);
    const glm::dvec3 &GetNormal() const;
    void SetNormal(const glm::vec3 &normal);
    unsigned int GetID() const;
    void SetID(unsigned int id);

    glm::dmat4 Q = {{0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 0.0}};
private:
    std::vector<EdgePtr> edges = {};
    std::vector<FacePtr> faces = {};

    // position
    glm::dvec3 Position;
    // normal
    glm::dvec3 Normal;

    unsigned int ID;
};
#endif //TESSELLATION_PRIMITIVES_H
