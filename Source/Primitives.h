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

struct VertexData {
    glm::vec3 Position;
    glm::vec3 Normal;
};

class Vertex;
class Edge;
class Face;
typedef typename std::shared_ptr<Edge> EPtr;
typedef typename std::shared_ptr<Face> FPtr;
typedef typename std::shared_ptr<Vertex> VPtr;

typedef typename std::vector<EPtr> EBuf;
typedef typename std::vector<FPtr> FBuf;
typedef typename std::vector<VPtr> VBuf;

class Edge {
public:
    Edge(VPtr& a, VPtr& b);
    const std::vector<FPtr>& GetFaces() const;
    void AddFace(FPtr& face);
    const std::array<VPtr, 2> &GetVertices() const;
    glm::dvec3 GetVector();


private:
    std::vector<FPtr> faces = {};
    std::array<VPtr, 2> vertices;
};

class Face {
public:
    Face(EPtr& e1, EPtr& e2, EPtr& e3);
    const std::array<EPtr, 3> &GetEdges() const;
    const glm::dvec4 & GetPlane() const;

private:
    std::array<EPtr, 3> edges;
    glm::dvec3 normal;
    glm::dvec4 plane;
};



class Vertex {
public:
    Vertex(): Position(), Normal() {}
    Vertex(glm::dvec3 position, glm::dvec3 normal): Position(position), Normal(normal) {}

    const std::vector<EPtr> &GetEdges() const;
    void AddEdge(EPtr& edge);
    VertexData GetVertexData();
    const glm::dvec3 &GetPosition() const;
    void SetPosition(const glm::vec3 &position);
    const glm::dvec3 &GetNormal() const;
    void SetNormal(const glm::vec3 &normal);
    unsigned int GetID() const;
    void SetID(unsigned int id);

private:
    std::vector<EPtr> edges = {};
    // position
    glm::dvec3 Position;
    // normal
    glm::dvec3 Normal;

    unsigned int ID;
};
#endif //TESSELLATION_PRIMITIVES_H
