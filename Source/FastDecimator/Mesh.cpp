//
// Created by dallin on 6/27/23.
//

#include <fstream>
#include <sstream>
#include <unordered_map>
#include "Mesh.h"


FastDecimator::Mesh::Mesh(const std::string &filename) {
    loadOBJ(filename);
}

inline unsigned long key(unsigned int i, unsigned int j) {
    if (i < j)
        return ((unsigned long) i) << 32 | j;
    else
        return ((unsigned long) j) << 32 | i;
}

FastDecimator::EPtr FastDecimator::Mesh::get_edge(std::unordered_map<unsigned long, EPtr> &pte, unsigned long k,
                                                  const FastDecimator::VPtr& v0, const FastDecimator::VPtr& v1) {
    if (pte.find(k) == pte.end()) {
        auto e = std::make_shared<Edge>(v0, v1);
        pte.insert({k, e});
        return e;
    }

    return pte.at(k);
}

void FastDecimator::Mesh::loadOBJ(const std::string &filename) {
    vertices.clear();
    edges.clear();
    faces.clear();

    std::unordered_map<unsigned long, EPtr> pairToEdge;

    std::ifstream infile(filename);
    std::string line;

    while (std::getline(infile, line)) {
        if (line.substr(0, 2) == "v ") {
            std::istringstream iss(line.substr(2, line.length()-2));
            double x, y, z;
            iss >> x >> y >> z;
            vertices.push_back(std::make_shared<Vertex>(glm::dvec3(x,y,z)));
        }
        else if (line.substr(0, 1) == "f") {
            line = line.substr(2, line.length() - 2);
            std::string s0, s1, s2;
            unsigned int w0, w1;
            //Split 1/1/1 2/2/2 3/3/3 into separate strings like "1/1/1"
            w0 = line.find(' ');
            w1 = line.find(' ', w0+1);
            s0 = line.substr(0, w0);
            s1 = line.substr(w0+1, w1 - w0-1);
            s2 = line.substr(w1+1, line.length() - w1-1);

            VPtr v0, v1, v2;
            unsigned int i0, i1, i2;
            unsigned int v;
            w0 = s0.find('/');
            w1 = s0.find('/', w0+1);
            i0 = std::stoi(s0.substr(0, w0)) - 1;
            v0 = vertices.at(i0);

            w0 = s1.find('/');
            w1 = s1.find('/', w0+1);
            i1 = std::stoi(s0.substr(0, w0)) - 1;
            v1 = vertices.at(i1);

            w0 = s2.find('/');
            w1 = s2.find('/', w0+1);
            i2 = std::stoi(s0.substr(0, w0)) - 1;
            v2 = vertices.at(i2);

            //Create edges
            auto e0 = get_edge(pairToEdge, key(i0, i1), v0, v1);
            auto e1 = get_edge(pairToEdge, key(i1, i2), v1, v2);
            auto e2 = get_edge(pairToEdge, key(i2, i0), v2, v0);

            //Create face and add edges to it
            auto face = std::make_shared<Face>(v0, v1, v2);
            face->edges = {e0, e1, e2};
            faces.push_back(face);

            //Add Face to edges and verts
            e0->faces.push_back(face);
            e1->faces.push_back(face);
            e2->faces.push_back(face);

            v0->edges.push_back(e0);
            v0->edges.push_back(e2);
            v1->edges.push_back(e0);
            v1->edges.push_back(e1);
            v2->edges.push_back(e1);
            v2->edges.push_back(e2);

            edges.push_back(e0);
            edges.push_back(e1);
            edges.push_back(e2);

            //Add faces to vertices
            v0->faces.push_back(face);
            v1->faces.push_back(face);
            v2->faces.push_back(face);
        }
    }
}

void FastDecimator::Mesh::decimate(double ratio, int max_face_feats) {
    this->init();
}

void FastDecimator::Mesh::init() {

    //Calculate Vertex error
    for (auto& face : faces) {
        //Init errors
        std::vector<glm::dvec3> vPos;
        for (const auto& v : face->vertices) {
            vPos.push_back(v->pos);
        }
        auto n = glm::normalize(glm::cross(vPos[1] - vPos[0], vPos[2] - vPos[0]));
        face->normal = n;

        for (auto& v : face->vertices) {
            v->Q = {n.x, n.y, n.z, -glm::dot(n, vPos[0])};
        }
    }

    //Calculate edge error
    for (auto& face : faces) {
        glm::dvec3 p;
        double minError = 1000000000.0;
        for (auto& edge : face->edges) {
            edge->error = calculate_error(edge, p);
            if (edge->error < minError)
                minError = edge->error;
        }
        face->error = minError;
    }
}

double FastDecimator::Mesh::calculate_error(const FastDecimator::EPtr &edge, glm::dvec3 &optimalPoint) {
    auto v1 = edge->vertex0, v2 = edge->vertex1;
    SymmetricMatrix q = v1->Q + v2->Q;

    double error = 0.0;
    double det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);
    if ( det != 0 )
    {

        // q_delta is invertible
        optimalPoint.x = -1/det*(q.det(1, 2, 3, 4, 5, 6, 5, 7 , 8));	// vx = A41/det(q_delta)
        optimalPoint.y =  1/det*(q.det(0, 2, 3, 1, 5, 6, 2, 7 , 8));	// vy = A42/det(q_delta)
        optimalPoint.z = -1/det*(q.det(0, 1, 3, 1, 4, 6, 2, 5,  8));	// vz = A43/det(q_delta)

        error = vertex_error(q, optimalPoint);
    }
    else
    {
        // det = 0 -> try to find best result
        auto p1 = v1->pos;
        auto p2 = v2->pos;
        glm::dvec3 p3 = (p1+p2) / 2.0;

        double error1 = vertex_error(q, p1);
        double error2 = vertex_error(q, p2);
        double error3 = vertex_error(q, p3);
        error = std::min(error1, std::min(error2, error3));
        if (error1 == error) optimalPoint=p1;
        if (error2 == error) optimalPoint=p2;
        if (error3 == error) optimalPoint=p3;
    }
    return error;
}

double FastDecimator::Mesh::vertex_error(const FastDecimator::SymmetricMatrix &q, glm::dvec3 &p) {
    auto x = p.x, y = p.y, z = p.z;

    return   q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x + q[4]*y*y
             + 2*q[5]*y*z + 2*q[6]*y + q[7]*z*z + 2*q[8]*z + q[9];
}

