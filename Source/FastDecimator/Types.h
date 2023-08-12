//
// Created by dallin on 6/27/23.
//

#ifndef TESSELLATION_TYPES_H
#define TESSELLATION_TYPES_H

#include <array>
#include <glm/glm.hpp>
#include <memory>
#include <utility>

namespace FastDecimator {
    class SymmetricMatrix {
    public:
        // Constructor

        SymmetricMatrix(double c = 0.0) {
            for (auto& item : m) {
                item = c;
            }
        }

        SymmetricMatrix(double m11, double m12, double m13, double m14,
                        double m22, double m23, double m24,
                        double m33, double m34,
                        double m44) {
            m[0] = m11;  m[1] = m12;  m[2] = m13;  m[3] = m14;
            m[4] = m22;  m[5] = m23;  m[6] = m24;
            m[7] = m33;  m[8] = m34;
            m[9] = m44;
        }

        // Make plane

        SymmetricMatrix(double a, double b, double c, double d)
        {
            m[0] = a*a;  m[1] = a*b;  m[2] = a*c;  m[3] = a*d;
            m[4] = b*b;  m[5] = b*c;  m[6] = b*d;
            m[7 ] =c*c; m[8 ] = c*d;
            m[9 ] = d*d;
        }

        double operator[](int c) const { return m[c]; }

        // Determinant

        double det(	int a11, int a12, int a13,
                       int a21, int a22, int a23,
                       int a31, int a32, int a33)
        {
            double det =  m[a11]*m[a22]*m[a33] + m[a13]*m[a21]*m[a32] + m[a12]*m[a23]*m[a31]
                          - m[a13]*m[a22]*m[a31] - m[a11]*m[a23]*m[a32]- m[a12]*m[a21]*m[a33];
            return det;
        }

        const SymmetricMatrix operator+(const SymmetricMatrix& n) const
        {
            return SymmetricMatrix(m[0] + n[0], m[1] + n[1], m[2] + n[2], m[3] + n[3],
                                   m[4]+n[4],   m[5]+n[5],   m[6]+n[6],
                                   m[ 7]+n[ 7], m[ 8]+n[8 ],
                                   m[ 9]+n[9 ]);
        }

        SymmetricMatrix& operator+=(const SymmetricMatrix& n)
        {
            m[0]+=n[0];   m[1]+=n[1];   m[2]+=n[2];   m[3]+=n[3];
            m[4]+=n[4];   m[5]+=n[5];   m[6]+=n[6];   m[7]+=n[7];
            m[8]+=n[8];   m[9]+=n[9];
            return *this;
        }

        std::array<double, 10> m;
    };

    // Forward Declaration
    class Vertex;
    class Edge;
    class Face;
    typedef std::shared_ptr<Vertex> VPtr;
    typedef std::shared_ptr<Edge> EPtr;
    typedef std::shared_ptr<Face> FPtr;

    class Vertex {
    public:
        explicit Vertex(glm::dvec3 pos): pos(pos), Q() {}

        Vertex(glm::dvec3 pos, SymmetricMatrix Q): pos(pos), Q(Q) {}

    public:
        glm::dvec3 pos;
        SymmetricMatrix Q;
        std::vector<FPtr> faces {};
        std::vector<EPtr> edges {};
    };


    class Edge {
    public:
        Edge() = default;

        Edge(VPtr vertex0, VPtr vertex1) : vertex0(std::move(vertex0)), vertex1(std::move(vertex1)) {}

    public:
        VPtr vertex0, vertex1;
        double error = 0.0;
        std::vector<FPtr> faces {};
    };

    class Face {
    public:
        Face() = default;

        explicit Face(const std::array<VPtr, 3> &vertices) : vertices(vertices) {}
        Face(VPtr v0, VPtr v1, VPtr v2): vertices({std::move(v0), std::move(v1), std::move(v2)}) {}

        Face(const std::array<VPtr, 3> &vertices, double error, const glm::dvec3 &normal)
                : vertices(vertices), error(error), normal(normal) {}

    public:
        std::array<VPtr, 3> vertices;
        std::array<EPtr, 3> edges;
        double error = 0.0;
        glm::dvec3 normal = {};
    };
}
#endif //TESSELLATION_TYPES_H
