#ifndef TESSELLATION_SIMPLIFY_H
#define TESSELLATION_SIMPLIFY_H
//
// Mesh Simplification Tutorial
//
// (C) by Sven Forstmann in 2014
//
// License : MIT
// http://opensource.org/licenses/MIT
//
//https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification
//
// 5/2016: Chris Rorden created minimal version for OSX/Linux/Windows compile

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <cfloat> //FLT_EPSILON, DBL_EPSILON
#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>
#include <unordered_set>

//using namespace glm;
glm::vec3 barycentric(const glm::vec3 &p, const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c){
    glm::vec3 v0 = b-a;
    glm::vec3 v1 = c-a;
    glm::vec3 v2 = p-a;
    double d00 = glm::dot(v0, v0);
    double d01 = glm::dot(v0, v1);
    double d11 = glm::dot(v1, v1);
    double d20 = glm::dot(v2, v0);
    double d21 = glm::dot(v2, v1);
    double denom = d00*d11-d01*d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return {u,v,w};
}

glm::vec3 interpolate(const glm::vec3 &p, const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, const glm::vec3 attrs[3])
{
    glm::vec3 bary = barycentric(p,a,b,c);
    glm::vec3 out = glm::vec3(0,0,0);
    out = out + attrs[0] * bary.x;
    out = out + attrs[1] * bary.y;
    out = out + attrs[2] * bary.z;
    return out;
}

double min(double v1, double v2) {
    return fmin(v1,v2);
}


class SymmetricMatrix {

public:

    // Constructor

    SymmetricMatrix(double c=0) {
        for (double &i : m) i = c;
    }

    SymmetricMatrix(	double m11, double m12, double m13, double m14,
                       double m22, double m23, double m24,
                       double m33, double m34,
                       double m44) {
        m[0] = m11;  m[1] = m12;  m[2] = m13;  m[3] = m14;
        m[4] = m22;  m[5] = m23;  m[6] = m24;
        m[7] = m33;  m[8] = m34;
        m[9] = m44;
    }

    // Make plane

    SymmetricMatrix(double a,double b,double c,double d)
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

    SymmetricMatrix operator+(const SymmetricMatrix& n) const
    {
        return SymmetricMatrix( m[0]+n[0],   m[1]+n[1],   m[2]+n[2],   m[3]+n[3],
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

    double m[10];
};
///////////////////////////////////////////

namespace MySimplify {
    double vertex_error(SymmetricMatrix q, glm::vec3 v) {
        auto x = v.x, y = v.y, z = v.z;
        return   q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x + q[4]*y*y
                 + 2*q[5]*y*z + 2*q[6]*y + q[7]*z*z + 2*q[8]*z + q[9];
    }
}

class Decimator {
public:

};

namespace Simplify
{
    // Global Variables & Strctures
    enum Attributes {
        NONE,
        NORMAL = 2,
        TEXCOORD = 4,
        COLOR = 8
    };
    struct Triangle {
        int v[3];
        double err[4];
        int deleted, dirty, attr;
        glm::vec3 n;
        glm::vec3 uvs[3];
        int material;
    };
    struct Vertex {
        glm::vec3 p;
        int tstart, tcount;
        SymmetricMatrix q;
        int border;
    };
    struct Ref {
        int tid, tvertex;
    };
    std::vector<Triangle> triangles;
    std::vector<Vertex> vertices;
    std::vector<Ref> refs;
    std::string mtllib;
    std::vector<std::string> materials;

    struct FeaturePoint {
        glm::vec3 p, n;
    };

    typedef std::shared_ptr<FeaturePoint> FeaturePointPtr;
    typedef std::unordered_map<int, std::unordered_set<FeaturePointPtr>> FeatureFaceMap;
    FeatureFaceMap featureFaceMap;

    // Helper functions

    double vertex_error(SymmetricMatrix q, double x, double y, double z);
    double calculate_error(int id_v1, int id_v2, glm::vec3 &p_result);
    bool flipped(glm::vec3 p,int i0,int i1,Vertex &v0,Vertex &v1,std::vector<int> &deleted);
    void update_uvs(int i0,const Vertex &v,const glm::vec3 &p,std::vector<int> &deleted);
    void update_triangles(int i0,Vertex &v,std::vector<int> &deleted,int &deleted_triangles);
    void update_mesh(int iteration);
    void compact_mesh();
    //
    // Main simplification function
    //
    // target_count  : target nr. of triangles
    // agressiveness : sharpness to increase the threshold.
    //                 5..8 are good numbers
    //                 more iterations yield higher quality
    //

    void simplify_mesh(int target_count, double agressiveness=7, bool verbose=false)
    {
        // init
        for (auto & triangle : triangles)
        {
            triangle.deleted=0;
        }

        for (int i = 0; i < triangles.size(); i++) {
            featureFaceMap.insert({i, {}});
        }

        // main iteration loop
        int deleted_triangles=0;
        std::vector<int> deleted0,deleted1;
        int triangle_count=triangles.size();
        //int iteration = 0;
        //loop(iteration,0,100)
        for (int iteration = 0; iteration < 100; iteration ++)
        {
            if(triangle_count-deleted_triangles<=target_count)break;

            // update mesh once in a while
            if(iteration%1==0)
            {
                update_mesh(iteration);
            }

            // clear dirty flag
            for (auto & triangle : triangles) triangle.dirty=0;

            //
            // All triangles with edges below the threshold will be removed
            //
            // The following numbers works well for most models.
            // If it does not, try to adjust the 3 parameters
            //
            double threshold = 0.000000001*pow(double(iteration+3),agressiveness);

            // target number of triangles reached ? Then break
            if ((verbose) && (iteration%5==0)) {
                printf("iteration %d - triangles %d threshold %g\n",iteration,triangle_count-deleted_triangles, threshold);
            }

            // remove vertices & mark deleted triangles
            for (auto & t : triangles)
            {
                if(t.err[3]>threshold) continue;
                if(t.deleted) continue;
                if(t.dirty) continue;

                for (int j = 0; j < 3; ++j) if(t.err[j]<threshold)
                    {

                        int i0=t.v[ j     ]; Vertex &v0 = vertices[i0];
                        int i1=t.v[(j+1)%3]; Vertex &v1 = vertices[i1];
                        // Border check
                        if(v0.border != v1.border)  continue;

                        // Compute vertex to collapse to
                        glm::vec3 p;
                        calculate_error(i0,i1,p);
                        deleted0.resize(v0.tcount); // normals temporarily
                        deleted1.resize(v1.tcount); // normals temporarily
                        // don't remove if flipped
                        if( flipped(p,i0,i1,v0,v1,deleted0) ) continue;

                        if( flipped(p,i1,i0,v1,v0,deleted1) ) continue;

                        if ( (t.attr & TEXCOORD) == TEXCOORD  )
                        {
                            update_uvs(i0,v0,p,deleted0);
                            update_uvs(i0,v1,p,deleted1);
                        }

                        // not flipped, so remove edge
                        v0.p=p;
                        v0.q=v1.q+v0.q;
                        int tstart=refs.size();

                        update_triangles(i0,v0,deleted0,deleted_triangles);
                        update_triangles(i0,v1,deleted1,deleted_triangles);

                        int tcount=refs.size()-tstart;

                        if(tcount<=v0.tcount)
                        {
                            // save ram
                            if(tcount)memcpy(&refs[v0.tstart],&refs[tstart],tcount*sizeof(Ref));
                        }
                        else
                            // append
                            v0.tstart=tstart;

                        v0.tcount=tcount;
                        break;
                    }
                // done?
                if(triangle_count-deleted_triangles<=target_count)break;
            }
        }
        // clean up mesh
        compact_mesh();
    } //simplify_mesh()

    void simplify_mesh_lossless(bool verbose=false)
    {
        // init
        for (auto & triangle : triangles) triangle.deleted=0;

        // main iteration loop
        int deleted_triangles=0;
        std::vector<int> deleted0,deleted1;
        int triangle_count=triangles.size();
        //int iteration = 0;
        //loop(iteration,0,100)
        for (int iteration = 0; iteration < 9999; iteration ++)
        {
            // update mesh constantly
            update_mesh(iteration);
            // clear dirty flag
            for (auto & triangle : triangles) triangle.dirty=0;
            //
            // All triangles with edges below the threshold will be removed
            //
            // The following numbers works well for most models.
            // If it does not, try to adjust the 3 parameters
            //
            double threshold = DBL_EPSILON; //1.0E-3 EPS;
            if (verbose) {
                printf("lossless iteration %d\n", iteration);
            }

            // remove vertices & mark deleted triangles
            for (auto & t : triangles)
            {
                if(t.err[3]>threshold) continue;
                if(t.deleted) continue;
                if(t.dirty) continue;

                for (int j = 0; j < 3; ++j) if(t.err[j]<threshold)
                    {
                        int i0=t.v[ j     ]; Vertex &v0 = vertices[i0];
                        int i1=t.v[(j+1)%3]; Vertex &v1 = vertices[i1];

                        // Border check
                        if(v0.border != v1.border)  continue;

                        // Compute vertex to collapse to
                        glm::vec3 p;
                        calculate_error(i0,i1,p);

                        deleted0.resize(v0.tcount); // normals temporarily
                        deleted1.resize(v1.tcount); // normals temporarily

                        // don't remove if flipped
                        if( flipped(p,i0,i1,v0,v1,deleted0) ) continue;
                        if( flipped(p,i1,i0,v1,v0,deleted1) ) continue;

                        if ( (t.attr & TEXCOORD) == TEXCOORD )
                        {
                            update_uvs(i0,v0,p,deleted0);
                            update_uvs(i0,v1,p,deleted1);
                        }

                        // not flipped, so remove edge
                        v0.p=p;
                        v0.q=v1.q+v0.q;
                        int tstart=refs.size();

                        update_triangles(i0,v0,deleted0,deleted_triangles);
                        update_triangles(i0,v1,deleted1,deleted_triangles);

                        int tcount=refs.size()-tstart;

                        if(tcount<=v0.tcount)
                        {
                            // save ram
                            if(tcount)memcpy(&refs[v0.tstart],&refs[tstart],tcount*sizeof(Ref));
                        }
                        else
                            // append
                            v0.tstart=tstart;

                        v0.tcount=tcount;
                        break;
                    }
            }
            if(deleted_triangles<=0)break;
            deleted_triangles=0;
        } //for each iteration
        // clean up mesh
        compact_mesh();
    } //simplify_mesh_lossless()


    // Check if a triangle flips when this edge is removed

    bool flipped(glm::vec3 p,int i0,int i1,Vertex &v0,Vertex &v1,std::vector<int> &deleted)
    {

        for (int k = 0; k < v0.tcount; ++k)
        {
            Triangle &t=triangles[refs[v0.tstart+k].tid];
            if(t.deleted)continue;

            int s=refs[v0.tstart+k].tvertex;
            int id1=t.v[(s+1)%3];
            int id2=t.v[(s+2)%3];

            if(id1==i1 || id2==i1) // delete ?
            {

                deleted[k]=1;
                continue;
            }
            glm::vec3 d1 = glm::normalize(vertices[id1].p-p);
            glm::vec3 d2 = glm::normalize(vertices[id2].p-p);
            if(fabs(glm::dot(d1, d2))>0.999) return true;
            glm::vec3 n = glm::normalize(glm::cross(d1, d2));
            deleted[k]=0;
            if(glm::dot(n, t.n)<0.2) return true;
        }
        return false;
    }

    // update_uvs

    void update_uvs(int i0,const Vertex &v,const glm::vec3 &p,std::vector<int> &deleted)
    {
        for (int k = 0; k < v.tcount; ++k)
        {
            Ref &r=refs[v.tstart+k];
            Triangle &t=triangles[r.tid];
            if(t.deleted)continue;
            if(deleted[k])continue;
            glm::vec3 p1=vertices[t.v[0]].p;
            glm::vec3 p2=vertices[t.v[1]].p;
            glm::vec3 p3=vertices[t.v[2]].p;
            t.uvs[r.tvertex] = interpolate(p,p1,p2,p3,t.uvs);
        }
    }

    // Update triangle connections and edge error after a edge is collapsed

    void update_triangles(int i0,Vertex &v,std::vector<int> &deleted,int &deleted_triangles)
    {
        glm::vec3 p;
        for (int k = 0; k < v.tcount; ++k)
        {
            Ref &r=refs[v.tstart+k];
            Triangle &t=triangles[r.tid];
            if(t.deleted)continue;
            if(deleted[k])
            {
                t.deleted=1;
                deleted_triangles++;
                continue;
            }
            t.v[r.tvertex]=i0;
            t.dirty=1;
            t.err[0]=calculate_error(t.v[0],t.v[1],p);
            t.err[1]=calculate_error(t.v[1],t.v[2],p);
            t.err[2]=calculate_error(t.v[2],t.v[0],p);
            t.err[3]=min(t.err[0],min(t.err[1],t.err[2]));
            refs.push_back(r);
        }
    }

    // compact triangles, compute edge error and build reference list

    void update_mesh(int iteration)
    {
        if(iteration>0) // compact triangles
        {
            int dst=0;
            for (int i = 0; i < triangles.size(); ++i)
                if(!triangles[i].deleted)
                {
                    triangles[dst++]=triangles[i];
                }
            triangles.resize(dst);
        }
        //

        // Init Reference ID list
        for (auto & vertex : vertices)
        {
            vertex.tstart=0;
            vertex.tcount=0;
        }
        for (auto & t : triangles)
        {
            for (int j : t.v) vertices[j].tcount++;
        }
        int tstart=0;
        for (auto & v : vertices)
        {
            v.tstart=tstart;
            tstart+=v.tcount;
            v.tcount=0;
        }

        // Write References
        refs.resize(triangles.size()*3);
        for (int i = 0; i < triangles.size(); ++i)
        {
            Triangle &t=triangles[i];
            for (int j = 0; j < 3; ++j)
            {
                Vertex &v=vertices[t.v[j]];
                refs[v.tstart+v.tcount].tid=i;
                refs[v.tstart+v.tcount].tvertex=j;
                v.tcount++;
            }
        }

        // Init Quadrics by Plane & Edge Errors
        //
        // required at the beginning ( iteration == 0 )
        // recomputing during the simplification is not required,
        // but mostly improves the result for closed meshes
        //
        if( iteration == 0 )
        {
            // Identify boundary : vertices[].border=0,1

            std::vector<int> vcount,vids;

            for (auto & vertex : vertices)
                vertex.border=0;

            for (int i = 0; i < vertices.size(); ++i)
            {
                Vertex &v=vertices[i];
                vcount.clear();
                vids.clear();
                for (int j = 0; j < v.tcount; ++j)
                {
                    int k=refs[v.tstart+j].tid;
                    Triangle &t=triangles[k];
                    for (int k : t.v)
                    {
                        int ofs=0,id=k;
                        while(ofs<vcount.size())
                        {
                            if(vids[ofs]==id)break;
                            ofs++;
                        }
                        if(ofs==vcount.size())
                        {
                            vcount.push_back(1);
                            vids.push_back(id);
                        }
                        else
                            vcount[ofs]++;
                    }
                }
                for (int j = 0; j < vcount.size(); ++j) if(vcount[j]==1)
                        vertices[vids[j]].border=1;
            }
            //initialize errors
            for (auto & vertex : vertices)
                vertex.q=SymmetricMatrix(0.0);

            for (auto & t : triangles)
            {
                glm::vec3 n,p[3];
                for (int j = 0; j < 3; ++j) p[j]=vertices[t.v[j]].p;
                n = glm::normalize(glm::cross(p[1]-p[0], p[2]-p[0]));
                t.n=n;
                for (int j : t.v) vertices[j].q = vertices[j].q+SymmetricMatrix(n.x,n.y,n.z,-glm::dot(n, p[0]));
            }
            for (auto & t : triangles)
            {
                // Calc Edge Error
                glm::vec3 p;
                for (int j = 0; j < 3; ++j) t.err[j]=calculate_error(t.v[j],t.v[(j+1)%3],p);
                t.err[3]=min(t.err[0],min(t.err[1],t.err[2]));
            }
        }
    }

    // Finally compact mesh before exiting

    void compact_mesh()
    {
        int dst=0;
        for (auto & vertex : vertices)
        {
            vertex.tcount=0;
        }
        for (int i = 0; i < triangles.size(); ++i)
            if(!triangles[i].deleted)
            {
                Triangle &t=triangles[i];
                triangles[dst++]=t;
                for (int j : t.v) vertices[j].tcount=1;
            }
        triangles.resize(dst);
        dst=0;
        for (int i = 0; i < vertices.size(); ++i)
            if(vertices[i].tcount)
            {
                vertices[i].tstart=dst;
                vertices[dst].p=vertices[i].p;
                dst++;
            }
        for (auto & t : triangles)
        {
            for (int & j : t.v) j=vertices[j].tstart;
        }
        vertices.resize(dst);
    }

    // Error between vertex and Quadric

    double vertex_error(SymmetricMatrix q, double x, double y, double z)
    {
        return   q[0]*x*x + 2*q[1]*x*y + 2*q[2]*x*z + 2*q[3]*x + q[4]*y*y
                 + 2*q[5]*y*z + 2*q[6]*y + q[7]*z*z + 2*q[8]*z + q[9];
    }

    // Error for one edge

    double calculate_error(int id_v1, int id_v2, glm::vec3 &p_result)
    {
        // compute interpolated vertex

        SymmetricMatrix q = vertices[id_v1].q + vertices[id_v2].q;
        bool   border = vertices[id_v1].border & vertices[id_v2].border;
        double error=0;
        double det = q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);
        if ( det != 0 && !border )
        {

            // q_delta is invertible
            p_result.x = -1/det*(q.det(1, 2, 3, 4, 5, 6, 5, 7 , 8));	// vx = A41/det(q_delta)
            p_result.y =  1/det*(q.det(0, 2, 3, 1, 5, 6, 2, 7 , 8));	// vy = A42/det(q_delta)
            p_result.z = -1/det*(q.det(0, 1, 3, 1, 4, 6, 2, 5,  8));	// vz = A43/det(q_delta)

            error = vertex_error(q, p_result.x, p_result.y, p_result.z);
        }
        else
        {
            // det = 0 -> try to find best result
            glm::vec3 p1=vertices[id_v1].p;
            glm::vec3 p2=vertices[id_v2].p;
            glm::vec3 p3=(p1+p2)/2.0f;
            double error1 = vertex_error(q, p1.x,p1.y,p1.z);
            double error2 = vertex_error(q, p2.x,p2.y,p2.z);
            double error3 = vertex_error(q, p3.x,p3.y,p3.z);
            error = min(error1, min(error2, error3));
            if (error1 == error) p_result=p1;
            if (error2 == error) p_result=p2;
            if (error3 == error) p_result=p3;
        }
        return error;
    }

    char *trimwhitespace(char *str)
    {
        char *end;

        // Trim leading space
        while(isspace((unsigned char)*str)) str++;

        if(*str == 0)  // All spaces?
            return str;

        // Trim trailing space
        end = str + strlen(str) - 1;
        while(end > str && isspace((unsigned char)*end)) end--;

        // Write new null terminator
        *(end+1) = 0;

        return str;
    }

    //Option : Load OBJ
    void load_obj(const char* filename, bool process_uv=false){
        vertices.clear();
        triangles.clear();
        //printf ( "Loading Objects %s ... \n",filename);
        FILE* fn;
        if(filename==nullptr)		return ;
        if((char)filename[0]==0)	return ;
        if ((fn = fopen(filename, "rb")) == nullptr)
        {
            printf ( "File %s not found!\n" ,filename );
            return;
        }
        char line[1000];
        memset ( line,0,1000 );
        int vertex_cnt = 0;
        int material = -1;
        std::map<std::string, int> material_map;
        std::vector<glm::vec3> uvs;
        std::vector<std::vector<int> > uvMap;

        while(fgets( line, 1000, fn ) != nullptr)
        {
            Vertex v;
            glm::vec3 uv;

            if (strncmp(line, "mtllib", 6) == 0)
            {
                mtllib = trimwhitespace(&line[7]);
            }
            if (strncmp(line, "usemtl", 6) == 0)
            {
                std::string usemtl = trimwhitespace(&line[7]);
                if (material_map.find(usemtl) == material_map.end())
                {
                    material_map[usemtl] = materials.size();
                    materials.push_back(usemtl);
                }
                material = material_map[usemtl];
            }

            if ( line[0] == 'v' && line[1] == 't' )
            {
                if ( line[2] == ' ' )
                    if(sscanf(line,"vt %lf %lf",
                              &uv.x,&uv.y)==2)
                    {
                        uv.z = 0;
                        uvs.push_back(uv);
                    } else
                    if(sscanf(line,"vt %lf %lf %lf",
                              &uv.x,&uv.y,&uv.z)==3)
                    {
                        uvs.push_back(uv);
                    }
            }
            else if ( line[0] == 'v' )
            {
                if ( line[1] == ' ' )
                    if(sscanf(line,"v %lf %lf %lf",
                              &v.p.x,	&v.p.y,	&v.p.z)==3)
                    {
                        vertices.push_back(v);
                    }
            }
            int integers[9];
            if ( line[0] == 'f' )
            {
                Triangle t;
                bool tri_ok = false;
                bool has_uv = false;

                if(sscanf(line,"f %d %d %d",
                          &integers[0],&integers[1],&integers[2])==3)
                {
                    tri_ok = true;
                }else
                if(sscanf(line,"f %d// %d// %d//",
                          &integers[0],&integers[1],&integers[2])==3)
                {
                    tri_ok = true;
                }else
                if(sscanf(line,"f %d//%d %d//%d %d//%d",
                          &integers[0],&integers[3],
                          &integers[1],&integers[4],
                          &integers[2],&integers[5])==6)
                {
                    tri_ok = true;
                }else
                if(sscanf(line,"f %d/%d/%d %d/%d/%d %d/%d/%d",
                          &integers[0],&integers[6],&integers[3],
                          &integers[1],&integers[7],&integers[4],
                          &integers[2],&integers[8],&integers[5])==9)
                {
                    tri_ok = true;
                    has_uv = true;
                }else // Add Support for v/vt only meshes
                if (sscanf(line, "f %d/%d %d/%d %d/%d",
                           &integers[0], &integers[6],
                           &integers[1], &integers[7],
                           &integers[2], &integers[8]) == 6)
                {
                    tri_ok = true;
                    has_uv = true;
                }
                else
                {
                    printf("unrecognized sequence\n");
                    printf("%s\n",line);
                    while(1);
                }
                if ( tri_ok )
                {
                    t.v[0] = integers[0]-1-vertex_cnt;
                    t.v[1] = integers[1]-1-vertex_cnt;
                    t.v[2] = integers[2]-1-vertex_cnt;
                    t.attr = 0;

                    if ( process_uv && has_uv )
                    {
                        std::vector<int> indices;
                        indices.push_back(integers[6]-1-vertex_cnt);
                        indices.push_back(integers[7]-1-vertex_cnt);
                        indices.push_back(integers[8]-1-vertex_cnt);
                        uvMap.push_back(indices);
                        t.attr |= TEXCOORD;
                    }

                    t.material = material;
                    //geo.triangles.push_back ( tri );
                    triangles.push_back(t);
                    //state_before = state;
                    //state ='f';
                }
            }
        }

        if ( process_uv && uvs.size() )
        {
            for (int i = 0; i < triangles.size(); ++i)
            {
                for (int j = 0; j < 3; ++j)
                    triangles[i].uvs[j] = uvs[uvMap[i][j]];
            }
        }

        fclose(fn);

        //printf("load_obj: vertices = %lu, triangles = %lu, uvs = %lu\n", vertices.size(), triangles.size(), uvs.size() );
    } // load_obj()

    // Optional : Store as OBJ

    void write_obj(const char* filename)
    {
        FILE *file=fopen(filename, "w");
        int cur_material = -1;
        bool has_uv = (triangles.size() && (triangles[0].attr & TEXCOORD) == TEXCOORD);

        if (!file)
        {
            printf("write_obj: can't write data file \"%s\".\n", filename);
            exit(0);
        }
        if (!mtllib.empty())
        {
            fprintf(file, "mtllib %s\n", mtllib.c_str());
        }
        for (int i = 0; i < vertices.size(); ++i)
        {
            //fprintf(file, "v %lf %lf %lf\n", vertices[i].p.x,vertices[i].p.y,vertices[i].p.z);
            fprintf(file, "v %g %g %g\n", vertices[i].p.x,vertices[i].p.y,vertices[i].p.z); //more compact: remove trailing zeros
        }
        if (has_uv)
        {
            for (int i = 0; i < triangles.size(); ++i) if(!triangles[i].deleted)
                {
                    fprintf(file, "vt %g %g\n", triangles[i].uvs[0].x, triangles[i].uvs[0].y);
                    fprintf(file, "vt %g %g\n", triangles[i].uvs[1].x, triangles[i].uvs[1].y);
                    fprintf(file, "vt %g %g\n", triangles[i].uvs[2].x, triangles[i].uvs[2].y);
                }
        }
        int uv = 1;
        for (int i = 0; i < triangles.size(); ++i) if(!triangles[i].deleted)
            {
                if (triangles[i].material != cur_material)
                {
                    cur_material = triangles[i].material;
                    fprintf(file, "usemtl %s\n", materials[triangles[i].material].c_str());
                }
                if (has_uv)
                {
                    fprintf(file, "f %d/%d %d/%d %d/%d\n", triangles[i].v[0]+1, uv, triangles[i].v[1]+1, uv+1, triangles[i].v[2]+1, uv+2);
                    uv += 3;
                }
                else
                {
                    fprintf(file, "f %d %d %d\n", triangles[i].v[0]+1, triangles[i].v[1]+1, triangles[i].v[2]+1);
                }
                //fprintf(file, "f %d// %d// %d//\n", triangles[i].v[0]+1, triangles[i].v[1]+1, triangles[i].v[2]+1); //more compact: remove trailing zeros
            }
        fclose(file);
    }
};

#endif //TESSELLATION_SIMPLIFY_H
