//
// Created by dallin on 2/28/23.
//

#include "IO.h"

void IO::toOBJ(const SurfaceMesh &sm, std::ostream &out)  {
    std::unordered_map<SM_vertex_descriptor, unsigned int> VDtoVIdx;
    std::unordered_map<SM_halfedge_descriptor, unsigned int> HDtoVtIdx;
    std::vector<Point_3> v;
    std::vector<Point_2> vt;
    std::vector<Vector> vn;
    std::vector<std::string> f;

    v.reserve(sm.number_of_vertices());
    vt.reserve(sm.number_of_vertices());
    vn.reserve(sm.number_of_vertices());

    auto uvmap = sm.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
    auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;

    //move vertices pos to v
    for (auto vert : sm.vertices()) {
        VDtoVIdx.insert({vert, v.size()});
        v.push_back(sm.point(vert));

        std::unordered_map<Point_2, unsigned int> uvToIdx;
        for (const auto& hd : halfedges_around_source(vert, sm)) {
            auto uv = get(uvmap, hd);
            if (uvToIdx.find(uv) != uvToIdx.end()) {
                //already processed this uv
                auto idx = uvToIdx.at(uv);
                HDtoVtIdx.insert({hd, idx});
            } else {
                //newly processed uv
                auto idx = vt.size();
                HDtoVtIdx.insert({hd, idx});
                uvToIdx.insert({uv, idx});
                vt.push_back(uv);
            }
        }
    }

    for (auto face : sm.faces()) {
        auto nrmIdx = vn.size();
        vn.push_back(get(fNorms, face));

        std::stringstream ss;
        for (auto hd : sm.halfedges_around_face(sm.halfedge(face))) {
            ss << " " << VDtoVIdx.at(sm.source(hd)) + 1 << "/" << HDtoVtIdx.at(hd) + 1 << "/" << nrmIdx + 1;
        }

        f.push_back(ss.str());
    }

    //Write to file
    for (auto pos : v) {
        out << std::setprecision(std::numeric_limits<double>::max_digits10) <<
            "v " << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
    }

    for (auto texCoord : vt) {
        out << std::setprecision(std::numeric_limits<double>::max_digits10) <<
            "vt " << texCoord.x() << " " << texCoord.y() << "\n";
    }

//    out << "vn 0.0 0.0 0.0\n";
    for (auto nrm : vn) {
        out << "vn " << nrm.x() << " " << nrm.y() << " " << nrm.z() << "\n";
    }

    for (const auto& fStr : f) {
        out << "f" << fStr << "\n";
    }
    out.flush();
}
