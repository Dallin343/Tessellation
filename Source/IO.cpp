//
// Created by dallin on 2/28/23.
//

#include "IO.h"
#include "lodepng.h"

namespace IO {
    template<typename T, typename V>
    void writeMap(const std::unordered_map<T, V>& map, std::ostream& os) {
        for (auto const& [key, val] : map) {
            os << key << " " << val << "\n";
        }

        os.flush();
    }

    template<typename T, typename V>
    std::unordered_map<T, V> readMap(std::ifstream& in) {
        std::unordered_map<T, V> map;
        T key;
        V val;

        while (in >> key >> val) {
            map.insert({key, val});
        }

        return map;
    }

    void toOBJ(const SurfaceMesh &sm, std::ostream &out, const std::string& outVD, const std::string& outHD, const std::string& outSeams)  {
        std::unordered_map<SM_vertex_descriptor, unsigned int> VDtoVIdx;
        std::unordered_map<SM_halfedge_descriptor, unsigned int> HDtoVtIdx;
        VdHdMap vdPairsToHD;
        std::vector<std::pair<SM_vertex_descriptor, SM_vertex_descriptor>> eSeams;

        std::vector<Point_3> v;
        std::vector<Point_2> vt;
        std::vector<Vector> vn;
        std::vector<std::string> f;

        v.reserve(sm.number_of_vertices());
        vt.reserve(sm.number_of_vertices());
        vn.reserve(sm.number_of_vertices());

        auto uvmap = sm.property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;
        auto fNorms = sm.property_map<SM_face_descriptor, Vector>("f:normal").first;
        auto vSeamMap = sm.property_map<SM_vertex_descriptor, bool>("v:on_seam").first;
        auto eSeamMap = sm.property_map<SM_edge_descriptor, bool>("e:on_seam").first;

        //move vertices pos to v
        std::vector<std::tuple<SM_vertex_descriptor, SM_vertex_descriptor, SM_halfedge_descriptor>> vdPairsToHDVec;
        for (auto vert : sm.vertices()) {

            VDtoVIdx.insert({vert, v.size()});
            v.push_back(sm.point(vert));

            std::unordered_map<Point_2, unsigned int> uvToIdx;
            for (const auto& hd : halfedges_around_source(vert, sm)) {
                vdPairsToHDVec.emplace_back(vert, sm.target(hd), hd);

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

        for (auto edge : sm.edges()) {
            if (get(eSeamMap, edge)) {
                auto src = sm.source(sm.halfedge(edge));
                auto tgt = sm.target(sm.halfedge(edge));
                eSeams.emplace_back(src, tgt);
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
            auto x = texCoord.x();
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

        if (!outVD.empty()) {
            VdMap writeVDToIdx;
            for (const auto& [vd, vIdx] : VDtoVIdx) {
                writeVDToIdx.insert({vd.idx(), vIdx});
            }
            std::ofstream of(outVD);
            writeMap(writeVDToIdx, of);
        }

        if (!outHD.empty()) {
            for (const auto& [src, target, hd] : vdPairsToHDVec) {
                auto newSrc = VDtoVIdx.at(src);
                auto newTarget = VDtoVIdx.at(target);
                vdPairsToHD.insert({key(newSrc, newTarget), hd.idx()});
            }
            std::ofstream of(outHD);
            writeMap(vdPairsToHD, of);
        }

        if (!outSeams.empty()) {
            std::ofstream of(outSeams);
            for (const auto& [src, tgt] : eSeams) {
                auto newSrc = VDtoVIdx.at(src);
                auto newTarget = VDtoVIdx.at(tgt);
                of << newSrc << " " << newTarget << "\n";
            }
            of.flush();
        }
    }

    std::tuple<SurfaceMeshPtr, VdMap, VdHdMap> fromOBJ(const std::string& filename, const std::string& vdMapFilename, const std::string& hdMapFilename, const std::string& seamsFilename) {
        SurfaceMeshPtr surfaceMesh = LoadMesh(filename);
        ReadUV(surfaceMesh, filename);

        auto fNorms = surfaceMesh->add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;
        auto vNorms = surfaceMesh->add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
        CGAL::Polygon_mesh_processing::compute_normals(*surfaceMesh, vNorms, fNorms);

        std::ifstream vdIn(vdMapFilename);
        VdMap vdMap = readMap<unsigned int, unsigned int>(vdIn);

        std::ifstream hdIn(hdMapFilename);
        VdHdMap vdHdMap = readMap<unsigned long, unsigned int>(hdIn);

        if (!seamsFilename.empty()) {
            auto vSeamMap = surfaceMesh->add_property_map<SM_vertex_descriptor, bool>("v:on_seam", false).first;
            auto eSeamMap = surfaceMesh->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
            std::ifstream seamsIn(seamsFilename);
            unsigned int src, tgt;

            while (seamsIn >> src >> tgt) {
                auto sVD = SM_vertex_descriptor(src), tVD = SM_vertex_descriptor(tgt);
                put(vSeamMap, sVD, true);
                put(vSeamMap, tVD, true);

                auto hd = surfaceMesh->halfedge(sVD, tVD);
                if (!hd) {
                    std::cout << "Halfedge does not exist.\n";
                }
                else {
                    auto e = surfaceMesh->edge(hd);
                    put(eSeamMap, e, true);
                }
            }
        }

        return std::make_tuple(surfaceMesh, vdMap, vdHdMap);
    }

    std::pair<SurfaceMeshPtr, SeamMeshPtr> fromOBJ(const std::string& filename, const std::string& seamFilename) {
        SurfaceMeshPtr surfaceMesh = LoadMesh(filename);
        ReadUV(surfaceMesh, filename);
        SeamMeshPtr seamMesh = AddSeams(surfaceMesh, seamFilename);
        auto fNorms = surfaceMesh->add_property_map<SM_face_descriptor, Vector>("f:normal", {0.0, 0.0, 0.0}).first;
        auto vNorms = surfaceMesh->add_property_map<SM_vertex_descriptor, Vector>("v:normal", {0.0, 0.0, 0.0}).first;
        CGAL::Polygon_mesh_processing::compute_normals(*surfaceMesh, vNorms, fNorms);

        return std::make_pair(surfaceMesh, seamMesh);
    }

    SurfaceMeshPtr LoadMesh(const std::string& filename) {
        auto mesh = std::make_shared<SurfaceMesh>();
        if(!CGAL::IO::read_OBJ(filename, *mesh))
        {
            std::cerr << "Invalid input file." << "\n";
        }

        return mesh;
    }

    void ReadUV(const SurfaceMeshPtr& mesh, const std::string& filename) {
        std::ifstream infile(filename);

        std::string line;
        std::vector<Point_2> texCoords;
        texCoords.reserve(mesh->num_vertices());

        UV_pmap uvmap = mesh->add_property_map<SM_halfedge_descriptor, Point_2>("h:uv").first;

        // Get texcoords from file.
        // There MAY be more texcoords than vertices
        while (std::getline(infile, line)) {
            if (line.substr(0, 2) == "vt") {
                std::istringstream iss(line.substr(3, line.length()-3));
                double u, v;
                iss >> u >> v;
                texCoords.emplace_back(u, v);

            } else if (line.substr(0, 1) == "f") {
                line = line.substr(2, line.length() - 2);
                std::string s0, s1, s2;
                unsigned int w0, w1;
                //Split 1/1/1 2/2/2 3/3/3 into separate strings like "1/1/1"
                w0 = line.find(' ');
                w1 = line.find(' ', w0+1);
                s0 = line.substr(0, w0);
                s1 = line.substr(w0+1, w1 - w0-1);
                s2 = line.substr(w1+1, line.length() - w1-1);

                SM_vertex_descriptor v0, v1, v2;
                Point_2 vt0, vt1, vt2;
                unsigned int v, vt;
                w0 = s0.find('/');
                w1 = s0.find('/', w0+1);
                v = std::stoul(s0.substr(0, w0)) - 1;
                v0 = SM_vertex_descriptor(v);
                vt = std::stoul(s0.substr(w0+1, w1 - w0-1)) - 1;
                vt0 = texCoords.at(vt);

                w0 = s1.find('/');
                w1 = s1.find('/', w0+1);
                v = std::stoul(s1.substr(0, w0)) - 1;
                v1 = SM_vertex_descriptor(v);
                vt = std::stoul(s1.substr(w0+1, w1 - w0-1)) - 1;
                vt1 = texCoords.at(vt);

                w0 = s2.find('/');
                w1 = s2.find('/', w0+1);
                v = std::stoul(s2.substr(0, w0)) - 1;
                v2 = SM_vertex_descriptor(v);
                vt = std::stoul(s2.substr(w0+1, w1 - w0-1)) - 1;
                vt2 = texCoords.at(vt);

                put(uvmap, mesh->halfedge(v0, v1), vt0);
//            put(uvmap, mesh->halfedge(v0, v2), vt0);
//            put(uvmap, mesh->halfedge(v1, v0), vt1);
                put(uvmap, mesh->halfedge(v1, v2), vt1);
                put(uvmap, mesh->halfedge(v2, v0), vt2);
//            put(uvmap, mesh->halfedge(v2, v1), vt2);
            }
        }

        auto highResUVMap = mesh->add_property_map<SM_halfedge_descriptor, Point_2>("bh:uv").first;
        std::copy(uvmap.begin(), uvmap.end(), highResUVMap.begin());
    }

    SeamMeshPtr AddSeams(const SurfaceMeshPtr& sm, const std::string& selectionsFile) {
        Seam_edge_pmap seam_edge_pm = sm->add_property_map<SM_edge_descriptor, bool>("e:on_seam", false).first;
        Seam_vertex_pmap seam_vertex_pm = sm->add_property_map<SM_vertex_descriptor, bool>("v:on_seam",false).first;
        // The seam mesh
        auto mesh = std::make_shared<SeamMesh>(*sm, seam_edge_pm, seam_vertex_pm);

        if (selectionsFile.empty()) {
            return mesh;
        }
        // Add the seams to the seam mesh
        SM_halfedge_descriptor bhd = mesh->add_seams(selectionsFile.c_str());

        if (bhd == SurfaceMesh::null_halfedge()) {
            std::cerr << "There was an error while adding seams." << std::endl;
        }

        return mesh;
    }

    void WriteTexture(const std::vector<glm::vec3>& tex, unsigned int w, unsigned int h, float offsetVal, unsigned int max, const std::string& file) {
        std::cout << "Writing texture...\n";
        std::ofstream out(file);
        const float maxOffset = (float)max + offsetVal;
        out << "P6\n" << w << "\n" << h << "\n255\n";// << max << "\n";
//        if (max < 256) {
//            std::cout << "Using single byte storage. (max = " << max << ")\n";
//        }
//        else {
//            std::cout << "Using double byte storage. (max = " << max << ")\n";
//        }
        for (const auto& pixel : tex) {
            auto r = pixel.r, g = pixel.g, b = pixel.b;

//            auto r2 = floor(255.0f * (r / maxOffset));
//            auto g2 = floor(255.0f * (g / maxOffset));
//            auto b2 = floor(255.0f * (b / maxOffset));
//
//            if (r2 > 72.0f || g2 > 72.0f || b2 > 72.0f ) {
//                int x = 1;
//            }

            out << (unsigned char)floor(r);
            out << (unsigned char)floor(g);
            out << (unsigned char)floor(b);

//            if (max < 256) {
//                //Only one byte stored.
//                out << (unsigned char)floor(pixel.r);
//                out << (unsigned char)floor(pixel.g);
//                out << (unsigned char)floor(pixel.b);
//            }
//            else {
//                //Two bytes stored
//                out << (uint16_t) floor(pixel.r);
//                out << (uint16_t) floor(pixel.g);
//                out << (uint16_t) floor(pixel.b);
//            }
        }
        out.flush();
    }
}

