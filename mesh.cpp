#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3i> &faces)
{
    _vertices = vertices;
    _faces = faces;
}

void Mesh::loadFromFile(const std::string &filePath)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }

    if(!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return;
    }

//    std::vector<Vector3f> vertices;
//    std::vector<Vector3f> normals;
//    std::vector<Vector3f> colors;
//    std::vector<Vector2f> uvs;
//    std::vector<int> materialIds;
//    std::vector<Vector3i> faces;

    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
//                tinyobj::real_t nx;
//                tinyobj::real_t ny;
//                tinyobj::real_t nz;
//                tinyobj::real_t tx;
//                tinyobj::real_t ty;

//                if(idx.normal_index != -1) {
//                    nx = attrib.normals[3*idx.normal_index+0];
//                    ny = attrib.normals[3*idx.normal_index+1];
//                    nz = attrib.normals[3*idx.normal_index+2];
//                } else {
//                    nx = 0;
//                    ny = 0;
//                    nz = 0;
//                }
//                if(idx.texcoord_index != -1) {
//                    tx = attrib.texcoords[2*idx.texcoord_index+0];
//                    ty = attrib.texcoords[2*idx.texcoord_index+1];
//                } else {
//                    tx = 0;
//                    ty = 0;
//                }

//                tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
//                tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
//                tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

                face[v] = _vertices.size();
                _vertices.push_back(Vector3f(vx, vy, vz));
//                normals.push_back(Vector3f(nx, ny, nz).normalized());
//                uvs.push_back(Vector2f(tx, ty));
//                colors.push_back(Vector3f(red, green, blue));
            }
            _faces.push_back(face);
//            materialIds.push_back(shapes[s].mesh.material_ids[f]);

            index_offset += fv;
        }
    }
    std::cout << "Loaded " << _faces.size() << " faces" << std::endl;
}

void Mesh::saveToFile(const std::string &filePath)
{
    std::ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << std::endl;
    }

    outfile.close();
}
