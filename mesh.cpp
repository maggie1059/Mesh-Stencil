#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>
#include <unordered_map>
#include <Eigen/Dense>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3i> &faces)
{
    _vertices = vertices;
    _faces = faces;
    _idkmap;
    _lastmap;
//    _vertmap;
    _vertidx;
    _fun.reserve(2);
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

    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

		face[v] = idx.vertex_index;

            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for(size_t i = 0; i < attrib.vertices.size(); i += 3) {
	_vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }
    std::cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << std::endl;
}

void Mesh::convertToHE(){
    for (Vector3i face : _faces){
        int iv1 = face[0];
        int iv2 = face[1];
        int iv3 = face[2];
        Vector3f v1 = _vertices[iv1];
        Vector3f v2 = _vertices[iv2];
        Vector3f v3 = _vertices[iv3];

        //take cross product, get normal, figure out orientation and swap verts if needed
        Vector3f AB = v2-v1;
        Vector3f AC = v3-v1;
        Vector3f normal = AB.cross(AC);
        normal.normalize();
        Vector3f centroid = (v1+v2+v3)/3.f;
        centroid.normalize();

        if (normal.dot(centroid) < 0){
            std::swap(v2, v3);
            std::swap(iv2, iv3);
            normal = -normal;
        }
//        HE he1{NULL, NULL, NULL, NULL, NULL}; //twin, next, vertex, edge, face
//        HE he2{NULL, NULL, NULL, NULL, NULL};
//        HE he3{NULL, NULL, NULL, NULL, NULL};
//        _halfedges.emplace_back(&he1);
//        _halfedges.emplace_back(&he2);
//        _halfedges.emplace_back(&he3);
//        _HEfaces.emplace(_HEfaces.end(), &f);

        _halfedges.emplace(_halfedges.end(), HE{NULL, NULL, NULL, NULL, NULL});
        _halfedges.emplace(_halfedges.end(), HE{NULL, NULL, NULL, NULL, NULL});
        _halfedges.emplace(_halfedges.end(), HE{NULL, NULL, NULL, NULL, NULL});
        _HEfaces.emplace(_HEfaces.end(), Face{&_halfedges[_halfedges.size()-3], face, normal};);

//        Face f{&he1, face, normal};
//        he1.face = &f;
//        he2.face = &f;
//        he3.face = &f;

        _halfedges[_halfedges.size()-3].face = &_HEfaces[_HEfaces.size()-1];
        _halfedges[_halfedges.size()-2].face = &_HEfaces[_HEfaces.size()-1];
        _halfedges[_halfedges.size()-1].face = &_HEfaces[_HEfaces.size()-1];

//        he1.next = &he2;
//        he2.next = &he3;
//        he3.next = &he1;

        _halfedges[_halfedges.size()-3].next = &_HEfaces[_HEfaces.size()-2];
        _halfedges[_halfedges.size()-2].next = &_HEfaces[_HEfaces.size()-1];
        _halfedges[_halfedges.size()-1].next = &_HEfaces[_HEfaces.size()-3];

        if (_idkmap.find(std::pair<int,int>(iv1, iv2)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv2, iv1)) == _idkmap.end()){
//            Edge e1{&he1, v1, v2};
//            he1.edge = &e1;
//            _edges.emplace_back(&e1);
//            _idkmap[std::pair<int,int>(iv1, iv2)] = &he1;

            _edges.emplace_back(Edge{&_halfedges[_halfedges.size()-3], v1, v2});
            _halfedges[_halfedges.size()-3].edge = &_edges[_edges.size()-1];
            _idkmap[std::pair<int,int>(iv1, iv2)] = &_halfedges[_halfedges.size()-3];
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv2, iv1)]->edge;
            he1.edge = blah;
            _idkmap[std::pair<int,int>(iv1, iv2)] = &_halfedges[_halfedges.size()-3];
        }

        if (_vertidx.find(iv1) == _vertidx.end()){
            Vertex v1_he{&he1, v1, 1, random_string()};
            he1.vertex = &v1_he;
            _HEverts.emplace_back(&v1_he);
            _vertidx[iv1] = {1, &v1_he};
        } else {
            _vertidx[iv1].degree++;
            _vertidx[iv1].vert->degree = _vertidx[iv1].degree;
            he1.vertex = _vertidx[iv1].vert;
        }

        if (_idkmap.find(std::pair<int,int>(iv2, iv3)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv3, iv2)) == _idkmap.end()){
            Edge e2{&he2, v2, v3};
            he2.edge = &e2;
            _edges.emplace_back(&e2);
            _idkmap[std::pair<int,int>(iv2, iv3)] = &he2;
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv3, iv2)]->edge;
            he2.edge = blah;
            _idkmap[std::pair<int,int>(iv2, iv3)] = &he2;//blah;
        }

        if (_vertidx.find(iv2) == _vertidx.end()){
            Vertex v2_he{&he2, v2, 1, random_string()};
            he2.vertex = &v2_he;
            _HEverts.emplace_back(&v2_he);
            _vertidx[iv2] = {1, &v2_he};
        } else {
            _vertidx[iv2].degree++;
            _vertidx[iv2].vert->degree = _vertidx[iv2].degree;
            he2.vertex = _vertidx[iv2].vert;
        }

        if (_idkmap.find(std::pair<int,int>(iv3, iv1)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv1, iv3)) == _idkmap.end()){
            Edge e3{&he3, v3, v1};
            he3.edge = &e3;
            _edges.emplace_back(&e3);
            _idkmap[std::pair<int,int>(iv3, iv1)] = &he3;
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv1, iv3)]->edge;
            he3.edge = blah;
            _idkmap[std::pair<int,int>(iv3, iv1)] = &he3;//blah;
        }

        if (_vertidx.find(iv3) == _vertidx.end()){
            Vertex v3_he{&he3, v3, 1, random_string()};
            he3.vertex = &v3_he;
            _HEverts.emplace_back(&v3_he);
            _vertidx[iv3] = {1, &v3_he};
        } else {
            _vertidx[iv3].degree++;
            _vertidx[iv3].vert->degree = _vertidx[iv3].degree;
            he3.vertex = _vertidx[iv3].vert;
        }
    }

    for (auto pair = _idkmap.begin(); pair != _idkmap.end(); ++pair){
        if (pair->second->twin == NULL){
            pair->second->twin = _idkmap[std::pair<int,int>(pair->first.second, pair->first.first)];
        }
    }

//    for(HE *h: _halfedges){
//        std::cout<< h->vertex->degree << std::endl;
//    }

//    for (Edge *e : _edges){
//        HE *curr = e->halfedge;
//        if (curr->twin == NULL){
//            Vertex *from = _vertmap[e->vert2].vert;
//            curr->twin = from->halfedge;
//        }
//    }
}

//void Mesh::flip(HE *halfedge){
//    Vertex *v1 = halfedge->next->next->vertex;
//    Vertex *v2 = halfedge->twin->next->next->vertex;

//}

//void Mesh::split(HE *halfedge){

//}

//void Mesh::collapse(HE *halfedge){

//}

void Mesh::convertToOBJ(){
    _vertices.clear();
    _faces.clear();
    std::vector<int> idx;
    idx.reserve(3);
    for (Face *f : _HEfaces){

        HE *currhe = f->halfedge;
        do {
            Vertex *v = currhe->vertex; //hash unique id for vertex to vertex's new index
//            std::cout << currhe->next << std::endl;
            if (_lastmap.find(v->randid) == _lastmap.end()){

                _vertices.emplace_back(v->position);

                _lastmap[v->randid] = _vertices.size() - 1;
                idx.emplace_back(_lastmap[v->randid]);

            } else {
                idx.emplace_back(_lastmap[v->randid]);
            }
            currhe = currhe->next;
        }
        while (currhe != f->halfedge);
        Vector3i curr(idx[0], idx[1], idx[2]);
        _faces.emplace_back(curr);
    }
}

std::string Mesh::random_string(){
    auto randchar = []() -> char
    {
        const char charset[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(12,0);
    std::generate_n( str.begin(), 12, randchar );
    return str;
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


void Mesh::fun(){
//    Fun fun1{4, 2};
//    _fun.push_back();
//    int hi = 4;
//    Fun whoo{&hi, NULL};
//    Sad meh{&whoo};
    _fun.emplace(_fun.end(), Fun{4, NULL});
//    _fun[0] = new Fun{4,2};
    std::cout << "hi" << std::endl;
//    _fun[1] = Fun{3,1};
    _sad.emplace(_sad.end(), Sad{&_fun[_fun.size() -1]});

}

void Mesh::fun2(){
    std::cout << "hi2" << std::endl;
    std::cout<< _fun[0].hi << std::endl;
}
