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
}

Mesh::~Mesh()
{
//    for (HE *he : _halfedges){
//        delete he;
//    }
//    for (Vertex *v : _HEverts){
//        delete v;
//    }
//    for (Edge *e : _edges){
//        delete e;
//    }
//    for (Face *f: _HEfaces){
//        delete f;
//    }
    for (auto it = _halfedges.begin(); it != _halfedges.end(); ++it){
        HE *f = it->second;
        delete f;
    }
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it){
        Vertex *f = it->second;
        delete f;
    }
    for (auto it = _edges.begin(); it != _edges.end(); ++it){
        Edge *f = it->second;
        delete f;
    }
    for (auto it = _HEfaces.begin(); it != _HEfaces.end(); ++it){
        Face *f = it->second;
        delete f;
    }
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
        HE *he1 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()}; //twin, next, vertex, edge, face
        HE *he2 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()};
        HE *he3 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()};
        _halfedges[he1->randid] = he1;
        _halfedges[he2->randid] = he2;
        _halfedges[he2->randid] = he2;
//        _halfedges.emplace_back(he1);
//        _halfedges.emplace_back(he2);
//        _halfedges.emplace_back(he3);

//        Face *f = new Face{he1, face, normal};
//        Face *f = new Face{he1, normal};
        Face *f = new Face{he1, random_string()};
        he1->face = f;
        he2->face = f;
        he3->face = f;
//        _HEfaces.emplace(_HEfaces.end(), f);
        _HEfaces[f->randid] = f;

        he1->next = he2;
        he2->next = he3;
        he3->next = he1;

        if (_vertidx.find(iv1) == _vertidx.end()){
            Vertex *v1_he = new Vertex{he1, v1, 1, random_string()};
            he1->vertex = v1_he;
//            _HEverts.emplace_back(v1_he);
            _HEverts[v1_he->randid] = v1_he;
            _vertidx[iv1] = {1, v1_he};

        } else {
            _vertidx[iv1].degree++;
            _vertidx[iv1].vert->degree = _vertidx[iv1].degree;
            he1->vertex = _vertidx[iv1].vert;
        }

        if (_idkmap.find(std::pair<int,int>(iv1, iv2)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv2, iv1)) == _idkmap.end()){
//            Edge *e1 = new Edge{he1, _vertidx[iv1].vert, _vertidx[iv2].vert};
            Edge *e1 = new Edge{he1, random_string()};
            he1->edge = e1;
//            _edges.emplace_back(e1);
            _edges[e1->randid] = e1;
            _idkmap[std::pair<int,int>(iv1, iv2)] = he1;
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv2, iv1)]->edge;
            he1->edge = blah;
            _idkmap[std::pair<int,int>(iv1, iv2)] = he1;
        }

        if (_vertidx.find(iv2) == _vertidx.end()){
            Vertex *v2_he = new Vertex{he2, v2, 1, random_string()};
            he2->vertex = v2_he;
//            _HEverts.emplace_back(v2_he);
            _HEverts[v2_he->randid] = v2_he;
            _vertidx[iv2] = {1, v2_he};
        } else {
            _vertidx[iv2].degree++;
            _vertidx[iv2].vert->degree = _vertidx[iv2].degree;
            he2->vertex = _vertidx[iv2].vert;
        }

        if (_idkmap.find(std::pair<int,int>(iv2, iv3)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv3, iv2)) == _idkmap.end()){
//            Edge *e2 = new Edge{he2, _vertidx[iv2].vert, _vertidx[iv3].vert};
            Edge *e2 = new Edge{he2, random_string()};
            he2->edge = e2;
//            _edges.emplace_back(e2);
            _edges[e2->randid] = e2;
            _idkmap[std::pair<int,int>(iv2, iv3)] = he2;
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv3, iv2)]->edge;
            he2->edge = blah;
            _idkmap[std::pair<int,int>(iv2, iv3)] = he2;
        }

        if (_vertidx.find(iv3) == _vertidx.end()){
            Vertex *v3_he = new Vertex{he3, v3, 1, random_string()};
            he3->vertex = v3_he;
//            _HEverts.emplace_back(v3_he);
            _HEverts[v3_he->randid] = v3_he;
            _vertidx[iv3] = {1, v3_he};
        } else {
            _vertidx[iv3].degree++;
            _vertidx[iv3].vert->degree = _vertidx[iv3].degree;
            he3->vertex = _vertidx[iv3].vert;
        }

        if (_idkmap.find(std::pair<int,int>(iv3, iv1)) == _idkmap.end() && _idkmap.find(std::pair<int,int>(iv1, iv3)) == _idkmap.end()){
//            Edge *e3 = new Edge{he3, _vertidx[iv3].vert, _vertidx[iv1].vert};
            Edge *e3 = new Edge{he3, random_string()};
            he3->edge = e3;
//            _edges.emplace_back(e3);
            _edges[e3->randid] = e3;
            _idkmap[std::pair<int,int>(iv3, iv1)] = he3;
        } else {
            Edge *blah = _idkmap[std::pair<int,int>(iv1, iv3)]->edge;
            he3->edge = blah;
            _idkmap[std::pair<int,int>(iv3, iv1)] = he3;
        }

    }

    for (auto pair = _idkmap.begin(); pair != _idkmap.end(); ++pair){
        if (pair->second->twin == NULL){
            pair->second->twin = _idkmap[std::pair<int,int>(pair->first.second, pair->first.first)];
        }
    }
}

int Mesh::getNumNeighbors(Vertex *v){
    HE *currhe = v->halfedge;
    int count = 0;
    do {
        currhe = currhe->twin->next;
        count++;
    } while (currhe != v->halfedge);
    return count;
}

void Mesh::flip(HE *halfedge){
    HE *AB = halfedge; // 1   //twin, next, vertex, edge, face
    HE *BA = halfedge->twin; // 2

    HE *BC = halfedge->next; // 1
    HE *CA = BC->next; // 1

    HE *AD = BA->next; // 2
    HE *DB = AD->next; // 2

    Face *ABC = halfedge->face; // 1 // halfedge, verts, normal
    Face *ADB = BA->face; // 2

    Vertex *A = halfedge->vertex;
    Vertex *B = BC->vertex;
    Vertex *C = CA->vertex; // halfedge, degree
    Vertex *D = DB->vertex;

    if (!(getNumNeighbors(A) == 3 || getNumNeighbors(B) == 3)){
        A->halfedge = AD;
        B->halfedge = BC;
        A->degree -= 1;
        B->degree -= 1;
        C->degree += 1;
        D->degree += 1;

        AB->next = DB; //twin, next, vertex, edge, face
        AB->vertex = C;
        BA->next = CA;
        BA->vertex = D;

        BC->next = AB;
        CA->next = AD;
        CA->face = ADB;

        AD->next = BA;
        DB->next = BC;
        DB->face = ABC;

        ABC->halfedge = BC;
        ADB->halfedge = AD;
    }
//    Edge *eddy = halfedge->edge; //halfedge, verts, normal
//    eddy->vert1 = C;
//    eddy->vert2 = D;
}

void Mesh::split(HE *halfedge, std::vector<Edge*> &newedges, const std::unordered_map<std::string, Vertex*> &oldverts){
    HE *AB = halfedge; // 1   //twin, next, vertex, edge, face
    HE *BA = halfedge->twin; // 2 //EA

    HE *BC = halfedge->next; // 1
    HE *CA = BC->next; // 1

    HE *AD = BA->next; // 2
    HE *DB = AD->next; // 2

    Edge *eddy = halfedge->edge; //halfedge

    Face *one = halfedge->face; // 1 // halfedge, normal
    Face *two = BA->face; // 2

    Vertex *A = halfedge->vertex;
    Vertex *B = BC->vertex;
    Vertex *C = CA->vertex; // halfedge, degree
    Vertex *D = DB->vertex;

    Vector3f position = (B->position*3.f/8.f) + (A->position*3.f/8.f) + (C->position*1.f/8.f) + (D->position*1.f/8.f); //(B->position + A->position)/2.f;
    Vertex *E = new Vertex{BA, position, 4, random_string()};
//    _HEverts.emplace_back(E);
    _HEverts[E->randid] = E;

    BA->vertex = E;

    C->degree += 1;
    D->degree += 1;

    //update _lastmap, _HEverts
    HE *EC = new HE{NULL, CA, E, NULL, NULL, random_string()};
    HE *CE = new HE{EC, NULL, C, NULL, one, random_string()}; //next = EB
    HE *EB = new HE{NULL, BC, E, NULL, one, random_string()};
    HE *BE = new HE{EB, NULL, B, NULL, two, random_string()}; //next = ED
    HE *ED = new HE{NULL, DB, E, NULL, two, random_string()};
    HE *DE = new HE{ED, BA, D, NULL, NULL, random_string()};

    EC->twin = CE;
    EB->twin = BE;
    ED->twin = DE;
    CE->next = EB;
    BE->next = ED;
    AB->next = EC;
    BC->next = CE;
    DB->next = BE;
    AD->next = DE;

//    _halfedges.push_back(EC);
//    _halfedges.push_back(CE);
//    _halfedges.push_back(EB);
//    _halfedges.push_back(BE);
//    _halfedges.push_back(ED);
//    _halfedges.push_back(DE);
    _halfedges[EC->randid] = EC;
    _halfedges[CE->randid] = CE;
    _halfedges[EB->randid] = EB;
    _halfedges[BE->randid] = BE;
    _halfedges[ED->randid] = ED;
    _halfedges[DE->randid] = DE;

    Edge *edEC = new Edge{EC, random_string()};
    Edge *edEB = new Edge{EB, random_string()};
    Edge *edED = new Edge{ED, random_string()};

//    _edges.emplace_back(edEC);
//    _edges.emplace_back(edEB);
//    _edges.emplace_back(edED);
    _edges[edEC->randid] = edEC;
    _edges[edEB->randid] = edEB;
    _edges[edED->randid] = edED;

    if (oldverts.find(C->randid) != oldverts.end()){
        newedges.push_back(edEC);
    }
    if (oldverts.find(D->randid) != oldverts.end()){
        newedges.push_back(edED);
    }
//    newedges.push_back(edEB);
//    newedges.push_back(eddy);

    EC->edge = edEC;
    CE->edge = edEC;
    EB->edge = edEB;
    BE->edge = edEB;
    ED->edge = edED;
    DE->edge = edED;

    Face *three = new Face{CA, random_string()};
    Face *four = new Face{AD, random_string()};

    B->halfedge = BC;
    C->halfedge = CE;
    D->halfedge = DB;


    one->halfedge = BC;
    two->halfedge = DB;

    AB->face = three;
    BA->face = four;
    BC->face = one;
    CA->face = three;
    AD->face = four;
    DB->face = two;
    EC->face = three;
    DE->face = four;

//    _HEfaces.emplace_back(three);
//    _HEfaces.emplace_back(four);
    _HEfaces[three->randid] = three;
    _HEfaces[four->randid] = four;

//    std::cout << A->randid <<std::endl;
//    std::cout << B->randid <<std::endl;
//    std::cout << C->randid <<std::endl;
//    std::cout << D->randid <<std::endl;
//    std::cout << E->randid <<std::endl;
//    std::cout << "stop" << std::endl;
}

void Mesh::collapse(HE *halfedge){
    HE *BD = halfedge;
    HE *DA = BD->next;
    HE *AB = DA->next;
    HE *DB = halfedge->twin;
    HE *BC = DB->next;
    HE *CD = BC->next;

    HE *DC = CD->twin;
    HE *AD = DA->twin;

    //delete CD, DA, AB, BC

    Face *one = DB->face;
    Face *two = BD->face;

    //delete one, two

    Edge *edCD = CD->edge;
    Edge *edDA = DA->edge;
    Edge *edAB = AB->edge;
    Edge *edBC = BC->edge;
    Edge *edDB = halfedge->edge;

    //delete edCD, edDA, all?

    Vertex *D = DA->vertex;
    Vertex *B = BD->vertex;

    //delete D?

    B->position = (B->position + D->position)/2.f;
    D->position = B->position;

    //delete D, DC, DB, BD, CD, DA, AD, one, two, edCD, edDA, edDB
    delete _HEfaces[one->randid];
    _HEfaces.erase(one->randid);
    delete _HEfaces[two->randid];
    _HEfaces.erase(two->randid);

    delete _HEverts[D->randid];
    _HEverts.erase(D->randid);

    delete _edges[edCD->randid];
    _edges.erase(edCD->randid);
    delete _edges[edDA->randid];
    _edges.erase(edDA->randid);
    delete _edges[edDB->randid];
    _edges.erase(edDB->randid);

    delete _halfedges[DC->randid];
    _halfedges.erase(DC->randid);
    delete _halfedges[DB->randid];
    _halfedges.erase(DB->randid);
    delete _halfedges[BD->randid];
    _halfedges.erase(BD->randid);
    delete _halfedges[CD->randid];
    _halfedges.erase(CD->randid);
    delete _halfedges[DA->randid];
    _halfedges.erase(DA->randid);
    delete _halfedges[AD->randid];
    _halfedges.erase(AD->randid);
}

void Mesh::subdivide(){
    std::vector<Edge*> newedges;
    std::vector<Edge*> edgecopy;
    std::unordered_map<std::string, Vertex*> oldverts;
    std::unordered_map<std::string, Vector3f> newpos;
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        Vertex *v = it->second;
        newpos[v->randid] = adjustPos(v);
        oldverts[v->randid] = v;
    }
//    for (Vertex *v : _HEverts){
//        newpos[v->randid] = adjustPos(v);
//        oldverts[v->randid] = v;
//    }
    for (auto it = _edges.begin(); it != _edges.end(); ++it ){
        Edge *e = it->second;
        edgecopy.push_back(e);
    }
//    for (Edge *e : _edges){
//        edgecopy.push_back(e);
//    }

    for(Edge *e : edgecopy){
        split(e->halfedge, newedges, oldverts);
    }
    for (Edge *e : newedges){
        flip(e->halfedge);
    }
    for (auto it = newpos.begin(); it != newpos.end(); ++it ){
        Vertex *curr = oldverts[it->first];
//        std::cout << getNumNeighbors(curr) << std::endl;
//        std::cout << "old: " << curr->position << std::endl;
        curr->position = it->second;
//        std::cout << "new: " << curr->position << std::endl;
//        count++;
    }
//    std::cout << count << std::endl;
    edgecopy.clear();
//    std::cout << " " << std::endl;
}

void Mesh::convertToOBJ(){
    _vertices.clear();
    _faces.clear();
//    int count = 0;
//    for (auto it = _HEfaces.begin(); it != _HEfaces.end(); ++it){
//        Edge *e = it->second;
//        edgecopy.push_back(e);
//    }
    for (auto it = _HEfaces.begin(); it != _HEfaces.end(); ++it){
//    for (Face *f : _HEfaces){
//        std::cout << _HEfaces.size() << std::endl;
//        count = 0;
//        count += 1;
        Face *f = it->second;
        std::vector<int> idx;
        idx.reserve(3);
        HE *currhe = f->halfedge;
//        std::cout << currhe->vertex->randid << std::endl;
        do {
            Vertex *v = currhe->vertex; //hash unique id for vertex to vertex's new index
            if (_lastmap.find(v->randid) == _lastmap.end()){
                Vector3f position = v->position;
                if (abs(position[0]) < 0.0001){
                    position[0] = 0;
                }
                if (abs(position[1]) < 0.0001){
                    position[1] = 0;
                }
                if (abs(position[2]) < 0.0001){
                    position[2] = 0;
                }
                _vertices.emplace_back(position);

                _lastmap[v->randid] = _vertices.size() - 1;
                idx.emplace_back(_lastmap[v->randid]);

            } else {
                idx.emplace_back(_lastmap[v->randid]);
            }
            currhe = currhe->next;
//            std::cout << currhe->vertex->randid  << std::endl;
//            if (count > 10){
//                break;
//            }
//            count += 1;
        }
        while (currhe != f->halfedge);//while(count <= 30);
//        std::cout<< idx[0] << " " << idx[1] << " " << idx[2] << std::endl;
        _faces.emplace_back(Vector3i(idx[0], idx[1], idx[2]));
//        break;
    }
}

Vector3f Mesh::adjustPos(Vertex *v){
    Vector3f adjust(0,0,0);
    int vd = v->degree;
    HE *currhe = v->halfedge;
    int count = 0;
    do {
        Vertex *currv = currhe->twin->vertex;
        if (vd == 3){
            adjust += 3.f/16.f * currv->position;
        } else {
            adjust += (3.f/(8.f*vd)) * currv->position;
        }
        currhe = currhe->twin->next;
        count++;
    } while (currhe != v->halfedge);
    float u;
    if (vd == 3){
        u = 3.f/16.f;
    } else {
        u = 3.f/(8.f*vd);
    }
    return adjust + ((1.f - (vd * u))*v->position);
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
    std::cout << "Wrote to file." << std::endl;
}
