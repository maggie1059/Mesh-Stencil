#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>
#include <unordered_map>
#include <Eigen/Dense>
#include <unordered_set>
#include <set>
#include <queue>

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

        //reminder: this is zero-indexed now
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

        std::shared_ptr<HE> he1(new HE{NULL, NULL, NULL, NULL, NULL, random_string()});
        std::shared_ptr<HE> he2(new HE{NULL, NULL, NULL, NULL, NULL, random_string()});
        std::shared_ptr<HE> he3(new HE{NULL, NULL, NULL, NULL, NULL, random_string()});

        _halfedges[he1->randid] = he1;
        _halfedges[he2->randid] = he2;
        _halfedges[he3->randid] = he3;

        std::shared_ptr<Face> f(new Face{he1, random_string(), Matrix4f::Zero()});
        he1->face = f;
        he2->face = f;
        he3->face = f;
        _HEfaces[f->randid] = f;

        he1->next = he2;
        he2->next = he3;
        he3->next = he1;

        if (_vertidx.find(iv1) == _vertidx.end()){
            std::shared_ptr<Vertex> v1_he(new Vertex{he1, v1, 1, random_string(), Matrix4f::Zero()});
            he1->vertex = v1_he;
            _HEverts[v1_he->randid] = v1_he;
            _vertidx[iv1] = {1, v1_he};
        } else {
            _vertidx[iv1].degree++;
            _vertidx[iv1].vert->degree = _vertidx[iv1].degree;
            he1->vertex = _vertidx[iv1].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv1, iv2)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv2, iv1)) == _edgepairs.end()){
            std::shared_ptr<Edge> e1(new Edge{he1, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});
            he1->edge = e1;
            _edges[e1->randid] = e1;
            _edgepairs[std::pair<int,int>(iv1, iv2)] = he1;
        } else {
            std::shared_ptr<Edge> e = _edgepairs[std::pair<int,int>(iv2, iv1)]->edge;
            he1->edge = e;
            _edgepairs[std::pair<int,int>(iv1, iv2)] = he1;
        }

        if (_vertidx.find(iv2) == _vertidx.end()){
            std::shared_ptr<Vertex> v2_he(new Vertex{he2, v2, 1, random_string(), Matrix4f::Zero()});
            he2->vertex = v2_he;
            _HEverts[v2_he->randid] = v2_he;
            _vertidx[iv2] = {1, v2_he};
        } else {
            _vertidx[iv2].degree++;
            _vertidx[iv2].vert->degree = _vertidx[iv2].degree;
            he2->vertex = _vertidx[iv2].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv2, iv3)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv3, iv2)) == _edgepairs.end()){
            std::shared_ptr<Edge> e2(new Edge{he2, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});
            he2->edge = e2;
            _edges[e2->randid] = e2;
            _edgepairs[std::pair<int,int>(iv2, iv3)] = he2;
        } else {
            std::shared_ptr<Edge> e = _edgepairs[std::pair<int,int>(iv3, iv2)]->edge;
            he2->edge = e;
            _edgepairs[std::pair<int,int>(iv2, iv3)] = he2;
        }

        if (_vertidx.find(iv3) == _vertidx.end()){
            std::shared_ptr<Vertex> v3_he(new Vertex{he3, v3, 1, random_string(), Matrix4f::Zero()});
            he3->vertex = v3_he;
            _HEverts[v3_he->randid] = v3_he;
            _vertidx[iv3] = {1, v3_he};
        } else {
            _vertidx[iv3].degree++;
            _vertidx[iv3].vert->degree = _vertidx[iv3].degree;
            he3->vertex = _vertidx[iv3].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv3, iv1)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv1, iv3)) == _edgepairs.end()){
            std::shared_ptr<Edge> e3(new Edge{he3, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});
            he3->edge = e3;
            _edges[e3->randid] = e3;
            _edgepairs[std::pair<int,int>(iv3, iv1)] = he3;
        } else {
            std::shared_ptr<Edge> e = _edgepairs[std::pair<int,int>(iv1, iv3)]->edge;
            he3->edge = e;
            _edgepairs[std::pair<int,int>(iv3, iv1)] = he3;
        }

    }

    for (auto pair = _edgepairs.begin(); pair != _edgepairs.end(); ++pair){
        if (pair->second->twin == NULL){
            pair->second->twin = _edgepairs[std::pair<int,int>(pair->first.second, pair->first.first)];
        }
    }
}

void Mesh::flip(std::shared_ptr<HE> halfedge){
    std::shared_ptr<HE> AB = halfedge; // 1
    std::shared_ptr<HE> BA = halfedge->twin; // 2

    std::shared_ptr<HE> BC = halfedge->next; // 1
    std::shared_ptr<HE> CA = BC->next; // 1

    std::shared_ptr<HE> AD = BA->next; // 2
    std::shared_ptr<HE> DB = AD->next; // 2

    std::shared_ptr<Face> ABC = halfedge->face; // 1
    std::shared_ptr<Face> ADB = BA->face; // 2

    std::shared_ptr<Vertex> A = halfedge->vertex;
    std::shared_ptr<Vertex> B = BC->vertex;
    std::shared_ptr<Vertex> C = CA->vertex;
    std::shared_ptr<Vertex> D = DB->vertex;

    if (!(getNumNeighbors(A) == 3 || getNumNeighbors(B) == 3)){
        A->halfedge = AD;
        B->halfedge = BC;
        A->degree -= 1;
        B->degree -= 1;
        C->degree += 1;
        D->degree += 1;

        AB->next = DB;
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
}

void Mesh::split(std::shared_ptr<HE> halfedge, std::vector<std::shared_ptr<Edge>> &newedges, const std::unordered_map<std::string, std::shared_ptr<Vertex>> &oldverts){
    std::shared_ptr<HE> AB = halfedge; // 1
    std::shared_ptr<HE> BA = halfedge->twin; // 2

    std::shared_ptr<HE> BC = halfedge->next; // 1
    std::shared_ptr<HE> CA = BC->next; // 1

    std::shared_ptr<HE> AD = BA->next; // 2
    std::shared_ptr<HE> DB = AD->next; // 2

    std::shared_ptr<Edge> eddy = halfedge->edge;

    std::shared_ptr<Face> one = halfedge->face; // 1
    std::shared_ptr<Face> two = BA->face; // 2

    std::shared_ptr<Vertex> A = halfedge->vertex;
    std::shared_ptr<Vertex> B = BC->vertex;
    std::shared_ptr<Vertex> C = CA->vertex;
    std::shared_ptr<Vertex> D = DB->vertex;

    Vector3f position = (B->position*3.f/8.f) + (A->position*3.f/8.f) + (C->position*1.f/8.f) + (D->position*1.f/8.f);
    std::shared_ptr<Vertex> E(new Vertex{BA, position, 4, random_string()});
    _HEverts[E->randid] = E;

    BA->vertex = E;

    C->degree += 1;
    D->degree += 1;

    //update _lastmap, _HEverts
    std::shared_ptr<HE> EC(new HE{NULL, CA, E, NULL, NULL, random_string()});
    std::shared_ptr<HE> CE(new HE{EC, NULL, C, NULL, one, random_string()});
    std::shared_ptr<HE> EB(new HE{NULL, BC, E, NULL, one, random_string()});
    std::shared_ptr<HE> BE(new HE{EB, NULL, B, NULL, two, random_string()});
    std::shared_ptr<HE> ED(new HE{NULL, DB, E, NULL, two, random_string()});
    std::shared_ptr<HE> DE(new HE{ED, BA, D, NULL, NULL, random_string()});

    EC->twin = CE;
    EB->twin = BE;
    ED->twin = DE;
    CE->next = EB;
    BE->next = ED;
    AB->next = EC;
    BC->next = CE;
    DB->next = BE;
    AD->next = DE;

    _halfedges[EC->randid] = EC;
    _halfedges[CE->randid] = CE;
    _halfedges[EB->randid] = EB;
    _halfedges[BE->randid] = BE;
    _halfedges[ED->randid] = ED;
    _halfedges[DE->randid] = DE;

    std::shared_ptr<Edge> edEC(new Edge{EC, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});
    std::shared_ptr<Edge> edEB(new Edge{EB, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});
    std::shared_ptr<Edge> edED(new Edge{ED, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0});

    _edges[edEC->randid] = edEC;
    _edges[edEB->randid] = edEB;
    _edges[edED->randid] = edED;

    if (oldverts.find(C->randid) != oldverts.end()){
        newedges.push_back(edEC);
    }
    if (oldverts.find(D->randid) != oldverts.end()){
        newedges.push_back(edED);
    }

    EC->edge = edEC;
    CE->edge = edEC;
    EB->edge = edEB;
    BE->edge = edEB;
    ED->edge = edED;
    DE->edge = edED;

    std::shared_ptr<Face> three(new Face{CA, random_string(), Matrix4f::Zero()});
    std::shared_ptr<Face> four(new Face{AD, random_string(), Matrix4f::Zero()});

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

    _HEfaces[three->randid] = three;
    _HEfaces[four->randid] = four;
}

void Mesh::collapse(std::shared_ptr<HE> halfedge, Vector3f cp, unordered_set<string> &skipEd){
    //inner
    std::shared_ptr<HE> BD = halfedge;
    std::shared_ptr<HE> DA = BD->next;
    std::shared_ptr<HE> AB = DA->next;
    std::shared_ptr<HE> DB = halfedge->twin;
    std::shared_ptr<HE> BC = DB->next;
    std::shared_ptr<HE> CD = BC->next;

    //outer
    std::shared_ptr<HE> CB = BC->twin;
    std::shared_ptr<HE> BA = AB->twin;

    std::shared_ptr<HE> DC = CD->twin;
    std::shared_ptr<HE> AD = DA->twin;

    //delete CD, DA, AB, BC

    std::shared_ptr<Face> one = DB->face;
    std::shared_ptr<Face> two = BD->face;

    //delete one, two

    std::shared_ptr<Edge> edCD = CD->edge;
    std::shared_ptr<Edge> edDA = DA->edge;
    std::shared_ptr<Edge> edAB = AB->edge;
    std::shared_ptr<Edge> edBC = BC->edge;
    std::shared_ptr<Edge> edDB = halfedge->edge;

    std::shared_ptr<Vertex> D = DA->vertex;
    std::shared_ptr<Vertex> B = BD->vertex;
    std::shared_ptr<Vertex> C = CD->vertex;
    std::shared_ptr<Vertex> A = AB->vertex;

    //check validity
    bool skip = false;
    std::unordered_set<string> neighborVerts = {};
    std::shared_ptr<HE> currhe = D->halfedge;
    do {
        neighborVerts.insert(currhe->twin->vertex->randid);
        currhe = currhe->twin->next;
    } while (currhe != D->halfedge);

    int count = 0;
    std::shared_ptr<HE> currhe2 = B->halfedge;
    do {
        if (neighborVerts.find(currhe2->twin->vertex->randid) != neighborVerts.end()){
            count++;
            if (getNumNeighbors(currhe2->twin->vertex) == 3){
                skip = true;
            }
        }
        currhe2 = currhe2->twin->next;
    } while (currhe2 != B->halfedge);
    if (count != 2){
        skip = true;
    }

    //check that normals won't flip for affected triangles
    if (!skip){
        std::shared_ptr<HE> currheb = B->halfedge;
        do {
            std::shared_ptr<Face> f = currheb->face;
            if (f==one || f==two){
                currheb = currheb->twin->next;
                continue;
            }
            std::shared_ptr<Vertex> v1 = currheb->vertex;
            std::shared_ptr<Vertex> v2 = currheb->next->vertex;
            std::shared_ptr<Vertex> v3 = currheb->next->next->vertex;
            Vector3f AB = v2->position - v1->position;
            Vector3f AC = v3->position - v1->position;
            Vector3f normal = AB.cross(AC);
            normal.normalize();

            Vector3f nAB = v2->position - cp;
            Vector3f nAC = v3->position - cp;
            Vector3f normal2 = nAB.cross(nAC);

            if (normal.dot(normal2) < 0.f){
                skip = true;
                break;
            }
            currheb = currheb->twin->next;
        } while (currheb != B->halfedge);
    }

    // check all neighbors only if skip isn't already true to cut down on runtime
    if (!skip){
        std::shared_ptr<HE> currhed = D->halfedge;
        do {
            std::shared_ptr<Face> f = currhed->face;
            if (f==one || f==two){
                currhed = currhed->twin->next;
                continue;
            }
            std::shared_ptr<Vertex> v1 = currhed->vertex;
            std::shared_ptr<Vertex> v2 = currhed->next->vertex;
            std::shared_ptr<Vertex> v3 = currhed->next->next->vertex;
            Vector3f AB = v2->position - v1->position;
            Vector3f AC = v3->position - v1->position;
            Vector3f normal = AB.cross(AC);
            normal.normalize();

            Vector3f nAB = v2->position - cp;
            Vector3f nAC = v3->position - cp;
            Vector3f normal2 = nAB.cross(nAC);

            if (normal.dot(normal2) < 0.f){
                skip = true;
                break;
            }
            currhed = currhed->twin->next;
        } while (currhed != D->halfedge);
    }

    if (!skip){
        C->degree -= 1;
        A->degree -= 1;
        B->degree += 2;

        B->position = cp;
        D->position = B->position;

        //delete D, DC, DB, BD, CD, DA, AD, one, two, edCD, edDA, edDB
        _HEfaces.erase(one->randid);
        _HEfaces.erase(two->randid);

        _HEverts.erase(D->randid);

        _edges.erase(edCD->randid);
        _edges.erase(edDA->randid);
        _edges.erase(edDB->randid);

        _halfedges.erase(DB->randid);
        _halfedges.erase(BD->randid);
        _halfedges.erase(CD->randid);
        _halfedges.erase(DA->randid);
        _halfedges.erase(BC->randid);
        _halfedges.erase(AB->randid);

        CB->twin = DC;
        BA->twin = AD;

        //top middle
        DC->vertex = B;
        DC->edge = edBC;
        DC->twin = CB;

        //bottom middle
        AD->twin = BA;
        AD->edge = edAB;

        C->halfedge = CB;
        A->halfedge = AD;
        B->halfedge = BA;

        edBC->halfedge = CB;
        edAB->halfedge = BA;

        std::shared_ptr<HE> curr = DC;
        curr = curr->next->next->twin;
        while (curr->next->next != AD){
            curr->vertex = B;
            curr = curr->next->next->twin;
        }
        curr->vertex = B;

        //reset affected quadrics and cost
        std::shared_ptr<HE> currhe1 = B->halfedge;
        do {
            std::shared_ptr<Face> f = currhe1->face;
            setFaceQuadric(f);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);

        do {
            setVertexQuadric(currhe1->twin->vertex);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);
        setVertexQuadric(B);

        do {
            std::shared_ptr<Vertex> v = currhe1->twin->vertex;
            std::shared_ptr<HE> vcurr = v->halfedge;
            do {
                std::shared_ptr<Edge> e = vcurr->edge;
                setEdgeQuadric(e);
                vcurr = vcurr->twin->next;
            } while (vcurr != v->halfedge);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);
        setVertexQuadric(B);
    }
    // if edge can't be collapsed, put in set to avoid putting it back on the priority queue
    else {
        skipEd.insert(edDB->randid);
    }
}

void Mesh::setFaceQuadric(std::shared_ptr<Face> f){
    //build quadric matrix for each face
    std::shared_ptr<Vertex> v1 = f->halfedge->vertex;
    std::shared_ptr<Vertex> v2 = f->halfedge->next->vertex;
    std::shared_ptr<Vertex> v3 = f->halfedge->next->next->vertex;
    Vector3f AB = v2->position - v1->position;
    Vector3f AC = v3->position - v1->position;
    Vector3f normal = AB.cross(AC);
    normal.normalize();
    Vector3f p = v1->position;
    float d = (-normal).dot(p);
    float a = normal[0];
    float b = normal[1];
    float c = normal[2];
    Vector4f v(a, b, c, d);

    Matrix4f q;
    q << a*a, a*b, a*c, a*d,
            a*b, b*b, b*c, b*d,
            a*c, b*c, c*c, c*d,
            a*d, b*d, c*d, d*d;
    f->q = q;
}

void Mesh::setVertexQuadric(std::shared_ptr<Vertex> v){
    // quadric for each vertex = sum of quadrics at all adjacent faces
    std::shared_ptr<HE> currhe = v->halfedge;
    Matrix4f quadric = Matrix4f::Zero();
    do {
        quadric += currhe->face->q;
        currhe = currhe->twin->next;
    } while (currhe != v->halfedge);
    v->q = quadric;
}

void Mesh::setEdgeQuadric(std::shared_ptr<Edge> e){
    // quadric for each edge = sum of quadrics at 2 endpoint vertices
    Matrix4f quad = e->halfedge->vertex->q;
    quad += e->halfedge->twin->vertex->q;
    e->q = quad;
    Matrix4f A;
    A << quad(0,0), quad(0,1), quad(0,2), quad(0,3),
            quad(0,1), quad(1,1), quad(1,2), quad(1,3),
            quad(0,2), quad(1,2), quad(2,2), quad(2,3),
            0.f, 0.f, 0.f, 1.f;
    Vector4f b(0.f, 0.f, 0.f, 1.f);
    Vector4f cp = A.inverse()*b;
    e->collapsepoint = Vector3f(cp[0], cp[1], cp[2]);
    Vector4f x(cp[0], cp[1], cp[2], 1.f);
    float cost = x.transpose()*quad*x;
    e->cost = cost;
}

void Mesh::setQuadrics(){
    for (auto it = _HEfaces.begin(); it != _HEfaces.end(); ++it ){
        std::shared_ptr<Face> f = it->second;
        setFaceQuadric(f);
    }
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        std::shared_ptr<Vertex> v = it->second;
        setVertexQuadric(v);
    }
    for (auto it = _edges.begin(); it != _edges.end(); ++it ){
        std::shared_ptr<Edge> e = it->second;
        setEdgeQuadric(e);
    }
}

void Mesh::simplify(int faces){
    int count = 0;
    int target = _HEfaces.size() - faces;

    setQuadrics();
    std::unordered_set<string> skipEd = {};

    while(_HEfaces.size() > target){
        priority_queue<std::shared_ptr<Edge>, std::vector<std::shared_ptr<Edge>>, costCompare> costs;
        for (auto it = _edges.begin(); it != _edges.end(); ++it ){
            std::shared_ptr<Edge> e = it->second;
            if (skipEd.find(e->randid) == skipEd.end()){
                costs.push(e);
            }
        }
        std::shared_ptr<Edge> top = costs.top();
        //collapse cheapest edge, set quadric at new vertex to sum of quadrics at endpoints of original edge
        collapse(top->halfedge, top->collapsepoint, skipEd);
        count++;
        //clear out skipped edges occasionally set in case they're no longer invalid for collapse
        if (count%1000 == 0){
            skipEd.clear();
        }
    }
}

void Mesh::subdivide(){
    std::vector<std::shared_ptr<Edge>> newedges;
    std::vector<std::shared_ptr<Edge>> edgecopy;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> oldverts;
    std::unordered_map<std::string, Vector3f> newpos;
    //store new positions separately
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        std::shared_ptr<Vertex> v = it->second;
        newpos[v->randid] = adjustPos(v);
        oldverts[v->randid] = v;
    }
    // store a copy of the old edges
    for (auto it = _edges.begin(); it != _edges.end(); ++it ){
        std::shared_ptr<Edge> e = it->second;
        edgecopy.push_back(e);
    }
    //split the old edges
    for(std::shared_ptr<Edge> e : edgecopy){
        split(e->halfedge, newedges, oldverts);
    }
    //flip the new edges touching one new and one old vertex
    for (std::shared_ptr<Edge> e : newedges){
        flip(e->halfedge);
    }
    //adjust positions for all vertices
    for (auto it = newpos.begin(); it != newpos.end(); ++it ){
        std::shared_ptr<Vertex> curr = oldverts[it->first];
        curr->position = it->second;
    }
    edgecopy.clear();
}

Vector3f Mesh::denoisePoint(std::shared_ptr<Vertex> v, float s_c, float s_s, float kernel, int depth){
    //get neighbors within kernel and adjust based on their positions
    Vector3f normal = getVertexNormal(v);
    std::unordered_set<string> neighbors = {};
    getNeighborSet(v, neighbors, v->position, 0, kernel, depth);
    neighbors.erase(v->randid);
    int K = neighbors.size();
    float sum = 0;
    float normalizer = 0;
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it){
        std::shared_ptr<Vertex> curr = _HEverts[*it];
        float t = (v->position - curr->position).norm();
        float h = normal.dot(curr->position - v->position);
        float w_c = exp(-pow(t, 2)/(2*pow(s_c, 2)));
        float w_s = exp(-pow(h, 2)/(2*pow(s_s, 2)));
        sum += (w_c*w_s)*h;
        normalizer += w_c*w_s;
    }
    if (normalizer == 0){
        return v->position;
    }
    Vector3f out = v->position + (normal*sum/normalizer);
    return out;
}

void Mesh::denoise(float s_c, float s_s, float kernel, int depth){
    //denoise each point and store new points before actually adjusting position
    std::unordered_map<std::string, Vector3f> newpos;
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        std::shared_ptr<Vertex> v = it->second;
        newpos[v->randid] = denoisePoint(v, s_c, s_s, kernel, depth);
    }
    for (auto it = newpos.begin(); it != newpos.end(); ++it ){
        std::shared_ptr<Vertex> curr = _HEverts[it->first];
        curr->position = it->second;
    }
}

void Mesh::createNoisySphere(){
    //for experimentation
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        std::shared_ptr<Vertex> v = it->second;
        Vector3f normal = getVertexNormal(v);
        float r = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.1f));
        v->position += normal*r;
    }
}

void Mesh::convertToOBJ(){
    _vertices.clear();
    _faces.clear();
    int count = 0;
    for (auto it = _HEfaces.begin(); it != _HEfaces.end(); ++it){
        count += 1;
        std::shared_ptr<Face> f = it->second;
        std::vector<int> idx;
        idx.reserve(3);
        std::shared_ptr<HE> currhe = f->halfedge;
        do {
            std::shared_ptr<Vertex> v = currhe->vertex; //hash unique id for vertex to vertex's new index
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
        }
        while (currhe != f->halfedge);
        _faces.emplace_back(Vector3i(idx[0], idx[1], idx[2]));
    }
}

Vector3f Mesh::adjustPos(std::shared_ptr<Vertex> v){
    // move position according to weights for splitting
    Vector3f adjust(0,0,0);
    int vd = v->degree;
    std::shared_ptr<HE> currhe = v->halfedge;
    int count = 0;
    do {
        std::shared_ptr<Vertex> currv = currhe->twin->vertex;
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
    //ensure no duplicates just in case
    if (usedids.find(str) != usedids.end()){
        return random_string();
    }
    else{
        usedids.insert(str);
        return str;
    }
}


int Mesh::getNumNeighbors(std::shared_ptr<Vertex> v){
    //for getting degree
    std::shared_ptr<HE> currhe = v->halfedge;
    int count = 0;
    do {
        currhe = currhe->twin->next;
        count++;
    } while (currhe != v->halfedge);
    return count;
}

Vector3f Mesh::getVertexNormal(std::shared_ptr<Vertex> v){
    std::shared_ptr<HE> currhe = v->halfedge;
    Vector3f bigNormal(0,0,0);
    int numFaces = 0;
    do {
        std::shared_ptr<Face> f = currhe->face;
        std::shared_ptr<Vertex> v1 = currhe->vertex;
        std::shared_ptr<Vertex> v2 = currhe->next->vertex;
        std::shared_ptr<Vertex> v3 = currhe->next->next->vertex;
        Vector3f AB = v2->position - v1->position;
        Vector3f AC = v3->position - v1->position;
        Vector3f normal = AB.cross(AC);
        normal.normalize();
        bigNormal += normal;
        numFaces++;
        currhe = currhe->twin->next;
    } while (currhe != v->halfedge);
    return bigNormal/numFaces;
}

void Mesh::getNeighborSet(std::shared_ptr<Vertex> v, unordered_set<string> &neighbors, Vector3f pos, int depth, float kernel, int maxdepth){
    std::shared_ptr<HE> currhe = v->halfedge;
    do {
        //euclidean distance
        float x = pos[0] - currhe->twin->vertex->position[0];
        float y = pos[1] - currhe->twin->vertex->position[1];
        float z = pos[2] - currhe->twin->vertex->position[2];
        float dist = pow(x,2) + pow(y,2) + pow(z,2);
        dist = sqrt(dist);

        //recurse if neighbor is within kernel distance, increment depth to prevent infinite recursion
        if (dist > 0.f && dist < kernel){
            neighbors.insert(currhe->twin->vertex->randid);
            if (depth < 3){
                getNeighborSet(currhe->twin->vertex, neighbors, pos, depth+1, kernel, maxdepth);
            }
        }
        currhe = currhe->twin->next;
    } while (currhe != v->halfedge);
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
