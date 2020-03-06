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

        HE *he1 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()};
        HE *he2 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()};
        HE *he3 = new HE{NULL, NULL, NULL, NULL, NULL, random_string()};
        _halfedges[he1->randid] = he1;
        _halfedges[he2->randid] = he2;
        _halfedges[he2->randid] = he2;

        Face *f = new Face{he1, random_string(), Matrix4f::Zero()};
        he1->face = f;
        he2->face = f;
        he3->face = f;
        _HEfaces[f->randid] = f;

        he1->next = he2;
        he2->next = he3;
        he3->next = he1;

        if (_vertidx.find(iv1) == _vertidx.end()){
            Vertex *v1_he = new Vertex{he1, v1, 1, random_string(), Matrix4f::Zero()};
            he1->vertex = v1_he;
            _HEverts[v1_he->randid] = v1_he;
            _vertidx[iv1] = {1, v1_he};
        } else {
            _vertidx[iv1].degree++;
            _vertidx[iv1].vert->degree = _vertidx[iv1].degree;
            he1->vertex = _vertidx[iv1].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv1, iv2)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv2, iv1)) == _edgepairs.end()){
            Edge *e1 = new Edge{he1, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};
            he1->edge = e1;
            _edges[e1->randid] = e1;
            _edgepairs[std::pair<int,int>(iv1, iv2)] = he1;
        } else {
            Edge *e = _edgepairs[std::pair<int,int>(iv2, iv1)]->edge;
            he1->edge = e;
            _edgepairs[std::pair<int,int>(iv1, iv2)] = he1;
        }

        if (_vertidx.find(iv2) == _vertidx.end()){
            Vertex *v2_he = new Vertex{he2, v2, 1, random_string(), Matrix4f::Zero()};
            he2->vertex = v2_he;
            _HEverts[v2_he->randid] = v2_he;
            _vertidx[iv2] = {1, v2_he};
        } else {
            _vertidx[iv2].degree++;
            _vertidx[iv2].vert->degree = _vertidx[iv2].degree;
            he2->vertex = _vertidx[iv2].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv2, iv3)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv3, iv2)) == _edgepairs.end()){
            Edge *e2 = new Edge{he2, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};
            he2->edge = e2;
            _edges[e2->randid] = e2;
            _edgepairs[std::pair<int,int>(iv2, iv3)] = he2;
        } else {
            Edge *e = _edgepairs[std::pair<int,int>(iv3, iv2)]->edge;
            he2->edge = e;
            _edgepairs[std::pair<int,int>(iv2, iv3)] = he2;
        }

        if (_vertidx.find(iv3) == _vertidx.end()){
            Vertex *v3_he = new Vertex{he3, v3, 1, random_string(), Matrix4f::Zero()};
            he3->vertex = v3_he;
            _HEverts[v3_he->randid] = v3_he;
            _vertidx[iv3] = {1, v3_he};
        } else {
            _vertidx[iv3].degree++;
            _vertidx[iv3].vert->degree = _vertidx[iv3].degree;
            he3->vertex = _vertidx[iv3].vert;
        }

        if (_edgepairs.find(std::pair<int,int>(iv3, iv1)) == _edgepairs.end() && _edgepairs.find(std::pair<int,int>(iv1, iv3)) == _edgepairs.end()){
            Edge *e3 = new Edge{he3, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};

            he3->edge = e3;
            _edges[e3->randid] = e3;
            _edgepairs[std::pair<int,int>(iv3, iv1)] = he3;
        } else {
            Edge *e = _edgepairs[std::pair<int,int>(iv1, iv3)]->edge;
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

void Mesh::flip(HE *halfedge){
    HE *AB = halfedge; // 1
    HE *BA = halfedge->twin; // 2

    HE *BC = halfedge->next; // 1
    HE *CA = BC->next; // 1

    HE *AD = BA->next; // 2
    HE *DB = AD->next; // 2

    Face *ABC = halfedge->face; // 1
    Face *ADB = BA->face; // 2

    Vertex *A = halfedge->vertex;
    Vertex *B = BC->vertex;
    Vertex *C = CA->vertex;
    Vertex *D = DB->vertex;

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

void Mesh::split(HE *halfedge, std::vector<Edge*> &newedges, const std::unordered_map<std::string, Vertex*> &oldverts){
    HE *AB = halfedge; // 1
    HE *BA = halfedge->twin; // 2

    HE *BC = halfedge->next; // 1
    HE *CA = BC->next; // 1

    HE *AD = BA->next; // 2
    HE *DB = AD->next; // 2

    Edge *eddy = halfedge->edge;

    Face *one = halfedge->face; // 1
    Face *two = BA->face; // 2

    Vertex *A = halfedge->vertex;
    Vertex *B = BC->vertex;
    Vertex *C = CA->vertex;
    Vertex *D = DB->vertex;

    Vector3f position = (B->position*3.f/8.f) + (A->position*3.f/8.f) + (C->position*1.f/8.f) + (D->position*1.f/8.f);
    Vertex *E = new Vertex{BA, position, 4, random_string()};
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

    _halfedges[EC->randid] = EC;
    _halfedges[CE->randid] = CE;
    _halfedges[EB->randid] = EB;
    _halfedges[BE->randid] = BE;
    _halfedges[ED->randid] = ED;
    _halfedges[DE->randid] = DE;

    Edge *edEC = new Edge{EC, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};
    Edge *edEB = new Edge{EB, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};
    Edge *edED = new Edge{ED, random_string(), Matrix4f::Zero(), Vector3f(0,0,0), 0};

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

    Face *three = new Face{CA, random_string(), Matrix4f::Zero()};
    Face *four = new Face{AD, random_string(), Matrix4f::Zero()};

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

void Mesh::collapse(HE *halfedge, Vector3f cp, unordered_set<string> &skipEd){
    //inner
    HE *BD = halfedge;
    HE *DA = BD->next;
    HE *AB = DA->next;
    HE *DB = halfedge->twin;
    HE *BC = DB->next;
    HE *CD = BC->next;

    //outer
    HE *CB = BC->twin;
    HE *BA = AB->twin;

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

    Vertex *D = DA->vertex;
    Vertex *B = BD->vertex;
    Vertex *C = CD->vertex;
    Vertex *A = AB->vertex;

    //check validity
    bool skip = false;
    std::unordered_set<string> neighborVerts = {};
    HE *currhe = D->halfedge;
    do {
        neighborVerts.insert(currhe->twin->vertex->randid);
        currhe = currhe->twin->next;
    } while (currhe != D->halfedge);

    int count = 0;
    HE *currhe2 = B->halfedge;
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
        HE *currheb = B->halfedge;
        do {
            Face *f = currheb->face;
            if (f==one || f==two){
                currheb = currheb->twin->next;
                continue;
            }
            Vertex *v1 = currheb->vertex;
            Vertex *v2 = currheb->next->vertex;
            Vertex *v3 = currheb->next->next->vertex;
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

    if (!skip){
        HE *currhed = D->halfedge;
        do {
            Face *f = currhed->face;
            if (f==one || f==two){
                currhed = currhed->twin->next;
                continue;
            }
            Vertex *v1 = currhed->vertex;
            Vertex *v2 = currhed->next->vertex;
            Vertex *v3 = currhed->next->next->vertex;
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
        delete one;
        _HEfaces.erase(two->randid);
        delete two;

        _HEverts.erase(D->randid);
        delete D;

        _edges.erase(edCD->randid);
        delete edCD;
        _edges.erase(edDA->randid);
        delete edDA;
        _edges.erase(edDB->randid);
        delete edDB;

        _halfedges.erase(DB->randid);
        delete DB;
        _halfedges.erase(BD->randid);
        delete BD;
        _halfedges.erase(CD->randid);
        delete CD;
        _halfedges.erase(DA->randid);
        delete DA;
        _halfedges.erase(BC->randid);
        delete BC;
        _halfedges.erase(AB->randid);
        delete AB;

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

        HE *curr = DC;
        curr = curr->next->next->twin;
        while (curr->next->next != AD){
            curr->vertex = B;
            curr = curr->next->next->twin;
        }
        curr->vertex = B;

        //reset affected quadrics and cost
        HE *currhe1 = B->halfedge;
        do {
            Face *f = currhe1->face;
            setFaceQuadric(f);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);

        do {
            setVertexQuadric(currhe1->twin->vertex);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);
        setVertexQuadric(B);

        do {
            Vertex *v = currhe1->twin->vertex;
            HE *vcurr = v->halfedge;
            do {
                Edge *e = vcurr->edge;
                setEdgeQuadric(e);
                vcurr = vcurr->twin->next;
            } while (vcurr != v->halfedge);
            currhe1 = currhe1->twin->next;
        } while (currhe1 != B->halfedge);
        setVertexQuadric(B);
    }
    // if edge can't be collapse, put in set to avoid putting it back on the priority queue
    else {
        skipEd.insert(edDB->randid);
    }
}

void Mesh::setFaceQuadric(Face *f){
    //build quadric matrix for each face
    Vertex *v1 = f->halfedge->vertex;
    Vertex *v2 = f->halfedge->next->vertex;
    Vertex *v3 = f->halfedge->next->next->vertex;
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

void Mesh::setVertexQuadric(Vertex *v){
    // quadric for each vertex = sum of quadrics at all adjacent faces
    HE *currhe = v->halfedge;
    Matrix4f quadric = Matrix4f::Zero();
    do {
        quadric += currhe->face->q;
        currhe = currhe->twin->next;
    } while (currhe != v->halfedge);
    v->q = quadric;
}

void Mesh::setEdgeQuadric(Edge *e){
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
        Face *f = it->second;
        setFaceQuadric(f);
    }
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        Vertex *v = it->second;
        setVertexQuadric(v);
    }
    for (auto it = _edges.begin(); it != _edges.end(); ++it ){
        Edge *e = it->second;
        setEdgeQuadric(e);
    }
}

void Mesh::simplify(int faces){
    int count = 0;
    int target = _HEfaces.size() - faces;

    setQuadrics();
    std::unordered_set<string> skipEd = {};

    while(_HEfaces.size() > target){ //if want to use proportion of original size as threshold
//    for (int i = 0; i < faces; i++){
        priority_queue<Edge*, std::vector<Edge*>, costCompare> costs;
        for (auto it = _edges.begin(); it != _edges.end(); ++it ){
            Edge *e = it->second;
            if (skipEd.find(e->randid) == skipEd.end()){
                costs.push(e);
            }
        }
        Edge *top = costs.top();
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
    std::vector<Edge*> newedges;
    std::vector<Edge*> edgecopy;
    std::unordered_map<std::string, Vertex*> oldverts;
    std::unordered_map<std::string, Vector3f> newpos;
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        Vertex *v = it->second;
        newpos[v->randid] = adjustPos(v);
        oldverts[v->randid] = v;
    }

    for (auto it = _edges.begin(); it != _edges.end(); ++it ){
        Edge *e = it->second;
        edgecopy.push_back(e);
    }

    for(Edge *e : edgecopy){
        split(e->halfedge, newedges, oldverts);
    }
    for (Edge *e : newedges){
        flip(e->halfedge);
    }
    for (auto it = newpos.begin(); it != newpos.end(); ++it ){
        Vertex *curr = oldverts[it->first];
        curr->position = it->second;
    }
    edgecopy.clear();
}

Vector3f Mesh::denoisePoint(Vertex *v, float s_c, float s_s, float kernel){
    Vector3f normal = getVertexNormal(v);
    std::unordered_set<string> neighbors = {};
    getNeighborSet(v, neighbors, v->position, 0, kernel);
    neighbors.erase(v->randid);
    int K = neighbors.size();
    std::cout<< K << std::endl;
    float sum = 0;
    float normalizer = 0;
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it){
        Vertex *curr = _HEverts[*it];
        float t = (v->position - curr->position).norm();
//        float h = normal.dot(v->position - curr->position);
        float h = normal.dot(curr->position - v->position);
        float w_c = exp(-pow(t, 2)/(2*pow(s_c, 2)));
        float w_s = exp(-pow(h, 2)/(2*pow(s_s, 2)));
        sum += (w_c*w_s)*h;
        normalizer += w_c*w_s;
    }
    Vector3f out = v->position + (normal*sum/normalizer);
    return out;
}

void Mesh::denoise(float s_c, float s_s, float kernel){
    std::unordered_map<std::string, Vector3f> newpos;
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        Vertex *v = it->second;
        newpos[v->randid] = denoisePoint(v, s_c, s_s, kernel);
    }
    for (auto it = newpos.begin(); it != newpos.end(); ++it ){
        Vertex *curr = _HEverts[it->first];
        curr->position = it->second;
    }
}

void Mesh::createNoisySphere(){
    for (auto it = _HEverts.begin(); it != _HEverts.end(); ++it ){
        Vertex *v = it->second;
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
        Face *f = it->second;
        std::vector<int> idx;
        idx.reserve(3);
        HE *currhe = f->halfedge;
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
        }
        while (currhe != f->halfedge);
        _faces.emplace_back(Vector3i(idx[0], idx[1], idx[2]));
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
    if (usedids.find(str) != usedids.end()){
        return random_string();
    }
    else{
        usedids.insert(str);
        return str;
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

Vector3f Mesh::getVertexNormal(Vertex *v){
    HE *currhe = v->halfedge;
    Vector3f bigNormal(0,0,0);
    int numFaces = 0;
    do {
        Face *f = currhe->face;
        Vertex *v1 = f->halfedge->vertex;
        Vertex *v2 = f->halfedge->next->vertex;
        Vertex *v3 = f->halfedge->next->next->vertex;
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

void Mesh::getNeighborSet(Vertex *v, unordered_set<string> &neighbors, Vector3f pos, int depth, float kernel){
    HE *currhe = v->halfedge;
    do {
        float x = pos[0] - currhe->twin->vertex->position[0];
        float y = pos[1] - currhe->twin->vertex->position[1];
        float z = pos[2] - currhe->twin->vertex->position[2];
        float dist = pow(x,2) + pow(y,2) + pow(z,2);
        dist = sqrt(dist);
//        Vector3f distance = v->position-currhe->twin->vertex->position;
//        std::cout<< "dist: " << dist <<std::endl;
        if (dist > 0.f && dist < kernel){
            neighbors.insert(currhe->twin->vertex->randid);
            if (depth < 2){
                getNeighborSet(currhe->twin->vertex, neighbors, pos, depth+1, kernel);
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
