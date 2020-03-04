#ifndef MESH_H
#define MESH_H

#include <vector>

#include <Eigen/StdVector>
#include <unordered_map>
#include <functional>
#include <Eigen/Geometry>

#include <string>
#include <utility>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)

using namespace std;
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const
    {
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

struct Vertex;
struct Edge;
struct Face;

struct HE{
    HE *twin;
    HE *next;
    Vertex *vertex;
    Edge *edge;
    Face *face;
    std::string randid;
};

struct Vertex{
    HE *halfedge;
    Eigen::Vector3f position;
    int degree;
    std::string randid;
};

struct Edge{
    HE *halfedge;
    std::string randid;
//    Vertex *vert1;
//    Vertex *vert2;
//    Eigen::Vector3f vert1;
//    Eigen::Vector3f vert2;
};

struct Face{
    HE *halfedge;
    std::string randid;
//    Eigen::Vector3i verts;
//    Eigen::Vector3f normal;
};

struct VertTracker{
    int degree;
    Vertex *vert;
};

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~Mesh();
    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
         const std::vector<Eigen::Vector3i> &faces);
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void convertToHE();
    void convertToOBJ();
    void fun();
    void fun2();
    void flip(HE *halfedge);
    void split(HE *halfedge, std::vector<Edge*> &newedges, const std::unordered_map<std::string, Vertex*> &oldverts);
    void collapse(HE *halfedge);
    void subdivide();

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;

//    std::vector<HE*> _halfedges;
//    std::vector<Vertex*> _HEverts;
//    std::vector<Edge*> _edges;
//    std::vector<Face*> _HEfaces;

    std::unordered_map<std::string, HE*> _halfedges;
    std::unordered_map<std::string, Vertex*> _HEverts;
    std::unordered_map<std::string, Edge*> _edges;
    std::unordered_map<std::string, Face*> _HEfaces;

    std::unordered_map<int, VertTracker> _vertidx; //used to keep vertices unique and get degree
    std::unordered_map<std::pair<int, int>, HE*, hash_pair> _idkmap; //used to get twins
    std::unordered_map<std::string, int> _lastmap; //used to convert back to obj
    std::string random_string();
    Eigen::Vector3f adjustPos(Vertex *v);
    int getNumNeighbors(Vertex *v);
};

#endif // MESH_H
