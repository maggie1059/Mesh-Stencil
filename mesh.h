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
struct Sad;

struct Fun{
    int hi;
    Sad *boo;
};

struct Sad{
    Fun *fun;
};

struct HE{
    HE *twin;
    HE *next;
    Vertex *vertex;
    Edge *edge;
    Face *face;
};

struct Vertex{
    HE *halfedge;
    Eigen::Vector3f position;
    int degree;
    std::string randid;
};

struct Edge{
    HE *halfedge;
    Eigen::Vector3f vert1;
    Eigen::Vector3f vert2;
};

struct Face{
    HE *halfedge;
    Eigen::Vector3i verts;
    Eigen::Vector3f normal;
};

struct VertTracker{
    int degree;
//    Edge *edge;
    Vertex *vert;
};

//class Vector3fHash{
//public:
//    bool operator()(const Eigen::Vector3f& v1) {
//       return v1[0];
//    }
//};

//template<typename T>
//struct matrix_hash : std::unary_function<T, size_t> {
//  std::size_t operator()(T const& matrix) const {
//    // Note that it is oblivious to the storage order of Eigen matrix (column- or
//    // row-major). It will give you the same hash value for two different matrices if they
//    // are the transpose of each other in different storage order.
//    size_t seed = 0;
//    for (size_t i = 0; i < matrix.size(); ++i) {
//      auto elem = *(matrix.data() + i);
//      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//    }
//    return seed;
//  }
//};

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
         const std::vector<Eigen::Vector3i> &faces);
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void convertToHE();
    void convertToOBJ();
    void fun();
    void fun2();

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    std::vector<HE*> _halfedges;
    std::vector<Vertex*> _HEverts;
    std::vector<Edge*> _edges;
    std::vector<Face*> _HEfaces;
//    std::unordered_map<int, VertTracker> _vertmap;
    std::unordered_map<int, VertTracker> _vertidx;
    std::unordered_map<std::pair<int, int>, HE*, hash_pair> _idkmap;
    std::unordered_map<std::string, int> _lastmap;
    std::string random_string();
//    std::unordered_map<Eigen::Vector3f, VertTracker, Vector3fHash> _vertmap;
//    std::unordered_map<Eigen::Vector3f, int, Vector3fHash> _vertidx;
    std::vector<Fun> _fun;
    std::vector<Sad> _sad;
};

#endif // MESH_H
