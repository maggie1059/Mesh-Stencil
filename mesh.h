#ifndef MESH_H
#define MESH_H

#include <vector>

#include <Eigen/StdVector>
#include <unordered_map>
#include <functional>
#include <Eigen/Geometry>
#include <unordered_set>

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
    Eigen::Matrix4f q;
};

struct Edge{
    HE *halfedge;
    std::string randid;
    Eigen::Matrix4f q;
    Eigen::Vector3f collapsepoint;
    float cost;
};

struct Face{
    HE *halfedge;
    std::string randid;
    Eigen::Matrix4f q;
};

struct VertTracker{
    int degree;
    Vertex *vert;
};

struct costCompare {
    bool operator()(const Edge* lhs, const Edge* rhs)
    {
        return lhs->cost > rhs->cost;
    }
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
    void createNoisySphere();
    void subdivide();
    void simplify(int faces);
    void denoise(float s_c, float s_s, float kernel);

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;

    std::unordered_set<string> usedids = {};

    std::unordered_map<std::string, HE*> _halfedges;
    std::unordered_map<std::string, Vertex*> _HEverts;
    std::unordered_map<std::string, Edge*> _edges;
    std::unordered_map<std::string, Face*> _HEfaces;

    std::unordered_map<int, VertTracker> _vertidx; //used to keep vertices unique and get degree
    std::unordered_map<std::pair<int, int>, HE*, hash_pair> _edgepairs; //used to get twins
    std::unordered_map<std::string, int> _lastmap; //used to convert back to obj
    std::string random_string();
    Eigen::Vector3f adjustPos(Vertex *v);

    void flip(HE *halfedge);
    void split(HE *halfedge, std::vector<Edge*> &newedges, const std::unordered_map<std::string, Vertex*> &oldverts);
    void collapse(HE *halfedge, Eigen::Vector3f cp, unordered_set<string> &skip);

    void setFaceQuadric(Face *f);
    void setVertexQuadric(Vertex *v);
    void setEdgeQuadric(Edge *e);
    void setQuadrics();
    Eigen::Vector3f denoisePoint(Vertex *v, float s_c, float s_s, float kernel);
    Eigen::Vector3f getVertexNormal(Vertex *v);

    int getNumNeighbors(Vertex *v);
    void getNeighborSet(Vertex *v, unordered_set<string> &neighbors, Eigen::Vector3f pos, int depth, float kernel);
};

#endif // MESH_H
