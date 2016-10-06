#include "SeparatingAxisTheorem.hpp"
#include <vector>

using namespace std;


void SeparatingAxisTheorem::getProjectionAxes(const vector<EigenPt>& verts,
                                              vector<EigenPt>& axes)
{
    for (int i=0; i<verts.size(); ++i) {
        int j = (i+1)%(verts.size());
        auto pt1 = verts[i];
        auto pt2 = verts[j];
        EigenPt edgeNormal(pt2[1] - pt1[1], -(pt2[0] - pt1[0]));
        axes.push_back(edgeNormal);
    }        
}

void SeparatingAxisTheorem::projectOntoAxis(const vector<EigenPt>& hullPts,
                                            const EigenPt axis,
                                            vector<EigenPt>& projPts)
{
    for (auto p : hullPts)
    {
        EigenPt pp = (p.dot(axis) / axis.dot(axis)) * axis;
        projPts.push_back(pp);
    }
}

    
// Check whether two sets of collinear points overlap each other
bool SeparatingAxisTheorem::pointsOverlap(vector<EigenPt>& ptsA, vector<EigenPt>& ptsB)
{
    // Find endpoints by sorting points
    auto sortFunc = [](const EigenPt a, const EigenPt b) {
        return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0]; };
    sort(ptsA.begin(), ptsA.end(), sortFunc);
    sort(ptsB.begin(), ptsB.end(), sortFunc);
    
    // Determine overlap via: A--B, C--D, B >= C 0 && D >= A        
    return !sortFunc(ptsB[ptsB.size()-1], ptsA[0]) &&
           !sortFunc(ptsA[ptsA.size()-1], ptsB[0]);
}


// Check whether two hulls, projected onto a set of axes
// (lines formed by vectors), overlap each other
bool SeparatingAxisTheorem::projectionOverlap(const vector<EigenPt>& ptsA,
                                              const vector<EigenPt>& ptsB,
                                              const vector<EigenPt>& axes)
{    
    vector<EigenPt> projA, projB;
    
    for (auto axis : axes)
    {
        // Project onto axis
        projectOntoAxis(ptsA, axis, projA);
        projectOntoAxis(ptsB, axis, projB);
        
        // If no overlap, return false
        if (!pointsOverlap(projA, projB)) return false;
        
        projA.clear();
        projB.clear();
    }
    return true;
}


// Check whether two 2D convex hulls overlap each other
bool SeparatingAxisTheorem::overlap(HullPtr a, HullPtr b) {
    int dimension = a->getDimension();
    assert(b->getDimension() == dimension);
    assert(2 == dimension); // Currently only support 2D
    
    vector<EigenPt> axesA, axesB;
    vector<EigenPt> pointsA, pointsB;

    // Reconstruct with PointXYZ point cloud, regardless of how the hull was formed
    CloudPtr vertsA(new Cloud);
    CloudPtr vertsB(new Cloud);
    a->reconstruct(*vertsA);
    b->reconstruct(*vertsB);

    // if no vertices, cloud is empty
    assert(vertsA->points.size() > 0 && vertsB->points.size() > 0);

    // Convert to Eigen vector array
    // TODO how to acheive this????
//        Eigen::MatrixXf eMap = vertsA->getMatrixXfMap(dimension, 4, 0);

    // Get the points from cloud
    std::for_each(vertsA->points.begin(), vertsA->points.end(), [&pointsA](pcl::PointXYZ p) {
            pointsA.emplace_back(p.data);
        });
    std::for_each(vertsB->points.begin(), vertsB->points.end(), [&pointsB](pcl::PointXYZ p) {
            pointsB.emplace_back(p.data);
        });

    // Find the axes onto which we'll project the hulls
    getProjectionAxes(pointsA, axesA);
    getProjectionAxes(pointsB, axesB);
        
    return projectionOverlap(pointsA, pointsB, axesA) &&
        projectionOverlap(pointsA, pointsB, axesB);
}

