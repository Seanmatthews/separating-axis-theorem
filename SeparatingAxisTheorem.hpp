#ifndef SEPARATINGAXISTHEOREM_HPP_
#define SEPARATINGAXISTHEOREM_HPP_

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/convex_hull.h>


class SeparatingAxisTheorem
{
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef pcl::ConvexHull<pcl::PointXYZ>::Ptr HullPtr;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef Eigen::Vector2f EigenPt;

    // TODO:
    // OPtionally get overlap percentage with: https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm

    static void getProjectionAxesSAT(const std::vector<EigenPt>& verts,
                                     std::vector<EigenPt>& axes) {
        for (int i=0; i<verts.size(); ++i) {
            int j = (i+1)%(verts.size());
            auto pt1 = verts[i];
            auto pt2 = verts[j];
            EigenPt edgeNormal(pt2[1] - pt1[1], -(pt2[0] - pt1[0]));
            axes.push_back(edgeNormal);
        }        
    }

    static void projectOntoAxisSAT(const std::vector<EigenPt>& hullPts,
                                   const EigenPt axis,
                                   std::vector<EigenPt>& projPts) {
        for (auto p : hullPts)
        {
            EigenPt pp = (p.dot(axis) / axis.dot(axis)) * axis;
            projPts.push_back(pp);
        }
    }

    
    // Check whether two sets of collinear points overlap each other
    static bool pointsOverlapSAT(std::vector<EigenPt>& ptsA,
                                 std::vector<EigenPt>& ptsB) {
        // Find endpoints by sorting points
        auto sortFunc = [](const EigenPt a, const EigenPt b) {
            return a[0] == b[0] ? a[1] < b[1] : a[0] < b[0]; };
        std::sort(ptsA.begin(), ptsA.end(), sortFunc);
        std::sort(ptsB.begin(), ptsB.end(), sortFunc);

        // Determine overlap via: A--B, C--D, B >= C 0 && D >= A        
        return !sortFunc(ptsB[ptsB.size()-1], ptsA[0]) &&
               !sortFunc(ptsA[ptsA.size()-1], ptsB[0]);
    }


    // Check whether two hulls, projected onto a set of axes
    // (lines formed by vectors), overlap each other
    static bool projectionOverlapSAT(const std::vector<EigenPt>& ptsA,
                                     const std::vector<EigenPt>& ptsB,
                                     const std::vector<EigenPt>& axes) {
        std::vector<EigenPt> projA, projB;
        
        for (auto axis : axes)
        {
            // Project onto axis
            projectOntoAxisSAT(ptsA, axis, projA);
            projectOntoAxisSAT(ptsB, axis, projB);

            // If no overlap, return false
            if (!pointsOverlapSAT(projA, projB)) return false;

            projA.clear();
            projB.clear();
        }
        return true;
    }


public:
    
    // Check whether two 2D convex hulls overlap each other
    static bool overlap(HullPtr a, HullPtr b) {
        int dimension = a->getDimension();
        assert(b->getDimension() == dimension);
        assert(2 == dimension); // Currently only support 2D

        std::vector<EigenPt> axesA, axesB;
        std::vector<EigenPt> pointsA, pointsB;

        // Reconstruct with PointXYZ point cloud, regardless of how the hull was formed
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertsA(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertsB(new pcl::PointCloud<pcl::PointXYZ>);                
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
        getProjectionAxesSAT(pointsA, axesA);
        getProjectionAxesSAT(pointsB, axesB);
        
        return projectionOverlapSAT(pointsA, pointsB, axesA) &&
            projectionOverlapSAT(pointsA, pointsB, axesB);
        return false;
    }

};


#endif 
