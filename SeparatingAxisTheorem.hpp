#ifndef SEPARATINGAXISTHEOREM_HPP_
#define SEPARATINGAXISTHEOREM_HPP_

#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

// PCL/Eigen implementation of the the Separating Axis Theorem for 2D convex hull
// collision detection. See https://en.wikipedia.org/wiki/Hyperplane_separation_theorem
// and http://www.dyn4j.org/2010/01/sat/ for more details.

class SeparatingAxisTheorem
{
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    typedef pcl::ConvexHull<pcl::PointXYZ>::Ptr HullPtr;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef Eigen::Vector2f EigenPt;

    // TODO:
    // OPtionally get overlap percentage with: https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm

    SeparatingAxisTheorem() {}
    
    static void getProjectionAxes(const std::vector<EigenPt>& verts,
                                  std::vector<EigenPt>& axes);

    static void projectOntoAxis(const std::vector<EigenPt>& hullPts,
                                const EigenPt axis,
                                std::vector<EigenPt>& projPts);
    
    static bool pointsOverlap(std::vector<EigenPt>& ptsA, std::vector<EigenPt>& ptsB);

    static bool projectionOverlap(const std::vector<EigenPt>& ptsA,
                                  const std::vector<EigenPt>& ptsB,
                                  const std::vector<EigenPt>& axes);

public:
    
    // Check whether two 2D convex hulls overlap each other
    static bool overlap(HullPtr a, HullPtr b);
};


#endif 
