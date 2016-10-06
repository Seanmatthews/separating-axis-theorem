#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include "SeparatingAxisTheorem.hpp"

using namespace pcl;
using namespace pcl::visualization;
using namespace std;

typedef pcl::PointCloud<PointXYZ> Cloud;
typedef pcl::ConvexHull<PointXYZ>::Ptr HullPtr;
typedef Cloud::Ptr CloudPtr;
typedef Eigen::Vector2f EigenPt;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
int testNum = 0;

static void keyboardCallback(const KeyboardEvent kb)
{
    switch (kb.getKeyCode())
    {
    case 0x00000053: // S
        viewer->saveScreenshot("test.png");
    case 0x00000020: // Space
        viewer->close();
        break;
    case 0x0000001b:
        exit(0);
        break;
    default:
        break;
    }
    ++testNum;
}


int main(int argc, char** argv)
{
    // Seed rand() so we get different "random" numbers between runs
    srand(time(NULL));
    int size = 10;

    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 5, 0, 0, 0, 0, 0, 0);
    viewer->registerKeyboardCallback(keyboardCallback);

    for (;;)
    {
        PointCloud<PointXYZ>::Ptr cloudA(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr cloudB(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr reconA(new PointCloud<PointXYZ>);
        PointCloud<PointXYZ>::Ptr reconB(new PointCloud<PointXYZ>);
        ConvexHull<PointXYZ>::Ptr hullA(new ConvexHull<PointXYZ>),
            hullB(new ConvexHull<PointXYZ>);
        
        cloudA->points.resize(size);
        cloudB->points.resize(size);

        // Values will fall in [(-1,-1), (1,1)] range +/- 0.65 on X in order
        // to produce a healthy number of non-colliding polygons for testing.
        cout << "Use these if you want to experiment with a particular configuration:\n\n";
        for (size_t i = 0; i<size; ++i)
        {
            cloudA->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f) - 0.65;
            cloudA->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            cloudA->points[i].z = 1;

            cloudB->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f) + 0.65;
            cloudB->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            cloudB->points[i].z = 1;

            cout << "cloudA->emplace_back( " << cloudA->points[i].x << ", "
                 << cloudA->points[i].y << ", 1);" << endl;
            cout << "cloudB->emplace_back( " << cloudB->points[i].x << ", "
                 << cloudB->points[i].y << ", 1);" << endl;
        }
        
        hullA->setInputCloud(cloudA);
        hullA->reconstruct(*reconA);
        hullB->setInputCloud(cloudB);
        hullB->reconstruct(*reconB);

        for (int i=0; i<reconA->points.size(); ++i)
        {
            int j = (i+1)%(reconA->points.size());
            viewer->addLine<PointXYZ>(reconA->points[i], reconA->points[j],
                                      0, 255, 0, "lineA" + to_string(i));
        }
        for (int i=0; i<reconB->points.size(); ++i)
        {
            int j = (i+1)%(reconB->points.size());
            viewer->addLine<PointXYZ>(reconB->points[i], reconB->points[j],
                                      255, 0, 0, "lineB" + to_string(i));
        }
        
        bool overlap = SeparatingAxisTheorem::overlap(hullA, hullB);
        string text = "Overlap: ";
        if (overlap) text += "YES";
        else text += "NO";
        cout << text << endl;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloudA, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(cloudB, 255, 0, 0);
        viewer->addPointCloud<PointXYZ> (cloudA, green_color, "cloudA");
        viewer->addPointCloud<PointXYZ> (cloudB, red_color, "cloudB");
        viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3, "cloudA");
        viewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 3, "cloudB");            
        viewer->addText(text, 400, 10, 30, 1, 1, 1);

        viewer->spin();
        viewer->resetStoppedFlag();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
    }
    
    return 0;
}
