#ifndef __MAIN_CLASS_H_
#define __MAIN_CLASS_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>
#include <iostream>






typedef pcl::PointXYZRGBA PointType;


class main_class
{

private:
    cv::Mat input_image;
    pcl::PolygonMesh human_mesh;
    pcl::PointCloud<pcl::PointXYZ> human_points;



public:
    pcl::visualization::PCLVisualizer viewer;
    main_class(std::string, std::string);
    void add_cordinate_system(bool);
    void visualize_data();

    std::vector<int> get_waist_points_indices();


};

#endif //__MAIN_CLASS_H_
