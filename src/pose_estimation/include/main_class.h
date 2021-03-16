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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <fstream>
#include <iostream>




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
    void visualize_data(bool, bool);

    std::vector<int> get_waist_points_indices();
    void get_plane_coefficients_for_height(pcl::ModelCoefficients::Ptr);
    std::vector<float> get_height_of_person(pcl::ModelCoefficients::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);


};

#endif //__MAIN_CLASS_H_
