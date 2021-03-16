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
#include <pcl/visualization/point_picking_event.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <fstream>
#include <iostream>



/*  Use (Shift + left click) to get the point  */

pcl::PointXYZ first_point, current_point;
std::vector<int> all_indices;
bool exit_loop = false;


bool validate_index(int index_)
{
    bool new_index = true;
    for (int i=0; i<all_indices.size(); i++)
    {
        if (all_indices[i] == index_)
        {
            new_index = false;
            break;
        }
    }

    return new_index;
}




void get_single_index(pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_pntr)
{

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (current_point_cloud_pntr);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.nearestKSearch (current_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    //    std::cout << "\nall_indices ";
    //    for (int i=0; i<all_indices.size(); i++)
    //    {
    //        std::cout << " " << all_indices[i];
    //    }
    //    std::cout << "\n";

    if (all_indices.size() > 1 and pointIdxNKNSearch[0] == all_indices[0])
    {
        //        std::cout << "need to exit\n";
        exit_loop = true;
    }

    if ( (current_point.x != 0) and (current_point.y != 0) and (current_point.z != 0) and (validate_index(pointIdxNKNSearch[0])) )
    {
        all_indices.push_back(pointIdxNKNSearch[0]);
    }

    if ( (first_point.x == 0) and (first_point.y == 0) and (first_point.z == 0) )
    {
        first_point.x = current_point.x;
        first_point.y = current_point.y;
        first_point.z = current_point.z;

    }

}


void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event)
{

    float x, y, z;
    if (event.getPointIndex () == -1)
    {
        return;
    }
    event.getPoint(x, y, z);

    current_point.x = x;
    current_point.y = y;
    current_point.z = z;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_data");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::string obj_path("/home/mohit/I2L-MeshNet/demo/output_mesh_lixel.obj");

    pcl::PolygonMesh human_mesh;
    pcl::PointCloud<pcl::PointXYZ> human_points;
    pcl::visualization::PCLVisualizer viewer;

    pcl::io::loadOBJFile (obj_path, human_mesh);
    pcl::io::loadOBJFile(obj_path, human_points);

    first_point.x = first_point.y = first_point.z = 0;
    current_point.x = current_point.y = current_point.z = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_pntr (new pcl::PointCloud<pcl::PointXYZ>);
    *current_point_cloud_pntr = human_points;
    viewer.addPointCloud (current_point_cloud_pntr, "human_points");
    viewer.addPolygonMesh(human_mesh, "human_mesh");

    std::string path = ros::package::getPath("pose_estimation");
    std::string data_folder_path(path + "/data/");
    std::ofstream myfile;
    myfile.open (data_folder_path + "indices.txt", std::ios::out);


    while(ros::ok())
    {
        viewer.registerPointPickingCallback (pointPickingEventOccurred);
        get_single_index(current_point_cloud_pntr);


        if (all_indices.size() > 0)
        {

            std::cout << "all_indices.size() " << all_indices.size() << "\n";

            viewer.addSphere (current_point, 0.005, 1.0, 0.0, 0.0, "sphere");
            viewer.addSphere (first_point, 0.005, 0.0, 0.0, 1.0, "sphere2");

            for (int i=1; i<all_indices.size(); i++)
            {
                pcl::PointXYZ p1, p2;
                p1 = current_point_cloud_pntr->points[all_indices[i-1]];
                p2 = current_point_cloud_pntr->points[all_indices[i]];

                viewer.addLine(p1, p2, 0, 1, 0, std::to_string(i));
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 20, std::to_string(i));
            }



            //            std::cout << "all_indices.size() " << all_indices.size() << "\n";
            //            viewer.addSphere (first_point, 0.005, 0.0, 0.0, 1.0, "sphere2");
            //            for (int i=1; i<all_indices.size(); i++)
            //            {
            //                pcl::PointXYZ p1;
            //                p1 = current_point_cloud_pntr->points[all_indices[i]];
            //                viewer.addSphere (p1, 0.005, 1.0, 0.0, 0.0, std::to_string(i));
            //            }

        }


        viewer.spinOnce();

        if (all_indices.size() > 0)
        {
            //        viewer.removeShape("sphere");
            viewer.removeShape("sphere2");

            for (int i=1; i<all_indices.size(); i++)
            {
                viewer.removeShape(std::to_string(i));
            }
        }


        if (exit_loop == true)
        {
            break;
        }
    }

    for (int i=0; i< all_indices.size(); i++)
    {
        myfile << all_indices[i] << "\n";
    }

    myfile.close();

    return 0;
}
