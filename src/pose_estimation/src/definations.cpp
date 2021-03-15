#include <main_class.h>




main_class::main_class(std::string image_path, std::string obj_path)
{
    input_image = cv::imread(image_path);
    pcl::io::loadOBJFile (obj_path, human_mesh);
    pcl::io::loadOBJFile(obj_path, human_points);
}



void main_class::add_cordinate_system(bool calculate_mean_point)
{
    pcl::PointXYZ mean_point;
    mean_point.x = 0;
    mean_point.y = 0;
    mean_point.z = 0;
    int count = 0;

    if (calculate_mean_point)
    {

        for (int i=0; i<human_points.points.size(); i++)
        {
            mean_point.x += human_points.points[i].x;
            mean_point.y += human_points.points[i].y;
            mean_point.z += human_points.points[i].z;
            count += 1;
        }

        if (count > 0)
        {
            mean_point.x /= count;
            mean_point.y /= count;
            mean_point.z /= count;
        }
        else
        {
            std::cout << "No points in human model\n";
            exit(0);
        }

    }


    viewer.addCoordinateSystem(0.1, mean_point.x, mean_point.y, mean_point.z, "mean_point_cordinate_sys");
}



void main_class::visualize_data()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_pntr (new pcl::PointCloud<pcl::PointXYZ>);
    *current_point_cloud_pntr = human_points;
    viewer.addPointCloud (current_point_cloud_pntr, "human_points");
    add_cordinate_system(true);
    viewer.addPolygonMesh(human_mesh, "human_mesh");

    std::vector<int> all_indices;
    all_indices = get_waist_points_indices();

    if (all_indices.size() > 0)
    {

        float waist_length = 0.0f;
        pcl::PointXYZ p1, p2;

        for (int i=1; i<all_indices.size(); i++)
        {
            p1 = current_point_cloud_pntr->points[all_indices[i-1]];
            p2 = current_point_cloud_pntr->points[all_indices[i]];

            float x = p1.x - p2.x;
            float y = p1.y - p2.y;
            float z = p1.z - p2.z;

            float dis = sqrt(x*x + y*y + z*z);
            waist_length += dis;

            viewer.addLine(p1, p2, 0, 1, 0, std::to_string(i));
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 20, std::to_string(i));
        }

        p1 = current_point_cloud_pntr->points[all_indices[0]];
        p2 = current_point_cloud_pntr->points[all_indices[all_indices.size() - 1]];

        float x = p1.x - p2.x;
        float y = p1.y - p2.y;
        float z = p1.z - p2.z;

        float dis = sqrt(x*x + y*y + z*z);
        waist_length += dis;

        std::cout << "waist_length (in inches)" << waist_length * 39.3701 << "\n";

        viewer.addLine(p1, p2, 0, 1, 0, std::to_string(0));
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 20, std::to_string(0));
    }


    viewer.spinOnce();

    viewer.removePointCloud("human_points");
    viewer.removePolygonMesh("human_mesh");
    viewer.removeCoordinateSystem("mean_point_cordinate_sys");


    for (int i=0; i<all_indices.size(); i++)
    {
        viewer.removeShape(std::to_string(i));
    }


    cv::imshow("input_image", input_image);
    cv::waitKey(1);
}



std::vector<int> main_class::get_waist_points_indices()
{
    std::string path = ros::package::getPath("pose_estimation");
    std::string data_folder_path(path + "/data/");
    std::ifstream myfile;
    myfile.open (data_folder_path + "waist_indices_final.txt", std::ios::in);

    std::vector<int> arr;
    int n;
    while (myfile >> n)
    {
        arr.push_back(n);
//        std::cout << n << "\n";
    }

    myfile.close();

    return arr;

}
