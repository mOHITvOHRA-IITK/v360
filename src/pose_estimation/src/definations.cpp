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



void main_class::visualize_data(bool visualize_waist, bool visualize_plane_for_height)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_pntr (new pcl::PointCloud<pcl::PointXYZ>);
    *current_point_cloud_pntr = human_points;
    viewer.addPointCloud (current_point_cloud_pntr, "human_points");
    add_cordinate_system(true);
    viewer.addPolygonMesh(human_mesh, "human_mesh");

    std::vector<int> all_indices;

    if (visualize_waist)
    {
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

    }


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_real (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector <float> human_height_vec;
    if (visualize_plane_for_height)
    {
        get_plane_coefficients_for_height(coefficients);

        float x=0;
        float y=0;
        float z=0;
        for (int i=0; i<human_points.points.size(); i++)
        {
            x += human_points.points[i].x;
            y += human_points.points[i].y;
            z += human_points.points[i].z;
        }

        x /= human_points.points.size();
        y /= human_points.points.size();
        z /= human_points.points.size();

        viewer.addPlane (*coefficients, x, y, z, "plane");

        human_height_vec = get_height_of_person(coefficients, projected_cloud_real);
        std::cout << "human_height " << human_height_vec[2] << "\n";

        for (int l=0; l<human_height_vec.size()-1; l++)
        {
            pcl::PointXYZ p1, p2;
            p1 = human_points.points[int(human_height_vec[l])];
            p2 = projected_cloud_real->points[int(human_height_vec[l])];

            viewer.addSphere (p1, 0.01, 0.0, 0.0, 1.0, "sphere" + std::to_string(l));
            viewer.addLine(p1, p2, 0, 1, 0, std::to_string(l));
            //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 20, std::to_string(l));
        }
    }

    viewer.spinOnce();

    viewer.removePointCloud("human_points");
    viewer.removePolygonMesh("human_mesh");
    viewer.removeCoordinateSystem("mean_point_cordinate_sys");


    for (int i=0; i<all_indices.size(); i++)
    {
        viewer.removeShape(std::to_string(i));
    }

    if (coefficients->values.size() > 0)
    {
        viewer.removeShape("plane");
    }

    if (human_height_vec.size() > 0)
    {
        for (int l=0; l<human_height_vec.size()-1; l++)
        {
            viewer.removeShape(std::to_string(l));
            viewer.removeShape("sphere" + std::to_string(l));
        }
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



void main_class::get_plane_coefficients_for_height(pcl::ModelCoefficients::Ptr coefficients)
{
    std::string path = ros::package::getPath("pose_estimation");
    std::string data_folder_path(path + "/data/");
    std::ifstream myfile;
    myfile.open (data_folder_path + "plane_points_for_height.txt", std::ios::in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int n;
    while (myfile >> n)
    {
        cloud->points.push_back(human_points.points[n]);
    }

    myfile.close();




    coefficients->values.clear();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

}



std::vector<float> main_class::get_height_of_person(pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud_real)
{
    std::vector<float> info;
    info.resize(3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud_pntr (new pcl::PointCloud<pcl::PointXYZ>);
    *current_point_cloud_pntr = human_points;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (current_point_cloud_pntr);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected_cloud_real);


    std::vector<float> dir_vec;
    dir_vec.resize(3);

    float max_dis1=0;
    float max_dis2=0;


    for (int i=0; i<human_points.points.size(); i++)
    {
        pcl::PointXYZ p1, p1_projected;
        p1 = human_points.points[i];
        p1_projected = projected_cloud_real->points[i];

        float x = p1.x - p1_projected.x;
        float y = p1.y - p1_projected.y;
        float z = p1.z - p1_projected.z;
        float dis = sqrt(x*x + y*y + z*z);

        dir_vec[0] = x/dis;
        dir_vec[1] = y/dis;
        dir_vec[2] = z/dis;

        float dot_p;
        dot_p = dir_vec[0]*coefficients->values[0] + dir_vec[1]*coefficients->values[1] + dir_vec[2]*coefficients->values[2];

        if (dot_p > 0)
        {
            if (dis > max_dis1)
            {
                max_dis1 = dis;
                info[0] = i;
            }
        }
        else
        {
            if (dis > max_dis2)
            {
                max_dis2 = dis;
                info[1] = i;
            }
        }

    }

    info[2] = max_dis1 + max_dis2;

    //    std::cout << "info " << info[0] << " " << info[1] << " " << info[2] << "\n"
    //              << "max_dis " << max_dis1 << " " << max_dis2 << "\n";

    return info;
}
