#include <main_class.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_data");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    main_class o("/home/mohit/I2L-MeshNet/demo/input.jpg", "/home/mohit/I2L-MeshNet/demo/output_mesh_lixel.obj");



    while(ros::ok())
    {
        o.visualize_data();
    }

    return 0;
}
