// Reference: https://www.cnblogs.com/gary-guo/p/6542141.html#commentform
//2019-05-05

// c++ standard library 
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
// opencv library
#include <opencv2/opencv.hpp>
// PCL library
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
 
using namespace std;
 
// Define the point cloud type
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 
// camera internal reference. Referenced from https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
const double camera_factor = 5000;
const double camera_cx = 318.6;
const double camera_cy = 255.3;
const double camera_fx = 517.3;
const double camera_fy = 516.5;

// change this appropriately to the path of the dataset
const string dataset_path = "../TUM_dataset/";
const string rgb_depth_gt_file = dataset_path + "rgb_depth_gt_associated.txt"; // path of associated rgb and depth file and ground truth file

// lists to hold data regarding depth and rgb files
vector<vector<string>> rgb_depth_gt_data; // each element is of type (timestamp1 rgb_path timestamp2 depth_path timestamp3 tx ty tz qx qy qz qw)


void get_contents(string file_path, vector<vector<string>> &out)
{
    // open .txt, read timestamp and file data, pass it to out and close file
    fstream file;
    file.open(file_path);
    string line;
    string l;
    vector<string> temp;
    if(file.is_open())
    {    
        while(getline(file, line))
        {
            if (line[0] == '#'){
                    continue;
                }
            stringstream ss(line);    
            while(getline(ss,l, ' '))
            {                
                temp.push_back(l);
            }
            out.push_back(temp);
            temp.clear();            
        }
    }
    file.close();
}

// main function
int main(int argc,char** argv)
{   
    int resolution = 10; // defines how dense or light the final Point Cloud is. Will use lot of RAM
    int cases_to_consider = 500; // No of images to consider, WARNING higher number will make use a lot of RAM. Use with caution.
    int skip = 1; // No of images to skip, in order to reduce data load.
    get_contents(rgb_depth_gt_file, rgb_depth_gt_data);
    int total_cases = min(cases_to_consider*skip, (int)rgb_depth_gt_data.size());
    
    //Point cloud variable
    //Create a null cloud using smart pointers. This kind of pointer will be automatically released when it is used up.
    PointCloud::Ptr cloud(new PointCloud);
    
    vector<float> pos, quat; // To hold bot position and orientation
    const vector<float> world_pos = {0., 0., 0.}; // Frame of reference
    vector<float> p1 = {0., 0., 0.}; // holds data of point cloud wrt world pos

    for(int i = 0; i < total_cases; i+=skip) {
        cout << "traversing img " << (int) i / skip + 1 << " of " << (int)total_cases / skip << endl;
        
        // Read rgb image and depth image, and converted to point cloud
        // Image matrix
        cv::Mat rgb, depth;
        string img_path = dataset_path + rgb_depth_gt_data[i][1];
        string depth_img_path = dataset_path + rgb_depth_gt_data[i][3];
        
        //rgb image is a color image of 8UC3
        rgb = cv::imread(img_path);
        //depth is a single-channel image of 16UC1. Note that flags are set to -1, indicating that the original data is read without modification.
        depth = cv::imread(depth_img_path, -1);
        
        // get positiion data
        pos.push_back(stof(rgb_depth_gt_data[i][5]));
        pos.push_back(stof(rgb_depth_gt_data[i][6]));
        pos.push_back(stof(rgb_depth_gt_data[i][7]));
        
        // get orientation data
        quat.push_back(stof(rgb_depth_gt_data[i][8]));
        quat.push_back(stof(rgb_depth_gt_data[i][9]));
        quat.push_back(stof(rgb_depth_gt_data[i][10]));
        quat.push_back(stof(rgb_depth_gt_data[i][11]));


        // Add robot position to Point Cloud with red marker
        PointT r;
        r.x = pos[0] + quat[3]*quat[3]*world_pos[0] + 2*quat[1]*quat[3]*world_pos[2] - 2*quat[2]*quat[3]*world_pos[1] + quat[0]*quat[0]*world_pos[0] + 2*quat[1]*quat[0]*world_pos[1] + 2*quat[2]*quat[0]*world_pos[2] - quat[2]*quat[2]*world_pos[0] - quat[1]*quat[1]*world_pos[0];
        r.y = pos[1] + 2*quat[0]*quat[1]*world_pos[0] + quat[1]*quat[1]*world_pos[1] + 2*quat[2]*quat[1]*world_pos[2] + 2*quat[3]*quat[2]*world_pos[0] - quat[2]*quat[2]*world_pos[1] + quat[3]*quat[3]*world_pos[1] - 2*quat[0]*quat[3]*world_pos[2] - quat[0]*quat[0]*world_pos[1];
        r.z = pos[2] + 2*quat[0]*quat[2]*world_pos[0] + 2*quat[1]*quat[2]*world_pos[1] + quat[2]*quat[2]*world_pos[2] - 2*quat[3]*quat[1]*world_pos[0] - quat[1]*quat[1]*world_pos[2] + 2*quat[3]*quat[0]*world_pos[1] - quat[0]*quat[0]*world_pos[2] + quat[3]*quat[3]*world_pos[2];
        r.r = 255;
        r.g = 0;
        r.b = 0;
        r.a = 255;
        cloud->points.push_back(r);
        // traverse the depth map            
        for(int m = 0; m<depth.rows; m+=resolution) {
            for(int n = 0; n<depth.cols; n+=resolution) {
            // Get the value at (m, n) in the depth map
            ushort d = depth.ptr<ushort>(m)[n];
            // d may have no value, if so, skip this point
            if(d == 0)
                continue;
            // d has a value, then add a point to the point cloud
            PointT p;           
            // Calculate the space coordinates of this point
            p1[2] = double(d)/camera_factor;  
            p1[0] = (n-camera_cx)*p1[2]/camera_fx;  
            p1[1] = (m-camera_cy)*p1[2]/camera_fy; 
        
            // transform to appropriate 3d coordinates
            p.x = pos[0] + quat[3]*quat[3]*p1[0] + 2*quat[1]*quat[3]*p1[2] - 2*quat[2]*quat[3]*p1[1] + quat[0]*quat[0]*p1[0] + 2*quat[1]*quat[0]*p1[1] + 2*quat[2]*quat[0]*p1[2] - quat[2]*quat[2]*p1[0] - quat[1]*quat[1]*p1[0];
            p.y = pos[1] + 2*quat[0]*quat[1]*p1[0] + quat[1]*quat[1]*p1[1] + 2*quat[2]*quat[1]*p1[2] + 2*quat[3]*quat[2]*p1[0] - quat[2]*quat[2]*p1[1] + quat[3]*quat[3]*p1[1] - 2*quat[0]*quat[3]*p1[2] - quat[0]*quat[0]*p1[1];
            p.z = pos[2] + 2*quat[0]*quat[2]*p1[0] + 2*quat[1]*quat[2]*p1[1] + quat[2]*quat[2]*p1[2] - 2*quat[3]*quat[1]*p1[0] - quat[1]*quat[1]*p1[2] + 2*quat[3]*quat[0]*p1[1] - quat[0]*quat[0]*p1[2] + quat[3]*quat[3]*p1[2];

            // Get its color from the rgb image
            //rgb is a three-channel BGR format, so get the colors in the following order.
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];
            p.a = 255;
            //Add p to the point cloud
            cloud->points.push_back(p);
            }                       
        }  
        pos.clear();
        quat.clear();          
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::io::savePLYFile("./pointcloud.ply", *cloud);
    // Clear the data and save
    cloud->points.clear();
    cout << "Point cloud saved." << endl;

	return 0;    
}