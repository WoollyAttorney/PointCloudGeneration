# PointCloudGeneration
Make point cloud from a set of RGBD images (courtesy of [TUM Sequence 'freiburg1_floor' dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)).

## How to build 
### Dependencies
OpenCV: [Build Instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html).
PCL library: [Build Instructions](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#compiling-pcl-posix).
TUM dataset: [datasets](https://vision.in.tum.de/data/datasets/rgbd-dataset/download).

After downloading the dataset, a few more steps needs to be done in order for the code to work.
1. Go to src/generatePointCloud.cpp and change the dataset path to your dataset path.
2. Using the association tool from TUM website [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools), associate rgb.txt and depth.txt into rgb_depth.txt. Then associate rgb_depth.txt and groundtruth.txt into rgb_depth_gt.txt and change the following in src/generatePointCloud.cpp.

### Build
In the Project folder, create a build directory using 
```mkdir build```\n
compile using 
```cmake ../src```\n
then generate binary using 
```make -j4```\n
run the binary using 
```./generatPointCloud```\n
You will get a .ply file. To view the file use meshlab.


### Output
the output folder has a .zip file containing a .ply file which is already generated and can be viewed using meshlab. The folder also has a vidoe demo of the same.


### References
code is inspired from https://www.cnblogs.com/gary-guo/p/6542141.html.
Blog about the explanation https://www.programmersought.com/article/8647778259/

