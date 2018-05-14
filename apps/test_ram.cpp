#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/correspondence.h>

#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>


#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>

#include "partialmodelbase.h"
#include "pmvoctreevasquez09.h"
#include "vpfilereader.h"

using namespace std;
typedef pcl::PointXYZ pointType;
typedef pcl::PointCloud<pointType> pointCloudType;
typedef pcl::PointCloud<pointType>::Ptr pointCloudTypePtr;
typedef pcl::PointXYZ PointType;

pointCloudTypePtr cloud (new pcl::PointCloud<pointType>);
vector <pointCloudTypePtr> all_z;
vector <pointCloudTypePtr> all_background;


int main(){
  string file1("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-0.xyz");
  string file2("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-1280.xyz");
  string file3("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-1220.xyz");
  string file4("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-420.xyz");
  string file5("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-1295.xyz");
  string file6("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo1/absolute/model_background/imagen-1300.xyz");
  
  string origin1("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_0.dat");
  string origin2("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_1280.dat");
  string origin3("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_1220.dat");
  string origin4("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_420.dat");
  string origin5("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_1295.dat");
  string origin6("/home/miguelmg/Desktop/secuencia0/poses/imagen_origin_1300.dat");
  
  string output1("/home/miguelmg/Desktop/secuencia0/octomap/octo1.ot");
  string output2("/home/miguelmg/Desktop/secuencia0/octomap/octo2.ot");
  string output3("/home/miguelmg/Desktop/secuencia0/octomap/octo3.ot");
  string output4("/home/miguelmg/Desktop/secuencia0/octomap/octo4.ot");
  string output5("/home/miguelmg/Desktop/secuencia0/octomap/octo5.ot");
  string output6("/home/miguelmg/Desktop/secuencia0/octomap/octo6.ot");

  string config_folder("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/config");
  string data_folder("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/ground_truth_models/mono.pcd");


  float alphaOcc = 0.2, alphaUnk = 0.8;

  PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk); //acumulated octree
  partial_model->setConfigFolder(config_folder);
  partial_model->setDataFolder(data_folder);
  partial_model->init();
  
  partial_model->updateWithScan(file1,origin1);
  partial_model->savePartialModel(output1);
  cout << "\nenter 1" << endl;
  cin.get();
  
  partial_model->updateWithScan(file2,origin2);
  partial_model->savePartialModel(output2);
  cout << "\n enter 2" << endl;
  cin.get();
  
  partial_model->updateWithScan(file3,origin3);
  partial_model->savePartialModel(output3);
  cout << "\n enter 3" << endl;
  cin.get();
  
  partial_model->updateWithScan(file4,origin4);
  partial_model->savePartialModel(output4);
  cout << "\n enter 4" << endl;
  cin.get();
  
  partial_model->updateWithScan(file5,origin5);
  partial_model->savePartialModel(output5);
  cout << "\n enter 5" << endl;
  cin.get();
  
  partial_model->updateWithScan(file6,origin6);
  partial_model->savePartialModel(output6);
  cout << "\n enter 6" << endl;
  cin.get();
return 0;
}
