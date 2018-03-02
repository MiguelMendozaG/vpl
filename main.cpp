#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/correspondence.h>

#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>


using namespace std;
typedef pcl::PointXYZ pointType;
typedef pcl::PointCloud<pointType> pointCloudType;
typedef pcl::PointCloud<pointType>::Ptr pointCloudTypePtr;

//pcl::PCLPointCloud2::Ptr z (new pcl::PCLPointCloud2 ());
vector <pointCloudTypePtr> all_z;
 

/**
 * 
 * Reads the point clouds
 * 
 */
void read_all_z(std::string input_folder){
  vector< vector<double> > pcl_raw;
  pcl::PCDReader reader_pcl;
  pointCloudTypePtr pcl_file_pcd (new pcl::PointCloud<pointType>);
  for(int i = 0; i<100; i+=5){
    std::stringstream indice_img;
    indice_img << i;
    std::cout  << "\n Imagen " << i << endl;
    pointCloudTypePtr cloud (new pcl::PointCloud<pointType>);
    pcl::io::loadPCDFile(input_folder + "imagen-" +  indice_img.str() + ".pcd", *cloud);
    all_z.push_back(cloud);
  }  
}

void changeColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int R, int G, int B)
{
  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    { continue; }
    (*cloud)[i].r = R; (*cloud)[i].g = G; (*cloud)[i].b = B;
  }
}


/*
 * It computes the coorrespondent points
 * A correspondent point is apoint in the ground trouth that is closer than a threhold to the a target pc
 */
size_t correspondentPoints(pointCloudTypePtr grountTruth, pointCloudTypePtr target, pointCloudTypePtr corrCloud, double radius)
{
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  pcl::search::KdTree<pointType> tree;
  tree.setInputCloud (target);
  
  corrCloud->clear();
  size_t corr_points = 0;
  
  for (size_t i = 0; i < grountTruth->size (); ++i)
  {
    if (! pcl_isfinite ((*grountTruth)[i].x))
    {
      continue;
    }
    if(tree.radiusSearch((*grountTruth)[i], radius, indices, sqr_distances, 1))
    {
      corr_points++;
      corrCloud->push_back((*target)[indices[0]]);
    }
  }
  return corr_points;
}


 /*
  * 
  * Algorith that computes the "NBV" from a set of views and already taken point clouds
  * 
  */
int main (int argc, char** argv)
{
  // Check the following parameters
  
  // downsampling res
  float voxel_res = 0.0005;
  // correspond thres
  double corres_thres = 0.001;
  
  //Read the groundtruth
  pointCloudTypePtr ground_truth(new pcl::PointCloud<pointType>);
  pcl::io::loadPCDFile("./model/model2.pcd", *ground_truth);
  
  // read all the sensor positions and orientations
  
  // 1 Read all the posible sensor readings (Z). They are point clouds.
  std::string input_folder ("./z/");
  read_all_z(input_folder);
    
  double coverage = 0;
  double coverage_stop_threshold = 0.99;
  int iteration = 1;
  int iteration_stop = 10;
  int w_star_index = 0;
  pointType w_pos;
  pointType w_pos_end;
  double overlap;
  pointCloudTypePtr z_ptr;
  pointCloudTypePtr P_acu (new pointCloudType());
  pointCloudTypePtr P_overlap(new pointCloudType());
  
  // Create the filtering object
  pcl::VoxelGrid<pointType> voxel_filter;
  voxel_filter.setLeafSize(voxel_res, voxel_res, voxel_res);
  
  while (coverage < coverage_stop_threshold && iteration < iteration_stop) {
    cout << "Iteration:" << iteration << endl;
    z_ptr = all_z[w_star_index];
    // nbv position
    w_pos.x = 0.0;
    w_pos.y = 0.0;
    w_pos.z = 0.4;
    // nbv orientation
    w_pos_end.x = 0.0;
    w_pos_end.y = 0.0;
    w_pos_end.z = 0.3;
    
    
    // segment the object from z;
    
    
    // register z ; not necessary because they are in the world's reference frame
    
    
    // join both pc
    *P_acu = *P_acu + *z_ptr;    
    
    //filtering
    //voxel_filter.setInputCloud(P_acu);
    //voxel_filter.filter(*P_acu);
    //NOTE: it seems like the voxel filter is introducing an error so I removed from here 
    
    // compute current coverage
    // TODO: change for the groundtruth
    size_t positives;
    size_t total = ground_truth->size();
    positives = correspondentPoints(ground_truth, P_acu, P_overlap, corres_thres);
    coverage = (double)positives/total;
    //cout << "Correspondent points: " << (long)positives << endl;
    cout << "Current coverage: " << (double)positives/total << endl;
    
    
    // Select the NBV
    //TODO:
    w_star_index ++;
    
    
    //Compute the overlap of the NBV 
    positives = correspondentPoints(P_acu, all_z[w_star_index], P_overlap, corres_thres);
    total = P_acu->size();
    overlap = (double)positives/total;
    cout << "NBV overlap: " << overlap << endl;
    
    
    iteration ++;
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorAcu(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOver(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZ(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZstar(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*z_ptr, *colorZ);
    pcl::copyPointCloud(*P_acu, *colorAcu);
    pcl::copyPointCloud(*P_overlap, *colorOver);
    pcl::copyPointCloud(*(all_z[w_star_index]), *colorZstar);
    changeColor(colorZ, 0, 255, 0);
    
    // Visualización
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addCoordinateSystem(0.1);
    
    // Display z
    //viewer.addPointCloud(colorZ, "Z");
    //TODO: Add arrow with previous w*
    
    // Display accumulated point cloud
    changeColor(colorAcu, 255, 255, 255);
    viewer.addPointCloud(colorAcu, "Accumulated");
    //TODO: Add arrow with w*
    // Display z*
    changeColor(colorZstar, 0,0,255);
    viewer.addPointCloud(colorZstar, "Z star");
    
    // Display overlap
    changeColor(colorOver, 255, 0, 0);
    viewer.addPointCloud(colorOver, "Overlap");
    
    viewer.addLine<pointType>(w_pos, w_pos_end, 255, 255 , 255,"w_star_prev");
    
    viewer.spin();

  }// end while
  
  
  return 0;
  
  //  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
  //pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());
}