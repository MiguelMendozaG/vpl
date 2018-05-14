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
//#include <pcl/registration/icp.h>


using namespace std;
typedef pcl::PointXYZ pointType;
typedef pcl::PointCloud<pointType> pointCloudType;
typedef pcl::PointCloud<pointType>::Ptr pointCloudTypePtr;
typedef pcl::PointXYZ PointType;

pointCloudTypePtr global_narf_points (new pointCloudType());

//////////////////////////////////////////
//////Global variables////////////
const int total_images = 1312;
const double coverage_stop_threshold = 0.80;
const int jumps = 5;
const int iteration_stop = 10;
const double w_escala_ini = 0.6;
const double w_escala_fin = 0.4;
const int n_end = 500;
double valor_octree = 0.0750925;
octomap::point3d x_min = {-valor_octree,-valor_octree,-valor_octree};  /// this values are set according eith the partialmodel file, it is refered to the bounding box
octomap::point3d x_max = {valor_octree, valor_octree, valor_octree};


//location folders
string my_direction("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo10/"); //location of the input dataset folder
string dir_nbv_narf ("/home/miguelmg/Documents/CIDETEC/'semestre 2'/'vision 3d'/proyecto/'6d pose'/hinterstoisser/nubes/modelo10/nbv/"); //in case your folder names contains space, you must specify it and add /nbv/ (e.g. ~/../'my foder'/nbv/)
string lectura_narf("./icp_narf_one_file_output " + dir_nbv_narf + "traslape.pcd" + " -m " + "-out " + dir_nbv_narf); //./executable input_point_cloud_dir -m -out output_narf_point_cloud_folder 
string direction_all_z (my_direction + "absolute/model/");
string direction_all_background (my_direction + "absolute/background/");
string direction_all_z_with_background (my_direction + "absolute/model_background/imagen-");
string direction_ground_truth ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/ground_truth_models/model10.pcd"); //ground truth point cloud location
string dir_nbv_i( my_direction + "nbv/");
string nub("/clouds");  // nubes
string num_poses("/num_pose");
string orn_pos ("/pose_orientation"); //pose_orientacion
string pos("/poses");
string octo_i ("/octo_i");
string octo_acum ("/octo_acum");
string direccion_posicion(my_direction + "position/pose/imagen_origin_");
string direccion_orientacion(my_direction + "position/orientation/imagen-ori-");
string pos_actual;
string file_name_scan;
string file_name_origin;
string octomaps;
string octomap_txt;

//pcl::PCLPointCloud2::Ptr z (new pcl::PCLPointCloud2 ());
vector <pointCloudTypePtr> all_z;
vector <pointCloudTypePtr> all_background;

/**
 * 
 * Reads the point clouds
 * 
 */
void read_all_z(std::string input_folder, bool z_or_bg){
  vector< vector<double> > pcl_raw;
  pcl::PCDReader reader_pcl;
  pointCloudTypePtr pcl_file_pcd (new pcl::PointCloud<pointType>);
  for(int i = 0; i<total_images; i+=jumps){
    std::stringstream indice_img;
    indice_img << i;
    std::cout  << "\n Imagen " << i << endl;
    pointCloudTypePtr cloud (new pcl::PointCloud<pointType>);
    pcl::io::loadPCDFile(input_folder + "imagen-" +  indice_img.str() + ".pcd", *cloud);
    if (z_or_bg)
      all_z.push_back(cloud);
    else
      all_background.push_back(cloud);
    
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
size_t correspondentPoints(pointCloudTypePtr grountTruth, pointCloudTypePtr target, pointCloudTypePtr corrCloud, double radius, bool save_)
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
  if (save_){
    *global_narf_points = *corrCloud;
    //cout << "\n Size of global_narf_points in correspondence: " << global_narf_points->size() << endl;
    //pcl::io::savePCDFile(dir_nbv_i + "traslape.pcd", *corrCloud);
    //cout << "\t corrCloud: " << corrCloud->size() << endl;
  }
  return corr_points;
}

bool narf_points(){
  //cout << "\n intro narfs" << endl;
  float angular_resolution = 0.5f;
  angular_resolution = pcl::deg2rad (angular_resolution);
  float support_size = 0.001f;
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  bool setUnseenToMaxRange = true;
  bool rotation_invariant = true;
  //pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *global_narf_points;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  //cout << "\n point_cloud size: " << point_cloud.points.size() << endl;
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
  
  // -----------------------------------------------
  // -----Create RangeImage from the PointCloud-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();
  
  // --------------------------------
  // -----Extract NARF keypoints-----
  // --------------------------------
  pcl::RangeImageBorderExtractor range_image_border_extractor;
  pcl::NarfKeypoint narf_keypoint_detector;
  narf_keypoint_detector.setRangeImageBorderExtractor (&range_image_border_extractor);
  narf_keypoint_detector.setRangeImage (&range_image);
  narf_keypoint_detector.getParameters ().support_size = support_size;
  
  pcl::PointCloud<int> keypoint_indices;
  narf_keypoint_detector.compute (keypoint_indices);
  //std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
  /*
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i){
    keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
    cout << "Puntos: " << range_image.points[keypoint_indices.points[i]].getVector3fMap() << endl;
    //cout << "X " << keypoin << endl;
    //NARF1[i] = double (keypoints.points[i].getVector3fMap ());
  }
  //cout << " todos " << keypoints.points[0].x << endl;
  cloud_one->width = keypoints.points.size();
  cloud_one->height = 1;
  cloud_one->is_dense = false;
  cloud_one->points.resize(cloud_one->width * cloud_one->height);
  for (size_t i=0; i<keypoint_indices.points.size (); ++i){
    cloud_one->points[i].x = keypoints.points[i].x;
    cloud_one->points[i].y = keypoints.points[i].y;
    cloud_one->points[i].z = keypoints.points[i].z;
  }
  //cout << " todos " << cloud_one->points[0].x << endl;
  cout << " points size: " << cloud_one->points.size() << endl;
  */
  range_image.reset();
  if (keypoint_indices.points.size() > 3)
    return true;
  else
    return false;
  
}

void file_octomap(string octo_input, string octo_file_output){
  octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read (octo_input);
  double occu_voxels = 0, free_voxels = 0;
  octomap::point3d p;
  int m=0;
  //if (tree){
    octomap::ColorOcTree* ot = dynamic_cast<octomap::ColorOcTree*>(tree);
    //cout << "\n octree " << ot << endl;
    if (ot){
      ofstream output_file(octo_file_output, ios::app);
      cout << "\n octomap read... creating file" << endl;
      ot->setBBXMax(x_max);
      ot->setBBXMin(x_min);
      ot->expand();
      cout << "\n bouning box " << ot->getBBXBounds() << endl;
      cout << "\n resolution " << ot->getResolution() << endl;
      for (octomap::ColorOcTree::leaf_iterator it = ot->begin_leafs(), end = ot->end_leafs(); it!=end; ++it){
	p = it.getCoordinate();
	if (ot->inBBX(p)){
	  //cout << "\n coord: " << p << endl;
	  if (ot->isNodeOccupied(*it)){
	    occu_voxels++;
	    output_file << p(0) <<" " << p(1) << " " << p(2) << " " << (it->getOccupancy()) <<endl;
	  }
	  else{
	    free_voxels++;
	    output_file <<p(0) <<" " << p(1) << " " << p(2) << " " << (it->getOccupancy()) << endl;
	  }
	}
      m++;
      }
   }
   else
     cout << "\n not read" << endl;
    
   delete tree;
   //delete ot;
   //delete p;
   cout << "\n\t occupied voxels = " << occu_voxels << endl;
   cout << "\n\t free voxels = " << free_voxels << endl;
  
}

 /*
  * 
  * Algorith that computes the "NBV" from a set of views and already taken point clouds
  * 
  */
int main (int argc, char** argv)
{
  
   std::cout << "Consejo Nacional de Ciencia y Tecnología" << std::endl;
  std::cout << "Instituto Nacional de Astrofísica Óptica y Electrónica" << std::endl;
  std::cout << "Centro de Innovación y Desarrollo Tecnológico en Cómputo" << std::endl;
  
  
  // octree resolution
  double resolution = 0.02;
  
  // Best scanning distance of the sensor
  double distance = 1.5;
  
  if (argc != 3) { // Check the value of argc. If not enough parameters have been passed, inform user and exit.
      std::cout << "Usage is ./program <folder> <ref_model>\n Note: The folder must contain inside the folders config and data. \n<ref_model> describes the points of the target model in order to calculate the coverage\nExample: ./program /home/robot"; 
      // Inform the user of how to use the program
      exit(0);
  } 
  
  std::string main_folder(argv[1]);
  std::string ref_model(argv[2]);
  
  std::string config_folder(main_folder);
  config_folder.append("/config");
  
  std::string data_folder(main_folder);
  data_folder.append("/data");
  // Check the following parameters
  
  // downsampling res
  float voxel_res = 0.0008;
  // correspond thres
  double corres_thres = 0.005;
  
  //Read the groundtruth
  pointCloudTypePtr ground_truth(new pcl::PointCloud<pointType>);
  pointCloudTypePtr ground_truth_saved(new pcl::PointCloud<pointType>);
  cout << "\n aqui" << endl;
  pcl::io::loadPCDFile(direction_ground_truth,*ground_truth);
  if (ground_truth->size()>0)
    cout << "\n Size of ground truth model: " << ground_truth->size() << endl;
  else{
    cout <<"\n Ground truth model not loaded" << endl;
    return 0;}
  *ground_truth_saved = *ground_truth;
  cin.get();
  //start loop for the reconstruction of all the poses
  read_all_z(direction_all_z,1);
  //read_all_z(direction_all_background,0);
  
    float alphaOcc = 0.2, alphaUnk = 0.8;
  /*
    PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk); //actual octree
      partial_model->setConfigFolder(config_folder);
      partial_model->setDataFolder(data_folder);
      partial_model->init();
   */   
    
    
    
    vpFileReader reader;
 
    pointCloudTypePtr W_pos (new pointCloudType());
      pointCloudTypePtr W_ast (new pointCloudType());
      pointCloudTypePtr z_ptr;
    pointCloudTypePtr z_ptr_background;
    pointCloudTypePtr P_acu (new pointCloudType());
    pointCloudTypePtr P_acu_saved (new pointCloudType());
    pointCloudTypePtr P_overlap(new pointCloudType());
    pointCloudTypePtr Pos_P_acu (new pointCloudType());
    vector < vector <double>> pose_;
    vector < vector <double>> orn_;
    pointType w_pos;
    pointType w_pos_end;
    pointType w_ast;
    pointType w_ast_end;
    int i_iterative =0;
  //partial_model_2->init();
  for (int i_reconstrucion = 575; i_reconstrucion < total_images ; i_reconstrucion+=jumps){
    //PartialModelBase *partial_model_2 = new PMVOctreeVasquez09(alphaOcc, alphaUnk); //acumulated octree
     PMVOctree* partial_model_2 = new PMVOctree();
    partial_model_2->setConfigFolder(config_folder);
    partial_model_2->setDataFolder(data_folder);
    partial_model_2->init();
    *ground_truth = *ground_truth_saved;
    int i_reconstrucion_mod = i_reconstrucion;
    double increment = 0;
    double max_increment = 0;
    double covtmas1 = 0;
    int max_intertoise = 0; 
    ostringstream i_recons;
    i_recons << i_reconstrucion;
    string indice_img = i_recons.str();
    
    string img_dir_actual(dir_nbv_i + indice_img);
    mkdir (img_dir_actual.c_str(),S_IRWXU);
    
    mkdir((dir_nbv_i + indice_img + pos).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos + num_poses).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos + orn_pos).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + nub).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + octo_i).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + octo_acum).c_str(), S_IRWXU);
    
    string data_num_poses(dir_nbv_i + indice_img + pos +"/num_pose/pose_nbv");
    string data_orn_poses(dir_nbv_i + indice_img + pos +"/pose_orientation/pose_nbv");
    string data_nubes(dir_nbv_i + indice_img + nub +"/nube_nbv");
    string pos_actual_octo (dir_nbv_i + indice_img + octo_i + "/octomap_");
    string pos_acum_octo (dir_nbv_i + indice_img + octo_acum + "/octomap_acum_");
  
    // read all the sensor positions and orientations
    
    // 1 Read all the posible sensor readings (Z). They are point clouds.

    double coverage = 0;
    int iteration = 1, counter;
    int w_star_index , w_star_iter ;
    w_star_index = w_star_iter = i_reconstrucion;
    w_star_index /=jumps;
    /*pointType w_pos;
    pointType w_pos_end;
    pointType w_ast;
    pointType w_ast_end;*/
    double overlap=0;
    bool NARF_points = 0;
    cout << "\n ref" << endl;
    //cin.get();
    //z_ptr->clear();
    //z_ptr_background->clear();
    P_acu->clear();
    //cout << "\n P_acu clear " << endl;
    //cin.get();
    P_acu_saved->clear();
    P_overlap->clear();
    
    /*
    pointCloudTypePtr z_ptr;
    pointCloudTypePtr z_ptr_background;
    pointCloudTypePtr P_acu (new pointCloudType());
    pointCloudTypePtr P_acu_saved (new pointCloudType());
    pointCloudTypePtr P_overlap(new pointCloudType());*/
    
    cout << "\n"<<i_reconstrucion << "\t Iteration: " << iteration << endl;
    //w_star_iter = w_star_index;
    
    while (coverage < coverage_stop_threshold && iteration < iteration_stop) {
      //W_pos->clear();
      //W_ast->clear();
      ////pointCloudTypePtr P_acu_background (new pointCloudType());
      //pointCloudTypePtr W_pos (new pointCloudType());
      //pointCloudTypePtr W_ast (new pointCloudType());
      cout << "Reconstruction " << i_reconstrucion << " - Iteration: " << iteration << endl;
      counter = iteration;
      stringstream w_iter;
      stringstream iter_while;
      w_iter << w_star_iter;
      iter_while << iteration;
      z_ptr = all_z[w_star_index];
      //z_ptr_background = all_background[w_star_index];
      
      
      // nbv position  uncomment when the pcl with arrows is shown
      /*pcl::io::loadPCDFile(direccion_posicion + w_iter.str() + ".pcd", *W_pos);
      w_pos.x = (W_pos->points[0].x) * w_escala_ini ;
      w_pos.y = (W_pos->points[0].y) * w_escala_ini ;
      w_pos.z = (W_pos->points[0].z) * w_escala_ini;
      // nbv orientation
      w_pos_end.x = (W_pos->points[0].x) * w_escala_fin;
      w_pos_end.y = (W_pos->points[0].y) * w_escala_fin;
      w_pos_end.z = (W_pos->points[0].z) * w_escala_fin;
      
      cout << "\n w pos ini " << w_iter.str() << " " << W_pos->points[0].x << " " << W_pos->points[0].y << " " << W_pos->points[0].z << endl;
      //cout << "\n w pos fin " << w_pos_end.x << " " << w_pos_end.y << " " << w_pos_end.z << endl;
      //////////////////////////// finishes uncomment section for pcl arrows
      */
      
      ///save M_acu
      file_name_scan = direction_all_z_with_background + w_iter.str() + ".xyz";
      file_name_origin = direccion_posicion + w_iter.str() + ".dat";
      //partial_model->updateWithScan(file_name_scan,file_name_origin);
      //cout << "\n Before octomap creation " << endl;
      //cin.get();
      partial_model_2->updateWithScan(file_name_scan,file_name_origin);
      //octomaps = pos_actual_octo + iter_while.str() + ".ot";
      //partial_model->savePartialModel(octomaps);
      octomaps = pos_acum_octo + iter_while.str() + ".ot";
      partial_model_2->savePartialModel(octomaps);
      
      octomap_txt = pos_acum_octo + iter_while.str() + ".txt";
      //cout << "\n After octomap creation " << endl;
      //cin.get();
      file_octomap(octomaps, octomap_txt);
      //cout << "\n After file of octomap creation " << endl;
      //cin.get();
      
      // segment the object from z;
      
      
      // register z ; not necessary because they are in the world's reference frame
      
      
      *P_acu = *P_acu + *z_ptr;   // join both pc
      //cout << "\n After P_acu + z_ptr " << endl;
      //cin.get();
      
      //filtering
            // Create the filtering object
      pcl::VoxelGrid<pointType> voxel_filter;
      voxel_filter.setLeafSize(voxel_res, voxel_res, voxel_res);
      voxel_filter.setInputCloud(P_acu);
      voxel_filter.filter(*P_acu);
      //cout << "\n After filter" << endl;
      //cin.get();
      
      /*
      //cout << "\n Saving cloud" << endl;
      vector< vector < double> > M_acu;
      //string pos_actual;
      pcl::io::savePCDFile( data_nubes + iter_while.str() + ".pcd", *P_acu + *z_ptr_background); //saves the acumulated point cloud with background
      //pos_actual = data_nubes + iter_while.str() + ".pcd";
      reader.readDoubleCoordFromPCD((data_nubes + iter_while.str() + ".pcd").c_str(), M_acu,pos_actual); //it is loaded into a vector 
      reader.saveDoubleCoordinates((data_nubes + iter_while.str() + ".pcd").c_str(), M_acu); //it is saved into a .dat file without header
      file_name_scan = data_nubes + iter_while.str() + ".pcd";
      file_name_origin = 
      partial_model_2->updateWithScan();
      */
      
      // compute current coverage
      // TODO: change for the groundtruth
      size_t positives;
      *ground_truth = *ground_truth_saved;
      size_t total = ground_truth->size();
      P_overlap->clear();
      positives = correspondentPoints(ground_truth, P_acu, P_overlap, corres_thres,0);
      coverage = (double)positives/total;
      //cout << "Correspondent points: " << (long)positives << endl;
      cout << "Current coverage: " << (double)positives/total << endl;
      *P_acu_saved = *P_acu;
      //cout << "\n Before iterations" << endl;
      //cin.get();
      
      
      // Select the NBV
      //TODO:
      i_iterative = 0;
      max_increment = 0;
      for (int i=0; i < total_images ; i+=jumps){
	cout << "\n" << i_reconstrucion << "\t\t Iteration:" << iteration << "\n\t  Image: " << i  << " Actual pos_z: " << max_intertoise << " increases " << max_increment <<endl;
	cout << "\t\t  Actual coverage: "<< coverage << endl;
	  //Compute the overlap of the NBV 
	total = P_acu->size();
	P_overlap ->clear();
	positives = correspondentPoints(P_acu, all_z[i_iterative], P_overlap, corres_thres, 1);
	overlap = (double)positives/total;
	//cout << "NBV overlap: " << (double)positives/total << endl;
	//cout << "P_acu size: " << P_acu->size() << endl;
	//cin.get();
	if (overlap >= 0.50){
	  //cout << "\t\t Overlap in image "<< i << endl;
	  //pcl::io::savePCDFile(dir_nbv_i + "traslape.pcd", *P_acu);
	  //NARF_points = narf_correspondence();
	  NARF_points = narf_points();
	  //cin.get();
	  //Extract Narf points
	  if (NARF_points){
	    //cout << "\t\t\t Narf points in image " << i << endl;
	    *ground_truth = *ground_truth_saved;
	    //pointCloudTypePtr Pos_P_acu (new pointCloudType());
	    *Pos_P_acu = *P_acu_saved + *all_z[i_iterative];
	    P_overlap->clear();
	    total = ground_truth->size();
	    positives = correspondentPoints(ground_truth, Pos_P_acu, P_overlap, corres_thres,0);
	    increment = (double) positives/total;
	    increment = increment - coverage;
	    
	    //check increments
	    if (increment > max_increment){
	      cout << "\n\t*******     Image " << i << " increases in " << increment << "   ****** " << endl; 
	      max_intertoise = i;
	      max_increment = increment;
	      /*
	      
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorAcu(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOver(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZ(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZstar(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::copyPointCloud(*z_ptr, *colorZ);
	      pcl::copyPointCloud(*P_acu, *colorAcu);
	      pcl::copyPointCloud(*P_overlap, *colorOver);
	      pcl::copyPointCloud(*(all_z[i_iterative]), *colorZstar);
	      changeColor(colorZ, 0, 255, 0);
	      
	      // Visualización
	      pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	      viewer.setBackgroundColor(255,255,255);
	      viewer.addCoordinateSystem(0.1);
	      
	      // Display z
	      //viewer.addPointCloud(colorZ, "Z");
	      //TODO: Add arrow with previous w*
	      
	      // Display accumulated point cloud
	      changeColor(colorAcu, 0, 0, 0);
	      viewer.addPointCloud(colorAcu, "Accumulated");
	      //TODO: Add arrow with w*
	      // Display z*
	      changeColor(colorZstar, 0,0,255);
	      viewer.addPointCloud(colorZstar, "Z star");
	      
	      // Display overlap
	      changeColor(colorOver, 255, 0, 0);
	      viewer.addPointCloud(colorOver, "Overlap");
	      
	      viewer.addLine<pointType>(w_pos, w_pos_end, 0, 0 , 0,"w_star_prev");
	      
	      viewer.spin();
	      
	      */
	    }
	  }
	}
	*P_acu = *P_acu_saved;
	i_iterative++;
      }//end for
      //w_star_index ++;
      i_reconstrucion_mod = max_intertoise;
      w_star_index = max_intertoise/jumps;
      w_star_iter = max_intertoise;
      
      pos_actual = data_num_poses + iter_while.str() + ".dat";
      reader.saveData2Text<int>(i_reconstrucion_mod,pos_actual);
      //vector < vector <double>> pose_;
      //vector < vector <double>> orn_;
      std:stringstream indice_pose;
      indice_pose << max_intertoise;
      pos_actual = direccion_posicion + indice_pose.str() + ".dat";
      reader.readDoubleCoordinates(pos_actual, pose_);
      pos_actual = direccion_orientacion + indice_pose.str() + ".dat";
      reader.readDoubleCoordinates(pos_actual, orn_);
      pose_.insert(pose_.end(), orn_.begin(), orn_.end());
      pos_actual = dir_nbv_i + i_recons.str() + "/poses/pose_orientation/pose_orn" + iter_while.str() + ".dat";
      reader.saveDoubleCoordinates(pos_actual, pose_);
      iteration ++;
      
      
      
	      /*pointCloudTypePtr P_overlap_shown (new pointCloudType());
	      P_overlap_shown->clear();
	      positives = correspondentPoints(P_acu, all_z[w_star_index], P_overlap_shown, corres_thres,0);
	      pcl::io::loadPCDFile(direccion_posicion + indice_pose.str() + ".pcd", *W_ast);
	      w_ast.x = (W_ast->points[0].x) * w_escala_ini ;
	      w_ast.y = (W_ast->points[0].y) * w_escala_ini ;
	      w_ast.z = (W_ast->points[0].z) * w_escala_ini;
	      // nbv orientation
	      w_ast_end.x = (W_ast->points[0].x) * w_escala_fin;
	      w_ast_end.y = (W_ast->points[0].y) * w_escala_fin;
	      w_ast_end.z = (W_ast->points[0].z) * w_escala_fin;
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorAcu(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorOver(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZ(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorZstar(new pcl::PointCloud<pcl::PointXYZRGB>);
	      pcl::copyPointCloud(*z_ptr, *colorZ);
	      pcl::copyPointCloud(*P_acu, *colorAcu);
	      pcl::copyPointCloud(*P_overlap_shown, *colorOver);
	      pcl::copyPointCloud(*(all_z[w_star_index]), *colorZstar);
	      changeColor(colorZ, 0, 255, 0);
      // Visualización
	      pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	      pcl::visualization::PCLVisualizer viewer2("Cloud Viewer");
	      viewer.setBackgroundColor(255,255,255);
	      viewer.addCoordinateSystem(0.1);
	      
	      viewer2.setBackgroundColor(255,255,255);
	      viewer2.addCoordinateSystem(0.1);
	      
	      // Display z
	      //viewer.addPointCloud(colorZ, "Z");
	      //TODO: Add arrow with previous w*
	      
	      // Display accumulated point cloud
	      changeColor(colorAcu, 0, 0, 0); //white
	      viewer.addPointCloud(colorAcu, "Accumulated");
	      //TODO: Add arrow with w*
	      // Display z*
	      changeColor(colorZstar, 0,0,0); //blue
	      viewer2.addPointCloud(colorZstar, "Z star");
	      
	      // Display overlap
	      changeColor(colorOver, 255, 0, 0); //red
	      //------viewer.addPointCloud(colorOver, "Overlap");
	      
	      //viewer.addLine<pointType>(w_pos, w_pos_end, 255, 255 , 255,"w_star_prev");
	      //------viewer.addArrow<pointType>(w_pos_end ,w_pos, 0,0,0, 0,"wprev");
	      viewer2.addArrow<pointType>(w_ast_end ,w_ast, 0,0,0, 0,"w_prev");
	      
	      cout << "\n size p_acu: " << P_acu->size() << endl;
	      cout << "\n  size overlap: " << P_overlap->size() << endl;
	      cout << "\n   size overlap: " << all_z[w_star_index]->size() << endl;
	      viewer.spin();*/

    }// end while
    //cout << "\n Before clear partial_model_2" << endl;
    //cin.get();
    delete partial_model_2;
    //cout << "\n After clear partial_model_2" << endl;
    //cin.get();
  }
  
  cout << "\n Finish" << endl;
  return 0;
}