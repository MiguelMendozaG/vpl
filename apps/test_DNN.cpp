#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/correspondence.h>
#include <ctime>

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

string my_direction("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo11/");
string direccion_posicion(my_direction + "position/pose/imagen_origin_");
string direccion_nube_backg(my_direction + "absolute/model_background/imagen-");
string direccion_nube(my_direction + "absolute/model/");
string direccion_nube_acum("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/dataset/test_DNN/nube_acum.pcd");
string direccion_nube_acum_dat("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/dataset/test_DNN/nube_acum.dat");
string save_octree("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/dataset/test_DNN/");
string save_octree2txt;
string nube;
string pose_nube;
string direccion_octree;

using namespace std;
typedef pcl::PointXYZ pointType;
typedef pcl::PointCloud<pointType> pointCloudType;
typedef pcl::PointCloud<pointType>::Ptr pointCloudTypePtr;
typedef pcl::PointXYZ PointType;


double valor_octree = 0.086408; //11
//double valor_octree = 0.129113; //13
//double valor_octree = 0.0923735; //15


octomap::point3d x_min = {-valor_octree,-valor_octree,-valor_octree};  /// this values are set according with the partialmodel file, it is refered to the bounding box
octomap::point3d x_max = {valor_octree, valor_octree, valor_octree};

int equiv_class2pose(int class_){
  int pose;
  if (class_ == 0) return 85;
  else if(class_ == 1) return 145;
  else if(class_ == 2) return 270;
  else if(class_ == 3) return 495;
  else if(class_ == 4) return 610;
  else if(class_ == 5) return 655;
  else if(class_ == 6) return 955;
  else if(class_ == 7) return 840;
  else if(class_ == 8) return 1265;
  else if(class_ == 9) return 1115;
  else if(class_ == 10) return 1275;
  else if(class_ == 11) return 1050;
  else if(class_ == 12) return 1285;
  else if(class_ == 13) return 1310;
  else if(class_ == 14) return 1250;
  else{	
    cout << "Error! " << endl;
    return 0;
  }
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



int main(int argc, char** argv){
  vpFileReader reader;
  vector < vector <double>> P_acum_vector;
  pointCloudTypePtr cloud (new pcl::PointCloud<pointType>);
  pointCloudTypePtr P_acum (new pcl::PointCloud<pointType>);
  std::string main_folder(argv[1]);
  //std::string ref_model(argv[2]);
  
  std::string config_folder(main_folder);
  config_folder.append("/config");
  
  std::string data_folder(main_folder);
  data_folder.append("/data");
  
  PMVOctree* partial_model_2 = new PMVOctree();
    partial_model_2->setConfigFolder(config_folder);
    partial_model_2->setDataFolder(data_folder);
    partial_model_2->init();
    
  float voxel_res = 0.0008;
  int valor;
  
  
  //pose inicial
  int pose = 10;
  
  ////buscar posicion equivalente
  int pos_equiv;
  
  bool continuar=1,  intial_pose= 0;
  int iter_init = 1;
  
  P_acum->clear();
  cout << "Initial pose ? : (0/1) " << endl;
  cin >> intial_pose;
  if (intial_pose){
    cout << "Enter intial pose: " << endl;
    cin >> pose;
    pos_equiv = pose;
  }
  while (continuar){
    stringstream pose_;
    pose_ << pose;
    
    if (intial_pose != 1)
      pos_equiv = equiv_class2pose(pose);
   
     intial_pose = 0;
    
    stringstream pose_equiv_;
    pose_equiv_ << pos_equiv;
 
    /*
    //leer nube
    pcl::io::loadPCDFile(direccion_nube + "imagen-" +  pose_equiv_.str() + ".pcd", *cloud);

    if (iter_init ==0){
      //leer nube acumulada anterior
      pcl::io::loadPCDFile(direccion_nube_acum, *P_acum);
      //iter_init = 1;
    }
    
    
    //sumar nubes
    *P_acum = *P_acum + *cloud;
    
    
    //filtar nubes
    pcl::VoxelGrid<pointType> voxel_filter;
    voxel_filter.setLeafSize(voxel_res, voxel_res, voxel_res);
    voxel_filter.setInputCloud(P_acum);
    voxel_filter.filter(*P_acum);
    
    //guardar nube acumulada actual -- quitar cabecera pcl
    pcl::io::savePCDFile(direccion_nube_acum, *P_acum);
    //reader.readDoubleCoordFromPCD(direccion_nube_acum,P_acum_vector,direccion_nube_acum);
    //reader.saveDoubleCoordinates(direccion_nube_acum_dat, P_acum_vector);
    */
    
    //leer nube acumulada
    nube = direccion_nube_backg + pose_equiv_.str() + ".xyz";
    pose_nube = direccion_posicion + pose_equiv_.str() + ".dat";
    
    
    //crear octree
    partial_model_2->updateWithScan(nube, pose_nube);
    //octomaps = pos_actual_octo + iter_while.str() + ".ot";
    //partial_model->savePartialModel(octomaps);
    direccion_octree = save_octree + "octree_" + pose_.str() + ".ot";
    partial_model_2->savePartialModel(direccion_octree);
    
    save_octree2txt = save_octree + "octree_pruned" + pose_.str() + ".txt";
    //cout << "\n After octomap creation " << endl;
    //cin.get();
    file_octomap(direccion_octree, save_octree2txt);
    
    cout << "Continue? (0/1): ";
    cin >> continuar;
    
    if (continuar){
      cout << "Predicted pose: " << endl;
      cin >> pose;
      iter_init = 0;
    }
    
    /*bool show = 0;
    cout << "Show point clouds?: (0/1)" << endl;
    cin >> show;
    if (show){
      
      
    }*/
    
    
  }
  
  return 0;
}