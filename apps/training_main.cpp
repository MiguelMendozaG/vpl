#include <stdio.h>
#include <vp.h>
#include <rssraytracingoctree.h>
#include <pmvoctreevasquez09.h>
#include <training_planner.h>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/correspondence.h>

typedef pcl::PointXYZ PointType;

int jumps= 5;
std::vector< std::vector< std::vector <double> > >  all_z;
std::vector< std::vector< std::vector <double> > >  backgrounds;


std::string my_direction("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/"); //location of the input dataset folder
std::string my_direction_spaces("/home/miguelmg/Documents/CIDETEC/'semestre 2'/'vision 3d'/proyecto/'6d pose'/hinterstoisser/nubes/modelo2/nbv/"); //location of the input dataset folder
std::string dir_nbv_narf ("/home/miguelmg/Documents/CIDETEC/'semestre 2'/'vision 3d'/proyecto/'6d pose'/hinterstoisser/nubes/modelo2/nbv/"); //in case your folder names contains space, you must specify it and add /nbv/ (e.g. ~/../'my foder'/nbv/)
std::string dir_mono("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/ground_truth_models/model2.xyz"); //ground truth point cloud location

std::string dir_nbv_i( my_direction + "nbv/");

std::string nube_mod_den("/dense_cloud"); // nube_model_denso
std::string nub("/clouds");  // nubes
std::string oct_acum("/accumulated_octomap"); // octomap_acumulado
std::string pos("/poses");
std::string dir_traslape("/overlapping_region"); //traslape
std::string carpeta_narf("/NARF_point_cloud"); 
std::string num_poses("/num_pose");
std::string orn_pos ("/pose_orientation"); //pose_orientacion
std::string direccion(my_direction + "absolute/model_background/imagen-");
std::string direccion_posicion(my_direction + "position/pose/imagen_origin_");
std::string direccion_orientacion(my_direction + "position/orientation/imagen-ori-");
std::string direccion_plano(my_direction + "absolute/background_filtered/imagen-");
std::string direccion_modelo(my_direction + "absolute/model/imagen-");
std::string direccion_read_all_z (my_direction + "absolute/model/");
std::string direccion_read_background (my_direction + "absolute/background_filtered/");
std::string extension(".dat");
std::string extension_xyz(".xyz");
std::string ext_oct(".ot");


void read_all_z(std::string input_folder, bool which_cloud){
  std::vector< std::vector<double> > pcl_raw;
  pcl::PCDReader reader_pcl;

  vpFileReader reader;

  for(int i = 0; i<1312; i+=jumps){
    std::stringstream indice_img;
    indice_img << i;
    std::string folder_pcl (input_folder + "imagen-" +  indice_img.str() + ".xyz");
    std::cout  << "\n Imagen " << i << endl;

    
    reader.readDoubleCoordinates(folder_pcl, pcl_raw);
    if (which_cloud == 0)
      all_z.push_back(pcl_raw);
    else
      backgrounds.push_back(pcl_raw);
  }
  //std::cout << "\n pcl raw" << all_z[0][0][0] << endl;
}


double compareClouds(std::vector< std::vector< double > > a, std::vector< std::vector< double > > reference,  double gap, bool voxel_filter, int img, int iteracion)
{
  vpFileReader reader;
   long int correspondences=0;
   pcl::PointCloud<pcl::PointXYZ>::Ptr narf_cloud (new pcl::PointCloud<pcl::PointXYZ>);

   
   std::cout << "\ncomparando puntos ..." << endl;
   
   for(int i = 0 ; i < reference.size();i++){
     for(int j= 0; j< a.size(); j++)
     {
       if( fabs(reference[i][0] - a[j][0]) < gap ){
	  if( fabs(reference[i][1] - a[j][1]) < gap ){
	   if( fabs(reference[i][2] - a[j][2]) < gap ){
	     correspondences ++;
	     break;
	   }
	  }
       }
     }
   }
   //// en caso de que se necesite generar nube de puntos traslapados
   if (voxel_filter == 1){
     std::cout << "correspondencias : " << correspondences << endl;
     long int correspondences_cloud = 0;
     narf_cloud->width = correspondences;/// se da tamaño a la nube de puntos
     narf_cloud->height = 1;
     narf_cloud->is_dense = false;
     narf_cloud->points.resize(narf_cloud->width * narf_cloud->height);
     for(int i = 0 ; i < reference.size();i++){
      for(int j= 0; j< a.size(); j++)
	{
	  if( fabs(reference[i][0] - a[j][0]) < gap ){
	      if( fabs(reference[i][1] - a[j][1]) < gap ){
	      if( fabs(reference[i][2] - a[j][2]) < gap ){
		  //std::cout << " Guardando imagen traslape " << endl;
		  narf_cloud->points[correspondences_cloud].x = a[j][0];
		  narf_cloud->points[correspondences_cloud].y = a[j][1];
		  narf_cloud->points[correspondences_cloud].z = a[j][2];
		correspondences_cloud ++;
		break;
	      }
	      }
	  }
	}
   }
   //// se guarda la nube de puntos en un archivo .pcd
   std::stringstream indice_img;
   indice_img << img;
   std::stringstream iter;
   iter << iteracion;
    
   std::string pos_traslape = dir_nbv_i + "traslape.pcd" ;
   pcl::io::savePCDFile(pos_traslape, *narf_cloud);
      
  }
   
   std::cout << "Correspondences: " << correspondences << std::endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   std::cout << "%" << percentage << std::endl;
   
   a.clear();
   reference.clear();
   return percentage;
  
}

void filtro_voxeles2(std::vector< std::vector< double > > nube_filtrada, int img){
  std::cout << "\n\t\t Cloud filter " << endl;
    std::stringstream indice_img;
  indice_img << img;
   pcl::PointCloud<pcl::PointXYZ>::Ptr nube_filtrada_pcl (new pcl::PointCloud<pcl::PointXYZ>);
   nube_filtrada_pcl->width = nube_filtrada.size();
   nube_filtrada_pcl->height = 1;
   nube_filtrada_pcl->is_dense = false;
   nube_filtrada_pcl->resize(nube_filtrada_pcl->width * nube_filtrada_pcl->height);
   long int m = 0;
   for (size_t i = 0; i < nube_filtrada.size(); i++){
     nube_filtrada_pcl->points[i].x = nube_filtrada[m][0];     
     nube_filtrada_pcl->points[i].y = nube_filtrada[m][1];
     nube_filtrada_pcl->points[i].z = nube_filtrada[m][2];
     m++;
  }
  pcl::io::savePCDFile(dir_nbv_i + indice_img.str() + nube_mod_den +"/nube_densa_pcl.xyz", *nube_filtrada_pcl);
  
  //Se abre archivo nube_escasa en PCL
  ////////////////////
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader_pcl;
  // Replace the path below with the path where you saved your file
  reader_pcl.read (dir_nbv_i + indice_img.str() + nube_mod_den +"/nube_densa_pcl.xyz", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.0008f, 0.0008f, 0.0008f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write (dir_nbv_i + indice_img.str() + nube_mod_den +"/nube_escasa_pcl.xyz", *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  ////////////////////

  std::string ID;
  int cuenta =0;
  ID = "WIDTH "; //id of the line we want to delete
    //ifstream read("infos.txt");
    std::ifstream read_f(dir_nbv_i + indice_img.str() + nube_mod_den +"/nube_escasa_pcl.xyz");
    //ofstream write("tmp.txt"); 
    std::ofstream write(dir_nbv_i + indice_img.str() + nube_mod_den +"/nube_escasa_pcl_sin_cabecera.xyz");
    if (read_f.is_open()) {
       std::string line;
       while (getline(read_f, line)) {
	 if (cuenta >10)
	   write << line + "\n";
	 cuenta++;
       }
    } else {
       std::cerr << "Error: coudn't open file\n";
       /* additional handle */
    }

    read_f.close();
    write.close();
    nube_filtrada.clear();
  
}

void save_chosen_narfs(int img, int iteracion){
  std::cout << "\n\t\t Saving NARF cloud" << endl;
  std::stringstream indice_img;
  indice_img << img;
  std::stringstream iter;
  iter << (iteracion-1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_narfs (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_traslape (new pcl::PointCloud<pcl::PointXYZ>);
  //std::string lectura_narf("./icp_narf_one_file_output " + dir_nbv_narf + indice_img.str() + "/traslape/traslape" + iter.str() + ".pcd" + " -m");
  pcl::io::loadPCDFile(dir_nbv_i + "narf_points.pcd", *cloud_of_narfs);
  pcl::io::loadPCDFile(dir_nbv_i + "traslape.pcd", *cloud_of_traslape);
  
  pcl::io::savePCDFile(dir_nbv_i + indice_img.str() +"/NARF_point_cloud/narf_cloud" + iter.str() + ".pcd", *cloud_of_narfs );
  pcl::io::savePCDFile(dir_nbv_i + indice_img.str() + dir_traslape + "/traslape_" + iter.str() + ".pcd", *cloud_of_traslape);
  
  
}


bool narf_correspondence( std::vector< std::vector< double > > posible_z, std::vector< std::vector< double > > acumulada, double gap, int img, int iteracion){
  std::stringstream indice_img;
  indice_img << img;
  std::stringstream iter;
  iter << iteracion;
  /// convertir posible_z a un archivo tipo pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr posible_z_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_narfs (new pcl::PointCloud<pcl::PointXYZ>);
  posible_z_pcl->width = posible_z.size();
  posible_z_pcl->height = 1;
  posible_z_pcl->is_dense = false;
  posible_z_pcl->resize(posible_z_pcl->width * posible_z_pcl->height);
  long int m = 0;
  for (size_t i = 0; i < posible_z.size(); i++){
    posible_z_pcl->points[i].x = posible_z[m][0];
    posible_z_pcl->points[i].y = posible_z[m][1];
    posible_z_pcl->points[i].z = posible_z[m][2];
    m++;
  }
  pcl::io::savePCDFile( dir_nbv_i + "posible_z.pcd" , *posible_z_pcl);  // se guarda una nube de puntos de posible_z
  std::cout << " posible z guardado " << endl;

  //// Extraer los Narf points de traslape
  std::string lectura_narf("./icp_narf_one_file_output " + dir_nbv_narf + "traslape.pcd" + " -m " + "-out " + my_direction_spaces); //./executable input_point_cloud_dir -m -out output_narf_point_cloud_folder 
  std::cout<< "\n\n\t Printing if processo is available..." << endl;
  if (system(NULL)) puts ("ok");
  else exit (EXIT_FAILURE);
  std::cout << "Executing command for NARF poits.."  << endl;
  system( (lectura_narf).c_str());
  std::cout << " Cloud of narf points saved " << endl;
  //std::cin.get();
  pcl::io::loadPCDFile(dir_nbv_i + "narf_points.pcd", *cloud_of_narfs);

  posible_z.clear();
  acumulada.clear();
  if (cloud_of_narfs->points.size() >= 3) // si existen al menos 3 narf points en el traslape, entonces regresa uno
    return 1;
  else // si hay menos de 3 puntos narfs en el traslape entonces regresa cero
    return 0; 
}


int main(int argc, char **argv) {
  //pcl::PointCloud<pcl::PointXYZ>::Ptr narf_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //std::cout<< "narf " <<  narf_cloud->points.x << endl;
 
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
  /*
  std::string rays_file(main_folder);
  rays_file.append("/data/sensor_rays.dat");
  */
 
  float alphaOcc = 0.2, alphaUnk = 0.8;
  
    PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
      partial_model->setConfigFolder(config_folder);
      partial_model->setDataFolder(data_folder);
      partial_model->init();
      
    
    PartialModelBase *partial_model_2 = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
    partial_model_2->setConfigFolder(config_folder);
    partial_model_2->setDataFolder(data_folder);
    partial_model_2->init();
    
     vpFileReader reader;
      
    read_all_z(direccion_read_all_z, 0);
    read_all_z(direccion_read_background, 1);
    std::cout << "\n all_z 0 " << backgrounds[0][0][0] << endl;
    std::cin.get();
      
  for (int img = 0; img <= 1312; img+=jumps){
    
    std::cout << "\n -------New Image img:" << img <<endl;
    
    int indice = img;
    int indice_2 = img;
    int t = 1;
    double max_increment = 0;
    int max_intertoise = 0; 
    double cov_t = 0, cov_tmas1 = 0;
    double increment = 0;
    double gap = 0.005;
    double traslape = 0;

    std::ostringstream oss;
    oss << img;
    std::string indice_img = oss.str();

     std::string oct("/octomap");

    vpFileReader reader;
    string img_dir_actual(dir_nbv_i + indice_img);

    mkdir (img_dir_actual.c_str(),S_IRWXU);
    mkdir((dir_nbv_i + indice_img + nube_mod_den).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + nub).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + oct).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + oct_acum).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos + num_poses).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos + orn_pos).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + dir_traslape).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + carpeta_narf).c_str(), S_IRWXU);

    std::string direccion_nube_filtrada(dir_nbv_i + indice_img + nube_mod_den +"/nube_escasa_pcl_sin_cabecera.xyz");
    std::string data_octomap(dir_nbv_i + indice_img + oct +"/octomap_");
    std::string data_octomap_acum(dir_nbv_i + indice_img + oct_acum +"/octomap_");
    std::string data_num_poses(dir_nbv_i + indice_img + pos +"/num_pose/pose_nbv");
    std::string data_orn_poses(dir_nbv_i + indice_img + pos +"/pose_orientacion/pose_nbv");
    std::string data_nubes(dir_nbv_i + indice_img + nub +"/nube_nbv");
    std::string data_nubes_denso (dir_nbv_i + indice_img + nube_mod_den +"/nube_nbv_aux");

    std::string pos_actual;
    std::string pos_actual_aux;
    std::string scan_actual;
    std::string scan_plano_t;
    std::string scan_modelo_t;
    std::string pos_actual_nubes;
    vector< vector<double> > z_t;
    vector< vector<double> > plano_t;
    vector< vector<double> > modelo_t;
    vector< vector<double> > nube_acumulada;
    vector< vector<double> > nube_acumulada_aux;
    vector< vector<double> > posible_acumulada;
    vector< vector<double> > p;
    vector< vector<double> > mono;
    vector< vector<double> > posible_z;
    bool NARF_points = 0;

    
    
    reader.readDoubleCoordinates(dir_mono,mono);
    
    
    partial_model_2->init();

    while (cov_t < 90){
    partial_model->init();
     
      std::stringstream ss;
      std::stringstream tt;
      std::stringstream tt_octo;
      ss << indice;
      tt << (t-1);
      tt_octo << (t-1);
      
      std::cout << "\n\nVista " << img << " Posicionamiento " << t << endl;
      //posicionamiento
      pos_actual = direccion_posicion + ss.str() + extension;
      pos_actual_aux = pos_actual;
      reader.readDoubleCoordinates(pos_actual,p);
      
      
      std::cout << "\n\nVista " << img << " Percepcion " << t << endl;
      //percepcion  (z)
      scan_actual = direccion + ss.str() + extension_xyz; //obtiene modelo con plano de la percepcion actual
      //reader.readDoubleCoordinates(scan_actual,z_t);
      
      
      scan_plano_t = direccion_plano + ss.str() + extension_xyz; //obtiene nube de puntos solamente del plano de la percepcion actual
      //reader.readDoubleCoordinates(scan_plano_t,plano_t);
      plano_t = backgrounds[indice_2];
      
      scan_modelo_t = direccion_modelo + ss.str() + extension_xyz; //obtiene nube de puntos solamente del modelo de la percepcion actual
      //reader.readDoubleCoordinates(scan_modelo_t,modelo_t);
      modelo_t = all_z[indice_2];
      
      
      
      std::cout << "\n\nVista " << img << " Update" << t << endl;
      //update (M(z,t))
      
      //partial_model->updateWithScan(scan_actual, pos_actual); //genera octomap de la nube de puntos de la percepcion actual
      std::cout << "\t\t\t Archivo pose " << pos_actual << endl;
      std::cout << "\t\t\t Archivo scan actual " << scan_actual << endl;
      pos_actual = data_octomap + tt_octo.str() + ext_oct;
      //partial_model->savePartialModel(pos_actual);
      std::cout << "\n\tOctomap # " << t << "guardado"<< endl;
      
      
      std::cout << "\n\t--- Acumular Nube de puntos " << endl;
      nube_acumulada.insert(nube_acumulada.end(), modelo_t.begin(),modelo_t.end());
      pos_actual_nubes = data_nubes_denso + extension_xyz;
      reader.saveDoubleCoordinates(pos_actual_nubes, nube_acumulada); //guarda nube de puntos para proceder a filtrar
      ////-------filtro de voxeles
      filtro_voxeles2(nube_acumulada, img);
      /////----- fin filtro de voxeles
      nube_acumulada.clear();
      pos_actual = direccion_nube_filtrada;
      reader.readDoubleCoordinates(pos_actual,nube_acumulada);
      
      
      nube_acumulada_aux.clear();
      nube_acumulada_aux.insert(nube_acumulada_aux.end(), nube_acumulada.begin(),nube_acumulada.end()); //se genera nube de puntos acumulada mas el plano de la actual percepcion
      nube_acumulada_aux.insert(nube_acumulada_aux.end(),plano_t.begin(),plano_t.end());
      pos_actual_nubes = data_nubes + tt.str() + extension_xyz;
      reader.saveDoubleCoordinates(pos_actual_nubes, nube_acumulada_aux); //guarda nube de puntos acumulada incluyendo plano
      std::cout << "\n\tnube acumulada #"<< t << " guardada" << endl;
      //partial_model_2->updateWithScan(pos_actual_nubes, pos_actual_aux);
      partial_model_2->updateWithScan(scan_actual, pos_actual_aux);
      pos_actual = data_octomap_acum + tt_octo.str() + ext_oct;
      partial_model_2->savePartialModel(pos_actual);
      std::cout << "\n\tOctomap acumulado # " << t << "guardado"<< endl;
      
      
      std::cout << "\n\nVista " << img << " Planificacion" << t << endl;
      //planificar ()
      cov_t = compareClouds(nube_acumulada,mono,gap,0, img, t);
      
      max_increment = 0;
      for (int j = 0; j<=1312; j+=jumps){
	int indice_3 = j;
	std::stringstream jj;
	jj << j;
	std::cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Comparando imagen #" << j << endl;
	std::cout << "\n\t\t\t Covertura " << cov_t << endl;

	pos_actual = direccion_modelo + jj.str() + extension_xyz;
	//reader.readDoubleCoordinates(pos_actual,posible_z);
	posible_z = all_z[indice_3];
	
	traslape = compareClouds(posible_z,nube_acumulada,gap,1, img, t);
	if (traslape >= 50)
	{ 
	  std::cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Traslape > 50% #" << j << endl;
	  
	  NARF_points = narf_correspondence(posible_z,nube_acumulada, gap, img, t);
	  if (NARF_points){ // si hay tres o mas narf points en el traslape, permite evaluar aumneto de covertura
	    std::cout << "\t\t\t Incremeno max actual = " << max_increment << "\t en img " << max_intertoise << endl;
	    posible_acumulada.clear();
	    posible_acumulada.insert(posible_acumulada.end(),nube_acumulada.begin(),nube_acumulada.end());
	    posible_acumulada.insert(posible_acumulada.end(), posible_z.begin(), posible_z.end());
	    cov_tmas1 = compareClouds(posible_acumulada,mono,gap,0,img,t);
	    increment = cov_tmas1 - cov_t;
	    
	    if (increment > max_increment)
	    {
	      std::cout << "\n\tencontré que la imagen " << j << " incrementa en " << increment << " porciento " << endl; 
	      save_chosen_narfs(img, t);
	      max_intertoise = j;
	      max_increment = increment;      
	    }
	  }
	}
      }
      
      indice = max_intertoise;
      indice_2 = max_intertoise;
      ///guarda el numero de la posicion que mas incrementó
      pos_actual = data_num_poses + tt.str() + extension;
      reader.saveData2Text<int>(indice,pos_actual);
      
      //guarda las coordenadas x,y,,z,yaw,pitch y roll de la posicion que mas incrementó
      vector < vector <double>> pose_;
      vector < vector <double>> orn_;
      std:stringstream indice_pose;
      indice_pose << max_intertoise;
      pos_actual = direccion_posicion + indice_pose.str() + extension;
      reader.readDoubleCoordinates(pos_actual, pose_);
      pos_actual = direccion_orientacion + indice_pose.str() + extension;
      reader.readDoubleCoordinates(pos_actual, orn_);
      pose_.insert(pose_.end(), orn_.begin(), orn_.end());
      pos_actual = dir_nbv_i + oss.str() + "/poses/pose_orientation/pose_orn" + tt.str() + extension;
      reader.saveDoubleCoordinates(pos_actual, pose_);
      
      t++;
      pose_.clear();
      orn_.clear();
    }
    nube_acumulada.clear();
    z_t.clear();
    plano_t.clear();
    modelo_t.clear();
    nube_acumulada_aux.clear();
    posible_acumulada.clear();
    p.clear();
    mono.clear();
    posible_z.clear();
  }
  delete partial_model_2;
  delete partial_model;
  
  return 0;
}
