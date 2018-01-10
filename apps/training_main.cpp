#include <vp.h>
#include <rssraytracingoctree.h>
#include <pmvoctreevasquez09.h>
#include <training_planner.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

string dir_nbv_i("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/");


double compareClouds(std::vector< std::vector< double > > a, std::vector< std::vector< double > > reference,  double gap, bool voxel_filter )
{
  //double gap = 0.004;
   long int correspondences=0;
   vector< vector<double> > nube_aux;

   
   cout << "\ncomparando puntos ..." << endl;
   
   for(int i = 0 ; i < reference.size();i++){
     for(int j= 0; j< a.size(); j++)
     {
       if( fabs(reference[i][0] - a[j][0]) < gap ){
	  if( fabs(reference[i][1] - a[j][1]) < gap ){
	   if( fabs(reference[i][2] - a[j][2]) < gap ){
	     correspondences ++;
	     if (voxel_filter == 1){
	      nube_aux[correspondences][0] = (reference[i][0] + a[j][0])/2;
	      nube_aux[correspondences][1] = (reference[i][1] + a[j][1])/2;
	      nube_aux[correspondences][2] = (reference[i][2] + a[j][2])/2;
	     }
	     
	     //cout << "\n\t\t" << reference[i][0] << "\t\t" << reference[i][1] << "\t\t" << reference[i][2] << endl;
	     //cout << " i = " << i;
	     break;
	   }
	  }
       }
     }
   }
   if (voxel_filter == 0)
     nube_aux.clear();
   
   std::cout << "Correspondences: " << correspondences << std::endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   std::cout << "%" << percentage << std::endl;
   
   return percentage;
  
}


std::vector< std::vector< double > > voxel_filter(std::vector< std::vector< double > > a, std::vector< std::vector< double > > b, double gap){
  int rep=0, indice = 0, gap_complete = 0;
  std::vector< std::vector< double > > nube_escaza;
  nube_escaza.clear();
  for (int i = 0; i < a.size(); i++){
    rep = 0;
    for (int j = 0; j < b.size(); j++){
      gap_complete = 0;
      if ( fabs(a[i][0] - b[i][0]) < gap ){
	if ( fabs(a[i][1] - b[i][1]) < gap ){
	  if ( fabs(a[i][2] - b[i][2]) < gap ){
	    nube_escaza[indice][0] = (a[i][0] + b[j][0])/2;
	    nube_escaza[indice][1] = (a[i][1] + b[j][1])/2;
	    nube_escaza[indice][2] = (a[i][2] + b[j][2])/2;
	    rep = 1;
	    indice++;
	    gap_complete = 1;
	  }
	}
      }
      if( rep == 1 && gap_complete == 0 ){
	nube_escaza[indice][0] = b[j][0];
	nube_escaza[indice][1] = b[j][1];
	nube_escaza[indice][2] = b[j][2];
	indice++;
      }
      if (rep == 0 ){
	nube_escaza[indice][0] = a[i][0];
	nube_escaza[indice][1] = a[i][1];
	nube_escaza[indice][2] = a[i][2];
	indice++;
	nube_escaza[indice][0] = b[j][0];
	nube_escaza[indice][1] = b[j][1];
	nube_escaza[indice][2] = b[j][2];
	indice++;
	rep = 1;
      }
    }
  }
  cout << "\n\n\t\t voxelizacion ok" << endl;
  return nube_escaza;
}


void filtro_voxeles(int img){
  std::stringstream points;
  std::stringstream indice_img;
  indice_img << img;
  string data_nubes_denso (dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_nbv_aux.xyz");
  string data_nubes_denso_salida (dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_nbv_aux_salida.xyz");
  string pos_nube;
  string info("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1");
  string info_total;
  long int tam = 0;
  vector< vector<double> > pcl_nube;
  //Genera datos de cabecera de acuerdo al formato de la libreria pcl
  vpFileReader reader;
  reader.readDoubleCoordinates(data_nubes_denso, pcl_nube);
  tam = pcl_nube.size();
  points << tam;
  info_total = "# .PCD v0.7 - Point Cloud Data file format\n" + info + "\nWIDTH " + points.str() + "\nHEIGHT 1" + "\nPOINTS " + points.str() + "\nDATA ascii\n";
  //Guarda en archivo datos_cabecera.xyz los datos de cabecera para abrir en PCL
  std::ofstream cabecera;
  cabecera.open (dir_nbv_i + indice_img.str() + "/nube_model_denso/datos_cabecera.xyz");
  cabecera << info_total;
  cabecera.close();
  //abre archivos datos_cabecera.xyz y nube_nbv_aux.xyz y los junta en un solo archivo
  std::ifstream cabecera_comb (dir_nbv_i + indice_img.str() +"/nube_model_denso/datos_cabecera.xyz");  
  std::ifstream coord_comb (dir_nbv_i + indice_img.str() +"/nube_model_denso/nube_nbv_aux.xyz");
  std::ofstream combinado_filro_voxeles (dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_densa_pcl.xyz");
  combinado_filro_voxeles << cabecera_comb.rdbuf() << coord_comb.rdbuf();
  //Se abre archivo nube_escasa en PCL
  ////////////////////
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader_pcl;
  // Replace the path below with the path where you saved your file
  reader_pcl.read (dir_nbv_i + indice_img.str() +"/nube_model_denso/nube_densa_pcl.xyz", *cloud); // Remember to download the file first!

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
  writer.write (dir_nbv_i + indice_img.str() +"/nube_model_denso/nube_escasa_pcl.xyz", *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  ////////////////////

  std::string ID;
  int cuenta =0;
  ID = "WIDTH "; //id of the line we want to delete
    //ifstream read("infos.txt");
    ifstream read_f(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl.xyz");
    //ofstream write("tmp.txt"); 
    ofstream write(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz");
    if (read_f.is_open()) {
       std::string line;
       while (getline(read_f, line)) {
	 if (cuenta >10)
	   write << line + "\n";
	 //cout << line << endl;
          //if (line.find(ID) != std::string::npos)
             //write << line;
	 cuenta++;
       }
    } else {
       std::cerr << "Error: coudn't open file\n";
       /* additional handle */
    }

    read_f.close();
    write.close();
    //std::remove("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/nube_model_denso/nube_escasa_pcl.xyz");
    //std::rename("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz", "/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/nube_model_denso/nube_escasa_pcl.xyz");
  
}


int main(int argc, char **argv) {
  
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
 
  
  
  for (int img = 1; img <= 1312; img++){
    
    int indice = 0, t = 1;
    double max_increment = 0;
    int max_intertoise = 0; 
    double cov_t = 0, cov_tmas1 = 0;
    double increment = 0;
    double gap = 0.005;
    double traslape = 0;
  
    //std::stringstream indice_img;
    //indice_img << img;
    std::ostringstream oss;
    oss << img;
    string indice_img = oss.str();
    string nube_mod_den("/nube_model_denso");
    string nub("/nubes");
    string oct("/octomap");
    string oct_acum("/octomap_acumulado");
    string pos("/poses");
    //indice_img << img;
    vpFileReader reader;
    string img_dir_actual(dir_nbv_i + indice_img);

    mkdir (img_dir_actual.c_str(),S_IRWXU);
    mkdir((dir_nbv_i + indice_img + nube_mod_den).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + nub).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + oct).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + oct_acum).c_str(), S_IRWXU);
    mkdir((dir_nbv_i + indice_img + pos).c_str(), S_IRWXU);

    
    string acumulada("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/object_pts.dat");
    string direccion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/plano_con_modelo/imagen-");
    string direccion_posicion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/posicion/pose/imagen_origin_");
    string direccion_plano("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/planos/imagen-");
    string direccion_modelo("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/modelo/imagen-");
    string direccion_nube_filtrada(dir_nbv_i + indice_img +"/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz");
    string data_octomap(dir_nbv_i + indice_img + "/octomap/octomap_");
    string data_octomap_acum(dir_nbv_i + indice_img + "/octomap_acumulado/octomap_");
    string data_poses(dir_nbv_i + indice_img + "/poses/pose_nbv");
    string data_nubes(dir_nbv_i + indice_img + "/nubes/nube_nbv");
    string data_nubes_denso (dir_nbv_i + indice_img + "/nube_model_denso/nube_nbv_aux");
    string dir_mono("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/config/mono.dat");
    string extension(".dat");
    string extension_xyz(".xyz");
    string ext_oct(".ot");
    string pos_actual;
    string pos_actual_aux;
    string scan_actual;
    string scan_plano_t;
    string scan_modelo_t;
    string pos_actual_nubes;
    vector< vector<double> > z_t;
    vector< vector<double> > plano_t;
    vector< vector<double> > modelo_t;
    vector< vector<double> > nube_acumulada;
    vector< vector<double> > nube_acumulada_aux;
    vector< vector<double> > posible_acumulada;
    vector< vector<double> > p;
    vector< vector<double> > mono;
    vector< vector<double> > posible_z;
    
    nube_acumulada.clear();
    
    reader.readDoubleCoordinates(dir_mono,mono);
    
    
    float alphaOcc = 0.2, alphaUnk = 0.8;
    /*
      PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
      partial_model->setConfigFolder(config_folder);
      partial_model->setDataFolder(data_folder);
      partial_model->init();
      */
      // Reads the sensor definition
      //partial_model->readRays(rays_file);
  
//  for (int img = 0; img < 1312; img++){
    while (cov_t < 90){
      //float alphaOcc = 0.2, alphaUnk = 0.8;
      PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
      partial_model->setConfigFolder(config_folder);
      partial_model->setDataFolder(data_folder);
      partial_model->init();
      
      PartialModelBase *partial_model_2 = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
      partial_model_2->setConfigFolder(config_folder);
      partial_model_2->setDataFolder(data_folder);
      partial_model_2->init();
      
      std::stringstream ss;
      std::stringstream tt;
      std::stringstream tt_octo;
      ss << indice;
      tt << t;
      tt_octo << (t-1);
      
      cout << "\n\nVista " << img << " Posicionamiento " << t << endl;
      //posicionamiento
      pos_actual = direccion_posicion + ss.str() + extension;
      pos_actual_aux = pos_actual;
      reader.readDoubleCoordinates(pos_actual,p);
      
      
      cout << "\n\nVista " << img << " Percepcion " << t << endl;
      //percepcion  (z)
      scan_actual = direccion + ss.str() + extension; //obtiene modelo con plano de la percepcion actual
      reader.readDoubleCoordinates(scan_actual,z_t);
      
      scan_plano_t = direccion_plano + ss.str() + extension; //obtiene nube de puntos solamente del plano de la percepcion actual
      reader.readDoubleCoordinates(scan_plano_t,plano_t);
      
      scan_modelo_t = direccion_modelo + ss.str() + extension; //obtiene nube de puntos solamente del modelo de la percepcion actual
      reader.readDoubleCoordinates(scan_modelo_t,modelo_t);
      
      
      cout << "\n\nVista " << img << " Update" << t << endl;
      //update (M(z,t))
      
      partial_model->updateWithScan(scan_actual, pos_actual); //genera octomap de la nube de puntos de la percepcion actual
      cout << "\t\t\t Archivo pose " << pos_actual << endl;
      cout << "\t\t\t Archivo scan actual " << scan_actual << endl;
      pos_actual = data_octomap + tt_octo.str() + ext_oct;
      partial_model->savePartialModel(pos_actual);
      cout << "\n\tOctomap # " << t << "guardado"<< endl;
      
      
      cout << "\n\t--- Acumular Nube de puntos " << endl;
      nube_acumulada.insert(nube_acumulada.end(), modelo_t.begin(),modelo_t.end());
      pos_actual_nubes = data_nubes_denso + extension_xyz;
      reader.saveDoubleCoordinates(pos_actual_nubes, nube_acumulada); //guarda nube de puntos para proceder a filtrar
      ////-------filtro de voxeles
      filtro_voxeles(img);
      /////----- fin filtro de voxeles
      nube_acumulada.clear();
      pos_actual = direccion_nube_filtrada;
      reader.readDoubleCoordinates(pos_actual,nube_acumulada);
      
      
      nube_acumulada_aux.clear();
      nube_acumulada_aux.insert(nube_acumulada_aux.end(), nube_acumulada.begin(),nube_acumulada.end()); //se genera nube de puntos acumulada mas el plano de la actual percepcion
      nube_acumulada_aux.insert(nube_acumulada_aux.end(),plano_t.begin(),plano_t.end());
      pos_actual_nubes = data_nubes + tt.str() + extension_xyz;
      reader.saveDoubleCoordinates(pos_actual_nubes, nube_acumulada_aux); //guarda nube de puntos acumulada incluyendo plano
      cout << "\n\tnube acumulada #"<< t << " guardada" << endl;
      partial_model_2->updateWithScan(pos_actual_nubes, pos_actual_aux);
      pos_actual = data_octomap_acum + tt_octo.str() + ext_oct;
      partial_model_2->savePartialModel(pos_actual);
      cout << "\n\tOctomap acumulado # " << t << "guardado"<< endl;
      
      
      cout << "\n\nVista " << img << " Planificacion" << t << endl;
      //planificar ()
      cov_t = compareClouds(nube_acumulada,mono,gap,0);
      
      max_increment = 0;
      for (int j = 0; j<=1312; j++){
	std::stringstream jj;
	jj << j;
	cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Comparando imagen #" << j << endl;

	pos_actual = direccion_modelo + jj.str() + extension;
	reader.readDoubleCoordinates(pos_actual,posible_z);
	
	traslape = compareClouds(posible_z,nube_acumulada,gap,0);
	if (traslape >= 50)
	{ 
	  cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Traslape > 50% #" << j << endl;
	  cout << "\t\t\t Incremeno max actual = " << max_increment << "\t en img " << max_intertoise << endl;
	  posible_acumulada.clear();
	  posible_acumulada.insert(posible_acumulada.end(),nube_acumulada.begin(),nube_acumulada.end());
	  posible_acumulada.insert(posible_acumulada.end(), posible_z.begin(), posible_z.end());
	  cov_tmas1 = compareClouds(posible_acumulada,mono,gap,0);
	  increment = cov_tmas1 - cov_t;
	  
	  if (increment > max_increment)
	  {
	    cout << "\n\tencontre que la imagen " << j << " incrementa en " << increment << " porciento " << endl; 
	    max_intertoise = j;
	    max_increment = increment;      
	  }
	}
      }
      indice = max_intertoise;
      pos_actual = data_poses + tt.str() + extension;
      reader.saveData2Text<int>(indice,pos_actual);
      
      //nube_acumulada = (nube_acumulada,0.004);
      
      t++;
      delete partial_model;
      delete partial_model_2;
    }
  }
  
  
  return 0;
  
  
}