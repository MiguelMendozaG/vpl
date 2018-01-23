#include <vp.h>
#include <rssraytracingoctree.h>
#include <pmvoctreevasquez09.h>
#include <training_planner.h>

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


std::string dir_nbv_i("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/");
std::string dir_nbv_narf ("/home/miguelmg/Documents/CIDETEC/'semestre 2'/'vision 3d'/proyecto/'6d pose'/hinterstoisser/nubes/nbv_i/");

typedef pcl::PointXYZ PointType;


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
    
    string dir_traslape (dir_nbv_i + indice_img.str() + "/traslape/traslape");
    string dir_traslape_ext (".pcd");
    
    string pos_traslape = dir_traslape + iter.str() + dir_traslape_ext;
   pcl::io::savePCDFile(pos_traslape, *narf_cloud);
   std::cout << " archivo narf guardado " << endl;
   //std::cin.get();
      
  }
   
   std::cout << "Correspondences: " << correspondences << std::endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   std::cout << "%" << percentage << std::endl;
   
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
  pcl::io::savePCDFile(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_densa_pcl.xyz", *nube_filtrada_pcl);
  
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
    std::ifstream read_f(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl.xyz");
    //ofstream write("tmp.txt"); 
    std::ofstream write(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz");
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
    std::ifstream read_f(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl.xyz");
    //ofstream write("tmp.txt"); 
    std::ofstream write(dir_nbv_i + indice_img.str() + "/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz");
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
}

bool narf_correspondence( std::vector< std::vector< double > > posible_z, std::vector< std::vector< double > > acumulada, double gap, int img, int iteracion){
  std::cout << "\n\n\t\t NARF Points de posible z" << endl;
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
//   std::cin.get();
  std::string lectura_narf("./icp_narf_one_file_output " + dir_nbv_narf + indice_img.str() + "/traslape/traslape" + iter.str() + ".pcd" + " -m"); 
  system( (lectura_narf).c_str());
  std::cout << " Cloud of narf points saved " << endl;
  //std::cin.get();
  pcl::io::loadPCDFile(dir_nbv_i + "narf_points.pcd", *cloud_of_narfs);
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
 
  
  for (int img = 15; img <= 1312; img+=5){
    
    std::cout << "\n -------New Image img:" << img <<endl;
    
    int indice = img;
    int t = 1;
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
    std::string indice_img = oss.str();
    std::string nube_mod_den("/nube_model_denso");
    std::string nub("/nubes");
    std::string oct("/octomap");
    std::string oct_acum("/octomap_acumulado");
    std::string pos("/poses");
    std::string dir_traslape("/traslape");
    std::string carpeta_narf("/NARF_point_cloud");
    std::string num_poses("/num_pose");
    std::string orn_pos ("/pose_orientacion");
    
    //indice_img << img;
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

    
    std::string acumulada("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/object_pts.dat");
    std::string direccion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/plano_con_modelo/imagen-");
    std::string direccion_posicion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/posicion/pose/imagen_origin_");
    std::string direccion_orientacion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/posicion/orientaciones/imagen-ori-");
    std::string direccion_plano("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/planos/imagen-");
    std::string direccion_modelo("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/modelo/imagen-");
    std::string direccion_nube_filtrada(dir_nbv_i + indice_img +"/nube_model_denso/nube_escasa_pcl_sin_cabecera.xyz");
    std::string data_octomap(dir_nbv_i + indice_img + "/octomap/octomap_");
    std::string data_octomap_acum(dir_nbv_i + indice_img + "/octomap_acumulado/octomap_");
    std::string data_num_poses(dir_nbv_i + indice_img + "/poses/num_pose/pose_nbv");
    std::string data_orn_poses(dir_nbv_i + indice_img + "/poses/pose_orientacion/pose_nbv");
    std::string data_nubes(dir_nbv_i + indice_img + "/nubes/nube_nbv");
    std::string data_nubes_denso (dir_nbv_i + indice_img + "/nube_model_denso/nube_nbv_aux");
    std::string dir_mono("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/config/mono.dat");
    std::string extension(".dat");
    std::string extension_xyz(".xyz");
    std::string ext_oct(".ot");
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
    
    nube_acumulada.clear();
    
    reader.readDoubleCoordinates(dir_mono,mono);
    
    
    float alphaOcc = 0.2, alphaUnk = 0.8;
    PartialModelBase *partial_model_2 = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
    partial_model_2->setConfigFolder(config_folder);
    partial_model_2->setDataFolder(data_folder);
    partial_model_2->init();

    while (cov_t < 90){
      //float alphaOcc = 0.2, alphaUnk = 0.8;
      PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
      partial_model->setConfigFolder(config_folder);
      partial_model->setDataFolder(data_folder);
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
      scan_actual = direccion + ss.str() + extension; //obtiene modelo con plano de la percepcion actual
      reader.readDoubleCoordinates(scan_actual,z_t);
      
      scan_plano_t = direccion_plano + ss.str() + extension; //obtiene nube de puntos solamente del plano de la percepcion actual
      reader.readDoubleCoordinates(scan_plano_t,plano_t);
      
      scan_modelo_t = direccion_modelo + ss.str() + extension; //obtiene nube de puntos solamente del modelo de la percepcion actual
      reader.readDoubleCoordinates(scan_modelo_t,modelo_t);
      
      
      std::cout << "\n\nVista " << img << " Update" << t << endl;
      //update (M(z,t))
      
      partial_model->updateWithScan(scan_actual, pos_actual); //genera octomap de la nube de puntos de la percepcion actual
      std::cout << "\t\t\t Archivo pose " << pos_actual << endl;
      std::cout << "\t\t\t Archivo scan actual " << scan_actual << endl;
      pos_actual = data_octomap + tt_octo.str() + ext_oct;
      partial_model->savePartialModel(pos_actual);
      std::cout << "\n\tOctomap # " << t << "guardado"<< endl;
      
      
      std::cout << "\n\t--- Acumular Nube de puntos " << endl;
      nube_acumulada.insert(nube_acumulada.end(), modelo_t.begin(),modelo_t.end());
      pos_actual_nubes = data_nubes_denso + extension_xyz;
      reader.saveDoubleCoordinates(pos_actual_nubes, nube_acumulada); //guarda nube de puntos para proceder a filtrar
      ////-------filtro de voxeles
      filtro_voxeles2(nube_acumulada, img);
      //filtro_voxeles(img);
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
      for (int j = 0; j<=1312; j+=5){
	std::stringstream jj;
	jj << j;
	std::cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Comparando imagen #" << j << endl;
	std::cout << "\n\t\t\t Covertura " << cov_t << endl;

	pos_actual = direccion_modelo + jj.str() + extension;
	reader.readDoubleCoordinates(pos_actual,posible_z);
	
	traslape = compareClouds(posible_z,nube_acumulada,gap,1, img, t);
	if (traslape >= 50)
	{ 
	  std::cout << "\n\t\t\tVista " << img << " Planificación # " << t <<" Traslape > 50% #" << j << endl;
	  
	  NARF_points = narf_correspondence(posible_z,nube_acumulada, gap, img, t);
	  std::cout<< "\n\t\t\t Narf points " << NARF_points << endl;
	  if (NARF_points){ // si hay tres o mas narf points en el traslape, permite evaluar aumneto de covertura
	    std::cout << "\t\t\t Mas de 3 NARF points" << endl;
	    std::cout << "\t\t\t Incremeno max actual = " << max_increment << "\t en img " << max_intertoise << endl;
	    posible_acumulada.clear();
	    posible_acumulada.insert(posible_acumulada.end(),nube_acumulada.begin(),nube_acumulada.end());
	    posible_acumulada.insert(posible_acumulada.end(), posible_z.begin(), posible_z.end());
	    cov_tmas1 = compareClouds(posible_acumulada,mono,gap,0,img,t);
	    increment = cov_tmas1 - cov_t;
	    
	    if (increment > max_increment)
	    {
	      std::cout << "\n\tencontré que la imagen " << j << " incrementa en " << increment << " porciento " << endl; 
	      max_intertoise = j;
	      max_increment = increment;      
	    }
	  }
	}
      }
      //// guarda nube de puntos de narfs de ultimo traslape
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_narf (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile(dir_nbv_i + "narf_points.pcd",*final_narf); //primero se guarda en final_narf
      pcl::io::savePCDFile(dir_nbv_i + oss.str() + "/NARF_point_cloud/narf_cloud" + tt.str() + ".pcd", *final_narf);
      
      indice = max_intertoise;
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
      pos_actual = dir_nbv_i + oss.str() + "/poses/pose_orientacion/pose_orn" + tt.str() + extension;
      reader.saveDoubleCoordinates(pos_actual, pose_);
      
      t++;
      delete partial_model;
      //delete partial_model_2;
    }
  }
  return 0;
}
