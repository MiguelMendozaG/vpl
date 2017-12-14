#include <vp.h>
#include <rssraytracingoctree.h>
#include <pmvoctreevasquez09.h>
#include <training_planner.h>

double compareClouds(std::vector< std::vector< double > > a, std::vector< std::vector< double > > reference,  double gap )
{
  //double gap = 0.004;
   long int correspondences=0;
   
   for(int i = 0 ; i < reference.size();i++){
     for(int j= 0; j< a.size(); j++)
     {
       if( fabs(reference[i][0] - a[j][0]) < gap ){
	  if( fabs(reference[i][1] - a[j][1]) < gap ){
	   if( fabs(reference[i][2] - a[j][2]) < gap ){
	     correspondences ++;
	     //cout << "\n\t\t" << reference[i][0] << "\t\t" << reference[i][1] << "\t\t" << reference[i][2] << endl;
	     break;
	   }
	  }
       }
     }
   }
   
   std::cout << "Correspondences: " << correspondences << std::endl;
   double percentage = (correspondences / (double) reference.size()) * 100 ;
   std::cout << "%" << percentage << std::endl;
   
   return percentage;
  
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
    
    std::string rays_file(main_folder);
    rays_file.append("/data/sensor_rays.dat");
  
 
  int indice = 0, t = 1;
  int max_increment = 0;
  int max_intertoise = 0; 
  double cov_t = 0, cov_tmas1 = 0;
  double increment = 0;
  double gap = 0.005;
  double traslape = 0;
  
  vpFileReader reader;
  string acumulada("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/object_pts.dat");
  string direccion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/imagen-");
  string extension(".dat");
  string ext_oct(".ot");
  string pos_actual;
  string scan_actual;
  string direccion_posicion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/posicion/pose/imagen_origin_");
  string data_octomap("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/octomap/octomap_");
  string data_poses("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/poses/pose_nbv");
  vector< vector<double> > z_t;
  vector< vector<double> > nube_acumulada;
  vector< vector<double> > posible_acumulada;
  vector< vector<double> > pos_bunny;
  vector< vector<double> > p;
  vector< vector<double> > mono;
  vector< vector<double> > posible_z;
  
  string dir_mono("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/config/mono.dat");
  
  reader.readDoubleCoordinates(dir_mono,mono);
  
  
  float alphaOcc = 0.2, alphaUnk = 0.8;
    PartialModelBase *partial_model = new PMVOctreeVasquez09(alphaOcc, alphaUnk);
    partial_model->setConfigFolder(config_folder);
    partial_model->setDataFolder(data_folder);
    partial_model->init();
    // Reads the sensor definition
    partial_model->readRays(rays_file);
  
  
  while (cov_t < 90){
    std::stringstream ss;
    std::stringstream tt;
    ss << indice;
    tt << t;
    
    cout << "\n\n Posicionamiento " << t << endl;
    //posicionamiento
    pos_actual = direccion_posicion + ss.str() + extension;
    reader.readDoubleCoordinates(pos_actual,p);
    
    
    cout << "\n\n Percepcion " << t << endl;
    //percepcion  (z)
    scan_actual = direccion + ss.str() + extension;
    reader.readDoubleCoordinates(scan_actual,z_t);
    
    
    cout << "\n\n Update" << t << endl;
    //update (M(z,t))
    partial_model->updateWithScan(scan_actual, pos_actual);
    pos_actual = data_octomap + tt.str() + ext_oct;
    partial_model->savePartialModel(pos_actual);
    nube_acumulada.insert(nube_acumulada.end(), z_t.begin(), z_t.end());
    //partial_model->
    //guardar pos acumulada
    
    
    cout << "\n\n Planificacion" << t << endl;
    //planificar ()
    cov_t = compareClouds(nube_acumulada,mono,gap);
    
    max_increment = 0;
    for (int j = 0; j<1312; j++){
      std::stringstream jj;
      jj << j;
      pos_actual = direccion + jj.str() + extension;
      reader.readDoubleCoordinates(pos_actual,posible_z);
      
      traslape = compareClouds(posible_z,nube_acumulada,gap);
      if (traslape >= 50)
      { 
	posible_acumulada.clear();
	posible_acumulada.insert(posible_acumulada.end(),nube_acumulada.begin(),nube_acumulada.end());
	posible_acumulada.insert(posible_acumulada.end(), posible_z.begin(), posible_z.end());
	cov_tmas1 = compareClouds(posible_acumulada,mono,gap);
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
    
    t++;
  }
  
  
  
  return 0;
  
  
}