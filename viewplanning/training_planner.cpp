#include "training_planner.h"

TrainingPlanner::TrainingPlanner(RobotSensor* rs, PartialModelBase* pm): WorkspaceNBVPlanner(rs, pm)
{

}

 /*Leer la nube de puntos acumulada
     Para cada una de las vistas, leer la imagen de Hintertoisse
     Calcular el porcentaje de incremento
     Regresar la que tiene un mayor incremento (con un traslape en la nube de puntos acumulada)*/
bool TrainingPlanner::planNBV(ViewStructure &v)
{
  vpFileReader reader;
  string acumulada(dataFolder+"/object_pts.dat");
  string direccion("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/absolutas/imagen-");
  string extension(".dat");
  string scan_actual;
  vector< vector<double> > pos_actual;
  vector< vector<double> > pos_acumulada;
  vector< vector<double> > pos_bunny;
  
  //int a = 10;
  
  //ss << a;
  //string str = ss.str();
  
  //comparar acumulada con el total
  string bunny("/home/miguelmg/repositorios/vpl/data_example/FreeFlyer/config/mono.dat");
  double coverage_t = compareFilePoints(acumulada, bunny, 0.005);
  double cov_tmas1 = 0;
  
  reader.readDoubleCoordinates(bunny, pos_bunny);
  
  int max_increment = 0;
  int max_intertoise = 0;      
  
  ViewList::iterator it;
  it = candidateViews.begin();
  
  for (int i=0; i<=2; i++){
    std::stringstream ss;
    cout << "\n\nComparando modelo" << endl;
    ss << i;
    scan_actual = direccion + ss.str() + extension;
    
    reader.readDoubleCoordinates(scan_actual,pos_actual);
    reader.readDoubleCoordinates(acumulada, pos_acumulada);
    //pos_actual.
    
    pos_acumulada.insert(pos_acumulada.end(), pos_actual.begin(), pos_actual.end());
    
    cov_tmas1 = compareClouds(pos_acumulada , pos_bunny, 0.005);
       
    double increment = cov_tmas1 - coverage_t;
    
    if (increment > max_increment)
    {
      cout << "encontre que la imagen " << i << " incrementa en " << increment << " porciento " << endl; 
      max_intertoise = i;
      max_increment = increment;      
    }
    
    it->eval = increment;
  }   
  
//  candidateViews.sortHighToLow();
  v = candidateViews.getBestView();
  return true;
}


double TrainingPlanner::compareClouds(std::vector< std::vector< double > > a, std::vector< std::vector< double > > reference,  double gap )
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



bool TrainingPlanner::init()
{
    NBVPlanner::init();

     std::cout << "\n------ WorkspaceNBVPlanner Configuration (Training Planner)------ " << std::endl;
  
  std::string config_file(configFolder);
  config_file.append("/");
  config_file.append("plannerConfig.ini");
  
  mrpt::utils::CConfigFile parser;
  ASSERT_FILE_EXISTS_(config_file);
  parser.setFileName(config_file);
  
  double x1,y1,z1,x2,y2,z2;
  partialModel->getOBB(x1,y1,z1,x2,y2,z2);
  
  objectCenter.clear();
  objectCenter.resize(3);
  
  objectCenter[0] = (x2+x1)/2;
  objectCenter[1] = (y2+y1)/2;
  objectCenter[2] = (z2+z1)/2;
  std::cout << "View sphere center: [" << objectCenter[0] << ", " << objectCenter[1] << ", " << objectCenter[2] << "]" << std:: endl;
  cx = objectCenter[0];
  cy = objectCenter[1];
  cz = objectCenter[2];
  radius = 0.40;
  

   string input("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/posicion/pose/imagen_origines.dat");
   vpFileReader reader;
   std::vector<std::vector<double>> posiciones;
   reader.readDoubleCoordinates(input,posiciones);
   vistas_dirigidas(posiciones,candidateViews);
  
  //generator.getViews(candidateViews); 
  //std::cout << candidateViews.size() << " views were generated" << std::endl;
  
  // This step is needed because the rays depend on the sensor pose with respect of the robot
  robotWithSensor->getViewsFromComfigurations(candidateViews);
  
  
  
  
  return true;

}

void TrainingPlanner::vistas_dirigidas(vector< vector< double > > points, ViewList& views)
{
  views.clear();
  ViewStructure view;
  
  std::vector< std::vector<double> >::iterator point_it;
  
  /// unit sphere point
  std::vector<double> usp(3); 
  
  /// view sphere point
  std::vector<double> vsp(3);
  
  std::vector<double> pointing_v(3);
  
  std::vector<double> coordinates(6);
  
  double yaw, pitch, roll;
  double norm;
  
  for(point_it = points.begin(); point_it != points.end(); point_it++){
    if(point_it->size() != 3){
      std::cout << "Error in points " << std::endl;
      exit(0);
    }
    
    usp[0] = (*point_it)[0];
    usp[1] = (*point_it)[1];
    usp[2] = (*point_it)[2];
    
    // expander el punto al radio indicado; 
    usp[0] = radius * usp[0];
    usp[1] = radius * usp[1];
    usp[2] = radius * usp[2];
        

    vsp[0] = cx + usp[0];
    vsp[1] = cy + usp[1];
    vsp[2] = cz + usp[2];
    
    // calcular el vector que apunta al objeto
    pointing_v[0] = cx - vsp[0];
    pointing_v[1] = cy - vsp[1];
    pointing_v[2] = cz - vsp[2];
    
    // calcular los angulos de rotación
    yaw = atan2(pointing_v[1], pointing_v[0]);
    
    norm = sqrt( pow(pointing_v[0],2) + pow(pointing_v[1],2) + pow(pointing_v[2],2) );
    //norm = radio;
    pitch = asin( pointing_v[2] / norm);
    pitch = -pitch; // I did this because the positive angles lie before the x-y plane.  
    roll = 0;
   
    // determinar la configuración
    view.w[0] = (double) vsp[0];
    view.w[1] = (double) vsp[1];
    view.w[2] = (double) vsp[2];
    view.w[3] = (double) (yaw);
    view.w[4] = (double) (pitch);
    view.w[5] = (double) (roll);
    
    view.q = view.w;
    // determinar la matriz de transformación homogenea
    view.setPose(view.w[0], view.w[1], view.w[2], view.w[3], view.w[4], view.w[5]);
//     mrpt::poses::CPose3D pose(view.w[0], view.w[1], view.w[2], view.w[3], view.w[4], view.w[5]);
//     mrpt::math::CMatrixDouble44 htm_p;
//     pose.getHomogeneousMatrix(htm_p);
//   
//     view.HTM(0,0) = htm_p(0,0);
//     view.HTM(0,1) = htm_p(0,1); 
//     view.HTM(0,2) = htm_p(0,2); 
//     view.HTM(0,3) = htm_p(0,3);
//     
//     view.HTM(1,0) = htm_p(1,0); 
//     view.HTM(1,1) = htm_p(1,1); 
//     view.HTM(1,2) = htm_p(1,2);
//     view.HTM(1,3) = htm_p(1,3);
//     
//     view.HTM(2,0) = htm_p(2,0);
//     view.HTM(2,1) = htm_p(2,1);
//     view.HTM(2,2) = htm_p(2,2);
//     view.HTM(2,3) = htm_p(2,3);
//     
//     view.HTM(3,0) = htm_p(3,0);
//     view.HTM(3,1) = htm_p(3,1);
//     view.HTM(3,2) = htm_p(3,2);
//     view.HTM(3,3) = htm_p(3,3);
    
    views.push_back(view);
  }
}

