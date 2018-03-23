#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OccupancyOcTreeBase.h>


using namespace std;

int main(){
  string filename ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/nbv/0/octo_acum/octomap_acum_1.ot");
  string filename_output ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo2/nbv/0/octo_acum/octomap_txt.txt");
  octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read (filename);
  double occu_voxels = 0, free_voxels = 0;
  octomap::point3d p;
  int m=0;
  //if (tree){
    octomap::ColorOcTree* ot = dynamic_cast<octomap::ColorOcTree*>(tree);
    //cout << "\n octree " << ot << endl;
    if (ot){
      ofstream output_file(filename_output, ios::app);
      cout << "\n octomap read" << endl;
      octomap::point3d x_min = {-0.11,-0.11,-0.11};
      octomap::point3d x_max = {0.11,0.11,0.11};
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
	}/*
	if (ot->isNodeOccupied(*it)){
	  occu_voxels++;
	  coord = it.getCoordinate();
	  //cout << "\n bouning box " << ot->getMetricMax() << endl;
	  //cout << "\n coord " << coord << endl;
	  //cout << "\n occupancy " << it->getOccupancy();
	  if (ot->inBBX(p)){
	    node_centers.push_back(p);
	    sizes.push_back(it.getSize());
	    //cout << "\n Node center: " << it.getCoordinate();
	    //cout << "\n value: " << it->getValue() << "\n";
	    inbbx_volxels++;
	  }
	}
	else{
	  if (ot->inBBX(p)){
	    node_centers.push_back(p);
	    sizes.push_back(it.getSize());
	    //cout << "\n Node not occu: " << it.getCoordinate();
	    //cout << "\n value: " << it->getValue() << "\n";
	    free_voxels++;
	  }
	  // cout << "\n free thresh " << it->getOccupancy();
	}*/
      m++;
      }
   }
   else
     cout << "\n not read" << endl;
    
   cout << "\n\t occupied voxels = " << occu_voxels << endl;
   cout << "\n\t free voxels = " << free_voxels << endl;

  //}
  return 0;
}