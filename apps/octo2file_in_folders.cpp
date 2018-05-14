#include <stdio.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream> //library for stringstream

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OccupancyOcTreeBase.h>


using namespace std;

octomap::point3d x_min = {-0.083216,-0.083216,-0.083216};  /// this values are set according eith the partialmodel file, it is refered to the bounding box
octomap::point3d x_max = {0.083216,0.083216,0.083216};

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

int main(){
  string input_folder ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/modelo3/nbv/");
  string octree_file;
  string octree_ot;
  string octree_folder;
  for (int i = 945; i < 1312; i+=5){
     stringstream n_folder;
     n_folder << i;
     octree_folder = input_folder + n_folder.str() + "/octo_acum/octomap_acum_";
     for (int j = 1; j <= 21; j++){
	stringstream n_octree;
	n_octree << j;
	octree_file = octree_folder + n_octree.str() + ".txt";
	octree_ot = octree_folder + n_octree.str() + ".ot";
	////////////////////////////
	//Open .txt files and remove if exists
	if (FILE *file = fopen(octree_file.c_str(), "r")){
	  cout << "\nfile " << octree_file <<" found"<< endl;
	  if (remove(octree_file.c_str()) != 0)
	    perror ("\nError deletinfg file");
	  //fclose(file);
	  ////////////////////////////
	  //Open .ot files and process them to obtain the .txt
	  file_octomap(octree_ot, octree_file);
	}
	else
	  cout << "****File not found" << octree_file << endl;
	
	
    }
  }
  
  return 0;
}