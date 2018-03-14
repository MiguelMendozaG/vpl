#include <stdio.h>
#include <string.h>

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeBaseImpl.h>

using namespace std;

int main(){
  string filename ("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/6d pose/hinterstoisser/nubes/nbv_i/0/octomap_acumulado/octomap_0.ot.unk.ot");
  octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read (filename);
  
  
  if (tree){
    octomap::ColorOcTree* ot = dynamic_cast<octomap::ColorOcTree*>(tree);
    cout << "\n octree " << ot << endl;
    if (ot){
      cout << "\n octomap read" << endl;
      for (octomap::ColorOcTree::leaf_iterator it = ot->begin_leafs(), end = ot->end_leafs(); it!=end; ++it){
	cout << "\n Node center: " << it.getCoordinate();
	cout << "\n value: " << it->getValue() << "\n";
  }
      
    }
    else
      cout << "\n not read" << endl;
    
  }
  
  /*octomap::OcTree* oct = new octomap::OcTree(filename);
  cout << "\n octree " <<  oct << endl;
  for (octomap::OcTree::leaf_iterator it = oct->begin_leafs(), end = oct->end_leafs(); it!=end; ++it){
    cout << "\n Node center: " << it.getCoordinate();
    cout << "\n value: " << it->getValue() << "\n";
  }
  */
  return 0;
}