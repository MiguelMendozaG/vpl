#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZ PointType;
//http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors)#Local_descriptors

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
float support_size = 0.001f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// --------------
// -----Help-----
// --------------
void 
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-s <float>   support size for the interest points (diameter of the used sphere - "
                                                                  "default "<<support_size<<")\n"
            << "-o <0/1>     switch rotational invariant version of the feature on/off"
            <<               " (default "<< (int)rotation_invariant<<")\n"
            << "-h           this help\n"
            << "\n\n";
}

pcl::PointCloud<pcl::PointXYZ>::Ptr narf_points(std::string input){
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  std::string pcd_filename_indices = input;
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = pcd_filename_indices;
    if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
      cerr << "Was not able to open file \""<<filename<<"\".\n";
      //printUsage (argv[0]);
      exit;
    }
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);
    //std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
    //if (pcl::io::loadPCDFile (far_ranges_filename.c_str (), far_ranges) == -1)
    //  std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
  }
  else
  {
    setUnseenToMaxRange = true;
    cout << "\nNo *.pcd file given\n\n";
    exit;
 
  }
  
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
  std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
  
  // -------------------------------------
  // -----Show keypoints in 3D viewer-----
  // -------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  keypoints.points.resize (keypoint_indices.points.size ());
  for (size_t i=0; i<keypoint_indices.points.size (); ++i){
    keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
    //cout << "Puntos: " << range_image.points[keypoint_indices.points[i]].getVector3fMap() << endl;
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
  //cout << " point: " << cloud_one->points.size() << endl;
  return cloud_one;
  
}

// --------------
// -----Main-----
// --------------
int 
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr NARF1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr NARF2 (new pcl::PointCloud<pcl::PointXYZ>);
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    setUnseenToMaxRange = true;
    cout << "Setting unseen values in range image to maximum range readings.\n";
  }
  if (pcl::console::parse (argc, argv, "-o", rotation_invariant) >= 0)
    cout << "Switching rotation invariant feature version "<< (rotation_invariant ? "on" : "off")<<".\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
    cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
  }
  if (pcl::console::parse (argc, argv, "-s", support_size) >= 0)
    cout << "Setting support size to "<<support_size<<".\n";
  if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
    cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
  angular_resolution = pcl::deg2rad (angular_resolution);
  
  
  // ------------------------------------------------------------------
  // -----Read pcd file or create example point cloud if not given-----
  // ------------------------------------------------------------------
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
  //std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  //std::string file_cloud("/home/miguelmg/Documents/CIDETEC/semestre 2/vision 3d/proyecto/pcl test/nubes/imagen-0.pcd");
  
  std::string filename2;
  
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  //std::vector<int> pcd_filename_output =  pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  int output = pcl::console::parse_argument (argc, argv, "-out", filename2);
  std::string filename = argv[pcd_filename_indices[0]];
  
  
  NARF1 = narf_points(filename);
  //cout << " point " << NARF1->points.size() << endl;
  //cout << " point x(0) " << NARF1->points[0].x << endl;
  //cout << "narf points " << NARF1 << endl;
  std::cout << "input: " << filename << endl;
  std::cout << "output: " << filename2 << endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::savePCDFile(filename2 + "narf_points.pcd", *NARF1);

}


