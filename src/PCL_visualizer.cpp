/* \author Geoffrey Biggs  */
/* \edited by Olaya Álvarez*/


#include "DrawPC.h"
#include <string>
#include <sstream>

int
main (int argc, char** argv)
{

  // ------------------------------------
  // ----------Load point cloud----------
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  std::string input = "";

   // Get string with file name
   cout << "Name of the pcd file you want to open: ";
   getline(cin, input);

  pcl::io::loadPCDFile (input, *basic_cloud_ptr); //test_pcd.pcd

  // --------------------------------------------
  // ----------Call the class who draws----------
  // --------------------------------------------
  DrawPC visualizer(basic_cloud_ptr,
                    pcl::console::find_argument (argc, argv, "-n")>=0,  //normal
                    pcl::console::find_argument (argc, argv, "-s")>=0,  //shapes
                    pcl::console::find_argument (argc, argv, "-c")>=0,  //comparative
                    pcl::console::find_argument (argc, argv, "-v")>=0,  //voxel
                    pcl::console::find_argument (argc, argv, "-m")>=0); //triangle mesh
  //find_argument returns -1 if the argument is not available

  //--------------------------------------------------------------------------------------------------------
  // ------------------------------------------------Main loop----------------------------------------------
  //------Here are executed the functions which require a loop, like the mouse and keyboard callbacks ------
  //--------------------------------------------------------------------------------------------------------
  while (!visualizer.view->wasStopped ())
  {
      if (pcl::console::find_argument (argc, argv, "-i")>=0)
      {
          visualizer.view->registerKeyboardCallback (keyboardEventOccurred, (void*)&visualizer.view);
          visualizer.view->registerMouseCallback (mouseEventOccurred, (void*)&visualizer.view);
      }
    visualizer.view->spinOnce (1000);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
