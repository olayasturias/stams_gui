#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <iostream>
/// Includes for PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
/// Includes for doing stuff when shooting down
#include <signal.h>
#include <ros/xmlrpc_manager.h>
/// Includes for cloud viewer
#include <pcl/visualization/cloud_viewer.h>
//For keyboard input
#include <string>
#include <sstream>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

pcl::PointCloud<pcl::PointXYZ>  total_cloud;

  std::string topic = "";

//! Class which reads a LaserScan message from a Profiling Sonar sensor and saves it as point cloud
/*! This class subscribes to a LaserScan topic from a Profiling Sonar sensor, converts it to a point
 * cloud topic (which is also published) and merges all the point cloud messages that it is processing
 * in a single point cloud. Finally, the result is saved in a PCD file. During the process, the point
 * cloud is visualized in a naive viewer.
 */
class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;                       /*!< provides the methods to transform from LaserScan to PointCloud */
  tf::TransformListener listener_;                                  /*!< used to transform the point cloud received to the desired frame */
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;   /*!< LaserScan suscriber */
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;        /*!< used to execute the callback every time a LaserScan message is received */
  ros::Publisher scan_pub_;                                         /*!< ros publisher used to publish the point cloud */
  pcl::visualization::CloudViewer viewer;                           /*!< used to configure the viewer */

  //! Class constructor
  /*! \param n_ ros Node handler
   *  \param laser_sub_ LaserScan suscriber to topic indicated in console
   *  \param laser_notifier tf frame selection
   *  \param viewer viewer configuration
   */
  LaserScanToPointCloud(ros::NodeHandle n) :
    n_(n),
    laser_sub_(n_, topic, 10),
    laser_notifier_(laser_sub_,listener_, "world", 10),
    viewer("Profiling Viewer")
  {/*
      // Select if you want to change LaserScan topic
      std::string input = "";
      cout << "Do you want to change topic /g500/ProfilingSonar? (y/n): ";
      getline(cin, input);
      if (strcmp("y",input)==0 || strcmp("Y",input)==0)
      {
          laser_sub_(n_, input, 10);
      }
      else
          std::cout << "Suscribing to /g500/ProfilingSonar" << std::endl;*/

      //Callback that is executed every time a LaserScan message is received
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));

    // Point cloud will be published in /ProfilingSonar_cloud
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/ProfilingSonar_cloud",1);
  }

  //! Callback that is executed every time a LaserScan message is received
  /*! This callbacks transforms the tf frame from the given by the topic to the indicated in the constructor,
   * and the LaserScan data to PointCloud. Then, the new PointCloud is concatenated with the previous received,
   * an finally, the complete Point Cloud is published.
   */
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "world",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    //Convert sensor_msgs PointCloud2 to pcl::PointCloud<pcl::pointxyz>
    pcl::PointCloud<pcl::PointXYZ>  pcl_cloud, aux;
                  //input | output//
    pcl::fromROSMsg(cloud, pcl_cloud);

    //Concatenating total_cloud and pcl_cloud
    total_cloud += pcl_cloud;

    //Representing the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_cloud (new pcl::PointCloud<pcl::PointXYZ>(total_cloud));

    viewer.showCloud(draw_cloud);

    scan_pub_.publish(cloud);

  }
};




// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

//! Callback executed when the node is shut down
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud", ros::init_options::NoSigintHandler);

  /// Configuration of customized shut down
  signal(SIGINT, mySigIntHandler);
  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  /// Creating the object for doing the transformation
  cout << "Write topic you want to subscribe to (/tritech_profiler/singlescan or /g500/ProfilingSonar or /g500/multibeam?): ";
  getline(cin, topic);

  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  // Do our own spin loop
  while (!g_request_shutdown)
  {
    // Stay here while the node is working

    ros::spinOnce();
    usleep(100000);
  }

  // Other pre-shutdown tasks should be included here

  //Get name of saved file
   std::string input = "";
   cout << "Name of the pcd file you want to save: ";
   getline(cin, input);
   cout << " " << input << endl << endl;

  pcl::io::savePCDFileASCII (input, total_cloud);

  ros::shutdown();
  return 0;
}
