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
/// Includes for cloud viewer
#include <pcl/visualization/cloud_viewer.h>
//For keyboard input
#include <string>
#include <sstream>


pcl::PointCloud<pcl::PointXYZ>  total_cloud;

  std::string topic = "";

//! Class which reads a LaserScan message from a Profiling Sonar sensor and saves it as point cloud
/*! This class subscribes to a LaserScan topic from a Profiling Sonar sensor, converts it to a point
 * cloud topic (which is also published) and merges all the point cloud messages that it is processing
 * in a single point cloud. Finally, the result is saved in a PCD file. During the process, the point
 * cloud is visualized in a naive viewer.
 */
class ScanToWaterfall{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;                       /*!< provides the methods to transform from LaserScan to PointCloud */
  tf::TransformListener listener_;                                  /*!< used to transform the point cloud received to the desired frame */
  message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub_;   /*!< LaserScan suscriber */
  tf::MessageFilter<sensor_msgs::PointCloud> pc_notifier_;        /*!< used to execute the callback every time a LaserScan message is received */
  ros::Publisher waterfall_pub_;                                         /*!< ros publisher used to publish the point cloud */
  pcl::visualization::CloudViewer viewer;                           /*!< used to configure the viewer */

  ScanToWaterfall(ros::NodeHandle n) :
    n_(n),
    pc_sub_(n_, topic, 10),
    pc_notifier_(pc_sub_,listener_, "shaft", 10),
    viewer("Profiling Viewer")
  {
      //Callback that is executed every time a LaserScan message is received
    pc_sub_.registerCallback(
      boost::bind(&ScanToWaterfall::scanCallback, this, _1));
    pc_notifier_.setTolerance(ros::Duration(0.01));

    // Point cloud will be published in /ProfilingSonar_cloud
    waterfall_pub_ = n_.advertise<sensor_msgs::PointCloud>("/ProfilingSonar_cloud",1);
  }

  //! Callback that is executed every time a LaserScan message is received
  /*! This callbacks transforms the tf frame from the given by the topic to the indicated in the constructor,
   * and the LaserScan data to PointCloud. Then, the new PointCloud is concatenated with the previous received,
   * an finally, the complete Point Cloud is published.
   */
  void scanCallback (const sensor_msgs::PointCloud::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    pcl::PointCloud<pcl::PointXYZ>  aux_cloud;
    try
    {
        for (int i = 0; i < scan_in->points.size(); i++)
        {
                aux_cloud.x = scan_in->points[i].x;
                aux_cloud.y = scan_in->points[i].y;
                //x = scan_in->points[i].x;
                //std::cout << x;
                //y = scan_in->points[i].y;
                //scan_in->points[i].z = i;
                for (int k = 0; k< scan_in->points[i].size(); k++)
                    aux_cloud.z = 0;

        }

    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    //Convert sensor_msgs PointCloud2 to pcl::PointCloud<pcl::pointxyz>
    //pcl::PointCloud<pcl::PointXYZ>  pcl_cloud, aux;
                  //input | output//
    //pcl::fromROSMsg(cloud, pcl_cloud);

    //Concatenating total_cloud and pcl_cloud
    //total_cloud += pcl_cloud;

    //Representing the point cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr draw_cloud (new pcl::PointCloud<pcl::PointXYZ>(total_cloud));

    //viewer.showCloud(draw_cloud);
    //viewer.showCloud(scan_in);


    //waterfall_pub_.publish(cloud);

  }
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud");


  /// Creating the object for doing the transformation
  cout << "Write topic you want to subscribe to (/g500/ProfilingSonar or /g500/multibeam?): ";
  getline(cin, topic);

  ros::NodeHandle n;
  ScanToWaterfall pctowf(n);

  ros::spin();

  return 0;
}