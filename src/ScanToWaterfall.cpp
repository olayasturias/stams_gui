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
#include <pcl_ros/point_cloud.h>
/// Includes for cloud viewer
#include <pcl/visualization/cloud_viewer.h>
//For keyboard input
#include <string>
#include <sstream>
#include <typeinfo>


pcl::PointCloud<pcl::PointXYZ>  total_cloud;
//For adding fake height
int h = 0;

//! Class which reads a LaserScan message from a Profiling Sonar sensor and saves it as point cloud
/*! This class subscribes to a LaserScan topic from a Profiling Sonar sensor, converts it to a point
 * cloud topic (which is also published) and merges all the point cloud messages that it is processing
 * in a single point cloud. Finally, the result is saved in a PCD file. During the process, the point
 * cloud is visualized in a naive viewer.
 */
class ScanToWaterfall{

public:

  ros::NodeHandle n_;
  tf::TransformListener listener_;                                  /*!< used to transform the point cloud received to the desired frame */
  message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub_;   /*!< PointCloud suscriber */
  tf::MessageFilter<sensor_msgs::PointCloud> pc_notifier_;        /*!< used to execute the callback every time a LaserScan message is received */
  ros::Publisher waterfall_pub_;                                         /*!< ros publisher used to publish the point cloud */
  pcl::visualization::CloudViewer viewer;                           /*!< used to configure the viewer */

  ScanToWaterfall(ros::NodeHandle n) :
    n_(n),
    pc_sub_(n_, "tritech_profiler/scan", 10),
    pc_notifier_(pc_sub_,listener_, "world", 10),
    viewer("Profiling Viewer")
  {
     // Point cloud will be published in /ProfilingSonar_cloud
    //ros::Publisher waterfall_pub_ = n_.advertise<sensor_msgs::PointCloud>("/ProfilingSonar_cloud",1);
    ros::Publisher waterfall_pub_ = n_.advertise<sensor_msgs::PointCloud>("/ProfilingSonar_cloud", 1);

      //Callback that is executed every time a LaserScan message is received
    pc_sub_.registerCallback(
      boost::bind(&ScanToWaterfall::scanCallback, this, _1));
    pc_notifier_.setTolerance(ros::Duration(0.01));
  }

  //! Callback that is executed every time a LaserScan message is received
  /*! This callbacks transforms the tf frame from the given by the topic to the indicated in the constructor,
   * and the LaserScan data to PointCloud. Then, the new PointCloud is concatenated with the previous received,
   * an finally, the complete Point Cloud is published.
   */
  void scanCallback (const sensor_msgs::PointCloud::ConstPtr& scan_in)
  {

    typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
    CloudType::Ptr cloud (new CloudType);
    CloudType::PointType p;

    try
    {
        for (int i = 0; i < scan_in->points.size(); i++)
        {
                float x, y, z;
                x = scan_in->points[i].x;
                y = scan_in->points[i].y;
                z = scan_in->points[i].z;
                z += h;
                p.x = x; p.y = y; p.z = z;
                cloud->push_back(p);
        }
        h++;


    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    /*


    // I need to copy my point cloud to another one of the same format but as a Ptr!
    pcl::PointCloud<pcl::PointXYZ> aux = pcl::PointCloud<pcl::PointXYZ>(*cloud);
    std::cout << aux.points.x << std::endl;

    //std::cout << typeid(aux_ptrCloud).name() << std::endl;
    //std::cout << typeid(total_cloud).name() << std::endl;

    //Concatenating total_cloud and cloud with single scan
    total_cloud += aux;


    //Representing the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr draw_cloud (new pcl::PointCloud<pcl::PointXYZ>(total_cloud));
    viewer.showCloud(draw_cloud);

    // convert pointcloud to ros msg
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(total_cloud,cloud_msg);

    //std::cout << '*' << std::endl;

    //std::cout << total_cloud << std::endl;

    //std::cout << '*' << std::endl;

    //std::cout << cloud_msg << std::endl;
    */


    if( !waterfall_pub_ ) {
    ROS_WARN("Publisher invalid!");
    }

    waterfall_pub_.publish(scan_in);

  }
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud");

  ros::NodeHandle n;
  ScanToWaterfall pctowf(n);

  ros::spin();

  return 0;
}