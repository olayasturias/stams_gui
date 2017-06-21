//! Class for representing and coloring a given PointCloud
/*! autor: Olaya Alvarez
 * it includes some other functions to draw normals, figures, etc
 */

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <math.h>
#include <pcl/segmentation/supervoxel_clustering.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
//For triangulation
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
//For saving vtk
#include <pcl/io/vtk_io.h>
//For drawing vtk
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkStructuredGridReader.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkGenericDataObjectReader.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>


// Functions for Keyboard events.
// They're out of the class since they need to be constantly executed.

unsigned int text_id = 0;

//! Callback function which is called every time the keyboard is pressed
/*! If the "r" key is pressed, all the text added in the visualizator is deleted */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

//! Callback function which is called every time the mouse is pressed
/*! If the mouse is pressed, it adds text to the visualizator where it was pressed */
void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}


//! This class receives a grey point cloud and adds it color
/*! Since it would be cylinder-shaped, adds it different colors depending on the distance to the axis
 * It also receives some bool arguments to add some drawing options:
 * \param normal to draw normals to the surface
 * \param shapes to draw shapes on the surface
 * \param comparation, to draw two point clouds with different normals
 * \param view allows to create a loop that stops the node when the visualizer is closed
 */
class DrawPC{
public:
    int h;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colorcloud;         /*!< Pointcloud where the colored cloud will be saved */
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 ;          /*!< Pointcloud with normals (in a radius of 0.05) drawn on it */
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 ;          /*!< Pointcloud with normals (in a radius of 0.7) drawn on it */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view;  /*!< variable whit the viewer configuration */

    //! Constructor.
    /*! Here, the cloud is colored, and then the special parameters are called according with the
     *  configuration selected
     */
    DrawPC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
           bool normal,
           bool shapes,
           bool comparation,
           bool voxelcloud,
           bool mesh):
        colorcloud(new pcl::PointCloud<pcl::PointXYZRGBA>),
        cloud_normals1(new pcl::PointCloud<pcl::Normal>),
        cloud_normals2(new pcl::PointCloud<pcl::Normal>)
    {
        // Add color to cloud, save in variable colorcloud
        PaintCloud(cloud);
        // Calculate normals with radius 0.05 and 0.1
        CalcNormals();
        if (mesh)
                view = MeshTriangulation(cloud);
        else
                view = SelectVisualizer(normal, shapes, comparation,voxelcloud);


    }//DrawPC

    //! Funcion for adding color to each point according to its distance from the center
    // We cant manually change the point cloud colors
    // So we create a new point cloud with the same values of the original cloud and add it colors
    void PaintCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointXYZRGBA colorpoint;
        pcl::PointCloud<pcl::PointXYZ>::iterator i;

        colorcloud->width = cloud->width;
        colorcloud->height = cloud->height;

        float dmax = 0;
        float dmin = 1000;

        for (i = cloud->points.begin(); i < cloud->points.end(); i++){
            if (sqrt(i->x*i->x+i->y/i->y) >= dmax)
                dmax = sqrt(i->x*i->x+i->y/i->y);
            if (sqrt(i->x*i->x+i->y/i->y) <= dmax)
                dmin = sqrt(i->x*i->x+i->y/i->y);

        }
        dmax *= 10;
        dmin *= 10;

        int j = 0,k = 0;

        for (i = cloud->points.begin(); i < cloud->points.end(); i++){
            // copy the point
            colorpoint.x = i->x;
            colorpoint.y = i->y;
            colorpoint.z = i->z;

            // add it a color according to the distance to the axis
            float dx = sqrt(colorpoint.x*colorpoint.x+colorpoint.y*colorpoint.y);
            //std::cout << "dx = " << dx*10 << std::endl;
            dx *= 10;
        /*    colorpoint.r = 255*(dx-dmin)/(dmax-dmin);
            colorpoint.g = 255*(dx-dmin)/(dmax-dmin);
            colorpoint.b = 255*(dx-dmin)/(dmax-dmin);*/

            uint8_t r,g,b;
            if (dx<(1000))
            {
                 r = 255*(dx-dmin)/(dmax-dmin);
                 g = 255*1.8*(dx-dmin)/(dmax-dmin);
                 b = 0;
                 j++;
            }
            else
            {
                r = 0;
                g = 0;
                b = 255*(dx-dmax)/(dmin-dmax);
                k++;
            }

            // pack r/g/b into rgb

            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            colorpoint.rgb = *reinterpret_cast<float*>(&rgb);


            // add point to colored cloud
            colorcloud->push_back(colorpoint);
        }//for

        std::cout << "dx<dmax/4 " << j << std::endl;
        std::cout << "else " << k << std::endl;
    }//PaintCloud

    //! Function which divides the Point cloud in voxels
    boost::shared_ptr<pcl::visualization::PCLVisualizer> VoxelizeCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
    {
        float voxel_resolution = 0.08f;
        float seed_resolution = 0.1f;
        float color_importance = 0.2f;
        float spatial_importance = 0.4f;
        float normal_importance = 1.0f;


        pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution,false);

        super.setInputCloud (cloud);
        super.setColorImportance (color_importance);
        super.setSpatialImportance (spatial_importance);
        super.setNormalImportance (normal_importance);

 /*       if (true)
          super.setUseSingleCameraTransform (false);

        pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_res, seed_res);
        super.setUseSingleCameraTransform(false);
        super.setInputCloud (cloud);
        super.setColorImportance (color_importance);
        super.setSpatialImportance (spatial_importance);
        super.setNormalImportance (normal_importance);*/

        std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;

        // Extract
        pcl::console::print_highlight ("Extracting supervoxels\n");
        super.extract (supervoxel_clusters);
        pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());


        // Draw
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);

        // Add colors
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr superVcloud;
        superVcloud = super.getColoredVoxelCloud ();

        viewer->addPointCloud (superVcloud, "voxelized cloud");

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(superVcloud);


        return(viewer);


    } //VoxelizeCloud

    //! Function which calculate surface normals in two different radius and saves them in two different variables
    void CalcNormals()
    {
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
        ne.setInputCloud (colorcloud);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
        ne.setSearchMethod (tree);
        // -----Calculate surface normals with a search radius of 0.05-----
        ne.setRadiusSearch (0.05);
        ne.compute (*cloud_normals1);
        // -----Calculate surface normals with a search radius of 0.7-----
        ne.setRadiusSearch (0.7);
        ne.compute (*cloud_normals2);
    }

    //! Function which opens 3D viewer and adds RGB point cloud into it
    boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbDraw (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
    {

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        return (viewer);
    }

    //! Function which opens 3D viewer and adds it RGB point cloud with normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> normalDraw (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
                                                                     pcl::PointCloud<pcl::Normal>::ConstPtr normals)
    /// -----Open 3D viewer and add point cloud and normals-----
    {
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      return (viewer);
    }

    //! Function which opens 3D viewer with RGB point cloud and draws some 3D shapes.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesDraw (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
    {
      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

      viewer->initCameraParameters ();

      //------------------------------------
      //-----Add shapes at cloud points-----
      //------------------------------------
      viewer->addLine<pcl::PointXYZRGBA> (cloud->points[0],
                                         cloud->points[cloud->size() - 1], "line");
      viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

      //---------------------------------------
      //-----Add shapes at other locations-----
      //---------------------------------------
      pcl::ModelCoefficients coeffs;
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      viewer->addPlane (coeffs, "plane");
      coeffs.values.clear ();
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (5.0);
      viewer->addCone (coeffs, "cone");
      viewer->addCoordinateSystem (1.0);

      return (viewer);
    }

    //! Function which opens two 3D viewers with the two different kind of normals drawn on them
    boost::shared_ptr<pcl::visualization::PCLVisualizer> comparativeDraw (
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
            pcl::PointCloud<pcl::Normal>::ConstPtr normals1,
            pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
    {
      // --------------------------------------------------------
      // -----Open 3D viewer and add point cloud and normals-----
      // --------------------------------------------------------
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->initCameraParameters ();

      int v1(0);
      viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
      viewer->setBackgroundColor (0, 0, 0, v1);
      viewer->addText("Radius: 0.05", 10, 10, "v1 text", v1);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud1", v1);

      int v2(0);
      viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
      viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
      viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud, 0, 255, 0);
      viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud2", v2);

      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
      viewer->addCoordinateSystem (1.0);

      viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
      viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

      return (viewer);
    }// comparative

    //! Function which triangulates the surface to obtain a surface from the point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> MeshTriangulation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    // Triangulates the point cloud and creates a mesh
    // The input cloud must be XYZ only
    {
        //----------------------------------------
        // ----------- Normal estimation ---------
        //----------------------------------------
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);
        n.setInputCloud (cloud);
        n.setSearchMethod (tree);
        n.setKSearch (20);
        n.compute (*normals);

        //------------------------------------------------------------
        // ----------- Concatenate the XYZ and normal fields ---------
        //------------ cloud_with_normals = cloud + normals ----------
        //------------------------------------------------------------
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
        //----------------------------------------
        // ---------- Create search tree ---------
        //----------------------------------------
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);
        //----------------------------------------
        // ---------- Initialize objects ---------
        //----------------------------------------
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;
        //------------------------------------------------------------------------
        // ---------- Set the maximum distance between connected points  ---------
        //---------------------(maximum edge length)------------------------------
        //---------------------- & typical values --------------------------------
        gp3.setSearchRadius (250);

        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (100);
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(0); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);
        //----------------------------------------
        // -------------- Get result -------------
        //----------------------------------------
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);
        //std::cout << "triangles " << triangles << std::endl;
        //------------------------------------------------
        // -------------- Vertex information -------------
        //------------------------------------------------
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();
        //------------------------------------------------
        // ----------------- Save in vtk -----------------
        //------------------------------------------------


        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (255,255, 255);
        viewer->addPolygonMesh(triangles,"triangles",0);
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();

        return (viewer);





        // Get string with file name
        std::string input = "";
        cout << "Name of the vtk file with triangles mesh: ";
        getline(cin, input);
        pcl::io::saveVTKFile (input, triangles);

        //DrawVTK(input);
    }//MeshTriangulation



    //! Function which selects the point cloud representation mode according to the parameters given by console
    boost::shared_ptr<pcl::visualization::PCLVisualizer> SelectVisualizer(bool normal,
                                                                          bool shapes,
                                                                          bool comparation,
                                                                          bool voxelcloud)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        if(normal)
            viewer = normalDraw(colorcloud, cloud_normals2);
        else if(shapes)
            viewer = shapesDraw(colorcloud);
        else if (comparation)
            viewer = comparativeDraw(colorcloud,cloud_normals1,cloud_normals2);
        else if (voxelcloud)
            viewer = VoxelizeCloud(colorcloud);
        else
            viewer = rgbDraw(colorcloud);


        return(viewer);

    }//SelectVisualizer

};
