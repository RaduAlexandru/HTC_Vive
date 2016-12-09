#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <chrono>
#include <thread>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/filters/fast_bilateral.h>

//#include <OGRE/OgreSceneManager.h>
//#include <OGRE/OgreRenderSystem.h>
//#include <OGRE/OgreRenderWindow.h>
//#include <OGRE/OgreHardwarePixelBuffer.h>
//#include <OGRE/RenderSystems/GL/OgreGLTexture.h>
//#include <OGRE/RenderSystems/GL/OgreGLRenderTexture.h>
//#include <OGRE/OgreTexture.h>
//#include <OGRE/OgreCamera.h>
//#include <OGRE/OgreSceneNode.h>
//#include <OGRE/OgreRenderTargetListener.h>
//#include <OGRE/RenderSystems/GL/OgreGLContext.h>
//#include <OGRE/OgreRoot.h>
//
////these stuff may not be needded TODO:check which ones are needed
//#include <OGRE/OgreLogManager.h>
//#include <OGRE/OgreViewport.h>
//#include <OGRE/OgreConfigFile.h>
//#include <OGRE/OgreEntity.h>
//#include <OGRE/OgreWindowEventUtilities.h>
//#include <OGRE/OgreMeshManager.h>


#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLModels.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
//#include <Vrui/VRWindow.h>

//#include <rviz/display.h>
//#include <rviz/ogre_helpers/render_widget.h>
//#include <rviz/ogre_helpers/render_system.h>
//#include <rviz/render_panel.h>
//#include <rviz/ogre_helpers/qt_ogre_render_window.h>
//#include <rviz/display_context.h>
//#include <rviz/window_manager_interface.h>
//#include <rviz/visualization_manager.h>
//#include <rviz/frame_manager.h>
//#include <rviz/display_group.h>


//#include <QObject>

//class Vrui::VRWindow;
#include <Vrui/VRWindow.h>

#include <GL/glx.h>
//#include <Qt/qx11info_x11.h>
#include <X11/Xlib.h>

//VTK
#include <vtkActor.h>
#include <vtkCamera.h>
#include <ExternalVTKWidget.h>
#include <vtkExternalOpenGLRenderer.h>
#include <vtkExternalOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkCubeSource.h>
#include <vtkSphereSource.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPNGReader.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkRendererCollection.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkPLYWriter.h>
#include <vtkTriangle.h>
#include <vtkFloatArray.h>
#include <vtkCleanPolyData.h>
#include <vtkQuadricDecimation.h>
#include <vtkImageImport.h>
#include <vtkTexture.h>
#include <vtkImageMapper.h>
#include <vtkActor2D.h>
#include <vtkImageSlice.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkImageCanvasSource2D.h>

#include <vtkExternalOpenGLCamera.h>
#include <vtkExternalOpenGLRenderer.h>
#include <vtkExternalOpenGLRenderWindow.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

typedef std::chrono::high_resolution_clock Clock;
using namespace sensor_msgs;
using namespace message_filters;

class ViveRviz :  public Vrui::Application {

//	Q_OBJECT
	public:
	ViveRviz(int& argc,char**& argv);
	
	virtual void display(GLContextData& contextData)const ;
	virtual void resetNavigation(void);
	virtual void frame();

	bool m_initialized;
	mutable boost::mutex m_display_mtx;

	vtkSmartPointer<vtkExternalOpenGLRenderer> m_vtk_renderer;
	vtkSmartPointer<vtkExternalOpenGLCamera> m_vtk_camera;
	vtkSmartPointer<vtkExternalOpenGLRenderWindow> m_vtk_window;


	vtkSmartPointer<vtkPolyData> m_mesh;
	vtkSmartPointer<vtkPolyDataMapper> m_mapper;
	vtkSmartPointer<vtkActor> m_actor;
	vtkSmartPointer<vtkTexture> m_texture;

	vtkSmartPointer<vtkImageImport> m_imageImport;

	vtkSmartPointer<vtkImageActor> m_img_actor;



	Eigen::Matrix<double, 3, 4>  proj_matrix;
	bool proj_matrix_set=false;
	bool m_texture_initialized=false;
	

	void meshCallback(const visualization_msgs::Marker::ConstPtr& msg);
	void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg );
	void kinectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& cloud_msg);
	void chatterCallback(const std_msgs::String::ConstPtr& msg);
	void startROSCommunication();
	void getCameraInfo();
	

};
