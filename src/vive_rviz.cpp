/*
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>
#include <OGRE/RenderSystems/GL/OgreGLRenderTexture.h>

#include <GL/gl.h>
#include <GL/GLMaterialTemplates.h>
#include <GL/GLModels.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>






class VruiDemoSmall:public Vrui::Application
	{
	public:
	VruiDemoSmall(int& argc,char**& argv);
	
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	};


VruiDemoSmall::VruiDemoSmall(int& argc,char**& argv)
	:Vrui::Application(argc,argv)
	{
	}

void VruiDemoSmall::display(GLContextData& contextData) const
	{
	glPushMatrix();
	
	glTranslated(-5.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.5f,1.0f));
	glDrawCube(7.5f);
	
	glTranslated(10.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,1.0f));
	glDrawSphereIcosahedron(4.5f,6);
	
	glPopMatrix();
	}

void VruiDemoSmall::resetNavigation(void)
	{
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12));
	}

VRUI_APPLICATION_RUN(VruiDemoSmall)*/


#include "vive_rviz.h"
#include "Vrui/VRWindow.h"
#include "Vrui/Viewer.h"




ViveRviz::ViveRviz(int& argc,char**& argv) :Vrui::Application(argc,argv),
	m_vtk_camera(vtkSmartPointer<vtkExternalOpenGLCamera>::New()),
	m_vtk_renderer( vtkSmartPointer<vtkExternalOpenGLRenderer>::New() ),
	m_vtk_window( vtkExternalOpenGLRenderWindow::New() ),
	m_initialized(false),
	m_mesh(vtkSmartPointer<vtkPolyData>::New()),
	m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
	m_actor( vtkSmartPointer<vtkActor>::New())
	{

	/* Set the navigation transformation to show the entire scene: */

	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12));
	}

void ViveRviz::frame(){
	if (! m_initialized){
	 	m_initialized=true;


		//Read a ply filei
		//vtkNew<vtkPLYReader> reader;
		//std::string filename= "/home/shuttle/Sources/GeometryViewer/data/optim_colored_o4.ply";
		//std::string filename= "/home/shuttle/Sources/GeometryViewer/data/test2.ply";
		//std::string filename = "/home/system/catkin_ws/src/vive_rviz/data/trigs1000000.ply";
		//std::string filename = "/home/system/catkin_ws/src/vive_rviz/data/optim_colored_o4.ply";
   		//reader->SetFileName(filename.data());
    		//reader->Update();


		//m_mapper->SetInputData(reader->GetOutput());



	 	m_mapper->SetInputData(m_mesh);
                m_actor->SetMapper(m_mapper);	


		m_vtk_renderer->SetActiveCamera(m_vtk_camera);
	        m_vtk_window->AddRenderer(m_vtk_renderer);
		m_vtk_window->SetStereoRender(1);  //Set so as to avoid the glPrintError: Invalid operation 
		m_vtk_renderer->AddActor(m_actor);


	
	}

	

}

void ViveRviz::display(GLContextData& contextData) const 
	{

	if (! m_initialized){
		return;
	}


	
	/*glPushMatrix();
	
	glTranslated(-5.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.5f,1.0f));
	glDrawCube(7.5f);
	
	glTranslated(10.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,1.0f));
	glDrawSphereIcosahedron(4.5f,6);
	
	glPopMatrix();*/

	
	//externalVTKidget->GetRenderWindow()->Render();


	// push everything!
	glPushAttrib( GL_ALL_ATTRIB_BITS );
	glPushClientAttrib( GL_CLIENT_ALL_ATTRIB_BITS );
	glMatrixMode( GL_COLOR );
	glPushMatrix();
	glMatrixMode( GL_TEXTURE );
	glPushMatrix();
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();


	m_display_mtx.lock();	
	m_vtk_window->Render();
	m_display_mtx.unlock();

	glMatrixMode( GL_COLOR );
	glPopMatrix();
	glMatrixMode( GL_TEXTURE );
	glPopMatrix();
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
	glPopAttrib();
	glPopClientAttrib();


	}

void ViveRviz::resetNavigation(void)
	{

	std::cout << "resetNavigation " << std::endl;

	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12));
	}

void ViveRviz::chatterCallback(const std_msgs::String::ConstPtr& msg){
	std::cout << "I heard" << msg->data.c_str() << std::endl;
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	
	
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 }

void ViveRviz::meshCallback(const visualization_msgs::Marker::ConstPtr& msg){


	std::cout << "mesh callback " << std::endl;
        //make a polydata and read the msg into it
	vtkSmartPointer<vtkPolyData> mesh_new = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> vtk_polys = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkUnsignedCharArray> vtk_colors= vtkSmartPointer<vtkUnsignedCharArray>::New();
	vtk_colors->SetNumberOfComponents(3);

	//std::cout << "num_points in msg: " << msg->points.size() << std::endl;	

	int i=0;
	for (int i=0; i< msg->points.size() ;i++  ){
		vtk_points->InsertNextPoint(msg->points[i].x ,msg->points[i].y  ,msg->points[i].z);
		vtk_colors->InsertNextTuple3(msg->colors[i].r*255, msg->colors[i].g*255, msg->colors[i].b*255);
		
	}

	//std::cout << "creating the cells " << std::endl;
	for (int i=0; i< msg->points.size();i=i+3){
                       	vtk_polys->InsertNextCell(3);
    			vtk_polys->InsertCellPoint(i);
    			vtk_polys->InsertCellPoint(i+1);
    			vtk_polys->InsertCellPoint(i+2);
	//		std::cout << "creating triangle " << i << std::endl;
	}


	//exit(0);

	mesh_new->SetPoints(vtk_points);
	mesh_new->GetPointData()->SetScalars(vtk_colors);
	mesh_new->SetPolys(vtk_polys);
	mesh_new->Squeeze();


	//mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper_new = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper_new->SetInputData(mesh_new);
	mapper_new->Update();

	//actor
	vtkSmartPointer<vtkActor> actor_new = vtkSmartPointer<vtkActor>::New();
  	actor_new->SetMapper(mapper_new);


	//Seems that it has the same speed as just setting the points to the mesh but sometimes it throws a segmentation fault	
	//m_mapper->SetInputData(mesh_new);
	//m_display_mtx.lock();
	//m_mapper->Update();	
	//m_display_mtx.unlock();


	m_display_mtx.lock();	
	m_mesh->SetPoints(vtk_points);
	m_mesh->GetPointData()->SetScalars(vtk_colors);
	m_mesh->SetPolys(vtk_polys);
	m_display_mtx.unlock();


	  	
}


void ViveRviz::kinectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

	std::cout << "kinect callback" << std::endl;

	pcl::PCLPointCloud2 temp_cloud;
    	pcl_conversions::toPCL(*msg,temp_cloud);
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    	pcl::fromPCLPointCloud2(temp_cloud,*cloud);


	std::cout << "cloud organized is: " << cloud->isOrganized() << std::endl;

	pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
	pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

	recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
	recon.setInputCloud(cloud);
	recon.reconstruct(*pcl_mesh);

	vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
	pcl::VTKUtils::convertToVTK (*pcl_mesh, vtk_mesh);
	
        m_display_mtx.lock();
	m_mapper->SetInputData(vtk_mesh);
        m_mapper->Update();   
        m_display_mtx.unlock();	

}


void ViveRviz::startROSCommunication(){
	std::cout << "started ros communication" << std::endl;
	ros::NodeHandle nodeH;
     	//ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
     	//ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

	
     	ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/points", 1000, &ViveRviz::kinectCallback, this);
	
    	ros::spin();
	std::cout << "finished ros communication" << std::endl;
}


int main(int argc,char* argv[]){


	//setlocale(LC_ALL, "C");
	ros::init(argc, argv, "vive_rviz");
	//ros::NodeHandle n;


	ViveRviz app(argc,argv);

	//ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(); 

	boost::thread t(&ViveRviz::startROSCommunication, &app);


	//ros::Subscriber sub = node->subscribe("chatter", 1000, &ViveRviz::chatterCallback,&app);
//	ros::Subscriber sub=n.subscribe("mesh_topic",1000,&ViveRviz::meshCallback,&app);


	app.run(); 
	return 0;




}

//VRUI_APPLICATION_RUN(ViveRviz)




