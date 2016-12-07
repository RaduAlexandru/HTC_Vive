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
	m_actor( vtkSmartPointer<vtkActor>::New()),
	m_using_back_renderer(false),
	m_vtk_back_camera(vtkSmartPointer<vtkExternalOpenGLCamera>::New()),
	m_vtk_back_renderer( vtkSmartPointer<vtkExternalOpenGLRenderer>::New() ),
	m_vtk_back_window( vtkExternalOpenGLRenderWindow::New() )
	{

	/* Set the navigation transformation to show the entire scene: */

	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12));
	}

void ViveRviz::frame(){
	if (! m_initialized){
	 	m_initialized=true;

		/*// Create a sphere
  		vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  		sphereSource->SetCenter(0.0, 0.0, 0.0);
  		sphereSource->SetRadius(1.0);
  		sphereSource->Update();
  		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  		mapper->SetInputConnection(sphereSource->GetOutputPort());
  		vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  		actor->SetMapper(mapper); */

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
    		//m_mapper->SetImmediateModeRendering(1);
   		//m_mapper->SetStatic(1);
                m_actor->SetMapper(m_mapper);	

	

		m_vtk_renderer->SetActiveCamera(m_vtk_camera);
	        m_vtk_window->AddRenderer(m_vtk_renderer);
		m_vtk_window->SetStereoRender(1);  //Set so as to avoid the glPrintError: Invalid operation 
		m_vtk_renderer->AddActor(m_actor);


		//Back renderer
		//m_vtk_back_renderer->SetActiveCamera(m_vtk_back_camera);
		//m_vtk_back_window->AddRenderer(m_vtk_back_renderer);
		//m_vtk_back_window->SetStereoRender(1);		
	
	}

	

}

void ViveRviz::display(GLContextData& contextData) const 
	{

	if (! m_initialized){
		return;
	}


	//------remove the previous actor


	//int num_actors= m_vtk_renderer->GetActors()->GetNumberOfItems();

//	std::cout << "we have actors: "<< num_actors << std::endl;

			//m_display_mtx.lock();
	//if (num_actors>=2){
//		std::cout << "starting to remove actors" << std::endl;
		//Remove all actors except the lates one
	//	m_vtk_renderer->GetActors()->InitTraversal();
	//	for (int i=0; i < num_actors-2;i++){
	//		std::cout << "removed an actor" << std::endl;
			//m_vtk_renderer->RemoveActor ( m_vtk_renderer->GetActors()->GetNextActor() );
			//m_vtk_renderer->GetActors()->GetNextActor()->VisibilityOff();
	//	}

		//vtkSmartPointer<vtkActor> latest_actor;
		//latest_actor=m_vtk_renderer->GetActors()->GetLastActor();
		
		//m_vtk_renderer->RemoveAllViewProps();
		//m_vtk_renderer->AddActor(latest_actor);
		
	//}

			//m_display_mtx.unlock();


	//m_mapper->Update();
	
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
	//if (m_using_back_renderer){
	//	std::cout << "rendering using the back one" << std::endl;
	//	m_vtk_back_window->Render();
	//}else{
	//	std::cout << "rendering using the front one" << std::endl;
	//	m_vtk_window->Render();
	//}

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
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//std::cout << "received a mesh" << std::endl;


        //make a polydata and read the msg into it

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

	vtkSmartPointer<vtkPolyData> mesh_new = vtkSmartPointer<vtkPolyData>::New();
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


//		m_display_mtx.lock();
//	if (m_using_back_renderer){
		//If write now we are using the back one , we switch to the other one and add the actor to it

//		m_vtk_renderer->RemoveAllViewProps();
//		m_vtk_renderer->AddActor(actor_new);
		//m_display_mtx.lock();
//		std::cout << "switch" << std::endl;
//		m_using_back_renderer=!m_using_back_renderer;
		//m_vtk_window->Render();
		//m_display_mtx.unlock();
//	}else{
//		m_vtk_back_renderer->RemoveAllViewProps();
//		m_vtk_back_renderer->AddActor(actor_new);
		//m_display_mtx.lock();
//		std::cout << "switch" << std::endl;
//		m_using_back_renderer=!m_using_back_renderer;
		//m_vtk_back_window->Render();
		//m_display_mtx.unlock();
//	}
//		m_display_mtx.unlock();

//	m_display_mtx.lock();
//	m_vtk_renderer->AddActor(actor_new);
//	m_display_mtx.unlock();
        	
	
	

	
	//m_mapper->SetInputData(mesh_new);
	//m_display_mtx.lock();
    	//m_mapper->SetImmediateModeRendering(1);
   	//m_mapper->SetStatic(1);
	//m_mapper->Update();	
	//m_display_mtx.unlock();


	m_display_mtx.lock();	
	m_mesh->SetPoints(vtk_points);
	m_mesh->GetPointData()->SetScalars(vtk_colors);
	m_mesh->SetPolys(vtk_polys);
	m_display_mtx.unlock();


	//std::cout << "There are " << mesh->GetNumberOfCells() << " cells." << std::endl;
 



	//vtkNew<vtkPLYReader> reader2;
          //      std::string filename2= "/home/shuttle/Sources/GeometryViewer/data/optim_colored_o5.ply";
           //     reader2->SetFileName(filename2.data());
           //     reader2->Update();
                //reader->GetOutput()->GetBounds(this->DataBounds);
	//std::cout << "finished reaaading---------------------------" << std::endl;
	

	
             //   m_mapper->SetInputData(reader2->GetOutput());
	//m_mapper->SetInputData(m_mesh);
	//m_mapper->Update();

//	vtkSmartPointer<vtkPLYWriter> plyWriter =vtkSmartPointer<vtkPLYWriter>::New();
//  	plyWriter->SetFileName("/home/system/catkin_ws/src/vive_rviz/data/trigs1000000.ply");
//  	plyWriter->SetInputData(mesh_new);
//	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
//	colors->SetNumberOfComponents(3);
//	colors->SetName("Colors");
//	plyWriter->SetArrayName("Colors");
//  	plyWriter->Write();	
//	exit(0);

	//std::this_thread::sleep_for(std::chrono::milliseconds(100));


	//display_mtix.unlock();



	//vtkSmartPointer<vtkPolyDataNormals> normals_alg = vtkSmartPointer<vtkPolyDataNormals>::New();
	//normals_alg->SetInputData(mesh);
	//normals_alg->Update();


	//vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	//mapper->SetInputData(normals_alg->GetOutput());
        //vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        //actor->SetMapper(mapper);

	//lock the display


//	vtk_renderer->RemoveAllViewProps();
 	//vtk_renderer->AddActor(actor);


//	m_display_mtx.unlock();

	//unlock the display


	
 
//	for(std::vector<geometry_msgs::Point>::const_iterator it = msg->points.begin(); it != msg->points.end(); ++it)
//	{
		//std::cout << "msg: "<< i << " is " << msg->points[i].x << std::endl;
		//Arr[i] = *it;
//		i++;
//	}


	//vtk_polys->InsertNextCell(3);

	 //vtk_points->InsertNextPoint(m_points[v_idx][0],m_points[v_idx][1],m_points[v_idx][2]);
	//m_polyData->SetPoints(vtk_points);
	//m_polyData->SetPolys(vtk_polys);
	//m_polyData->Squeeze();

	//assign the polydata to the mapper and call update on it (or modified)

  	
}


void ViveRviz::startROSCommunication(){
	std::cout << "started ros communication" << std::endl;
	ros::NodeHandle nodeH;
     	//ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
     	ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);
    	ros::spin();
	std::cout << "finished ros communication" << std::endl;
}


int main(int argc,char* argv[]){


	setlocale(LC_ALL, "C");
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




