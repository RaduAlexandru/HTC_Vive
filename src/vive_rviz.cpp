/*
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


//#include <OGRE/OgreSceneManager.h>
//#include <OGRE/OgreRenderSystem.h>
//#include <OGRE/OgreRenderWindow.h>
//#include <OGRE/OgreHardwarePixelBuffer.h>
//#include <OGRE/RenderSystems/GL/OgreGLTexture.h>
//#include <OGRE/RenderSystems/GL/OgreGLRenderTexture.h>

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

VRUI_APPLICATION_RUN(VruiDemoSmall)

*/




#include "vive_rviz.h"
#include "Vrui/VRWindow.h"
#include "Vrui/Viewer.h"
#include <cv_bridge/cv_bridge.h>


using namespace sensor_msgs;
using namespace message_filters;



ViveRviz::ViveRviz(int& argc,char**& argv) :Vrui::Application(argc,argv),
	m_vtk_camera(vtkSmartPointer<vtkExternalOpenGLCamera>::New()),
	m_vtk_renderer( vtkSmartPointer<vtkExternalOpenGLRenderer>::New() ),
	m_vtk_window( vtkExternalOpenGLRenderWindow::New() ),
	m_initialized(false),
	m_mesh(vtkSmartPointer<vtkPolyData>::New()),
	m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
	m_actor( vtkSmartPointer<vtkActor>::New()),
	m_texture ( vtkSmartPointer<vtkTexture>::New()),
	m_texture_initialized (false),
	m_imageImport (vtkSmartPointer<vtkImageImport>::New()),
	m_img_actor (vtkSmartPointer<vtkImageActor>::New()),
	m_texture_changed(false)
	{


	std::string inputFilename = "/home/system/catkin_ws/optim_colored_o4.ply";
 
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( inputFilename.c_str() );


	m_mapper->SetInputConnection(reader->GetOutputPort());
	 //m_mapper->SetInputData(m_mesh);
         m_actor->SetMapper(m_mapper);

	//m_mapper->SetImmediateModeRendering(1);

         m_vtk_renderer->SetActiveCamera(m_vtk_camera);
         m_vtk_window->AddRenderer(m_vtk_renderer);
         m_vtk_window->SetStereoRender(1);  //Set so as to avoid the glPrintError: Invalid operation 
         m_vtk_renderer->AddActor(m_actor);


	//m_vtk_window-> 	SetAutomaticWindowPositionAndResize(0);

	/* Set the navigation transformation to show the entire scene: */
	Vrui::setNavigationTransformation(Vrui::Point(0,0,0),Vrui::Scalar(0));
	}

void ViveRviz::frame(){


		//Read a ply filei
		//vtkNew<vtkPLYReader> reader;
		//std::string filename= "/home/shuttle/Sources/GeometryViewer/data/optim_colored_o4.ply";
		//std::string filename= "/home/shuttle/Sources/GeometryViewer/data/test2.ply";
		//std::string filename = "/home/system/catkin_ws/src/vive_rviz/data/trigs1000000.ply";
		//std::string filename = "/home/system/catkin_ws/src/vive_rviz/data/optim_colored_o4.ply";
   		//reader->SetFileName(filename.data());
    		//reader->Update();


		//m_mapper->SetInputData(reader->GetOutput());



	 	//m_mapper->SetInputData(m_mesh);
                //m_actor->SetMapper(m_mapper);	


		//m_vtk_renderer->SetActiveCamera(m_vtk_camera);
	        //m_vtk_window->AddRenderer(m_vtk_renderer);
		//m_vtk_window->SetStereoRender(1);  //Set so as to avoid the glPrintError: Invalid operation 
		//m_vtk_renderer->AddActor(m_actor);

		//m_vtk_renderer->AddActor(m_img_actor);

	/* if (m_texture_initialized && m_texture_changed){
                m_actor->SetTexture(m_texture);
                m_texture_changed=false;
        }*/
	

	
	//m_display_mtx.lock();

	
	//auto t1 = Clock::now();

	//m_display_mtx.lock();
	//if (m_added_new_actor){
	//	vtkSmartPointer<vtkActor> actor;
	//	actor=m_vtk_renderer->GetActors()->GetLastActor();
	//	m_vtk_renderer->RemoveAllViewProps();
	//	m_vtk_renderer->AddActor(actor);
	//	m_added_new_actor=false;
	//}
	//m_display_mtx.unlock();
	//m_display_mtx.unlock();

	//auto t2 = Clock::now();
	//auto duration_rendering = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
	//std::cout << "frame time: "<<  duration_rendering << '\n';





	//auto t1 = Clock::now();

	//new way of doing it
	if (m_added_new_actor){
		//std::cout << " frame has seen that a new actor was added " << std::endl;
		m_vtk_renderer->RemoveAllViewProps();
		m_vtk_renderer->AddActor(m_actor);
		m_added_new_actor=false;
	}	

	//auto t2 = Clock::now();
	//auto duration_rendering = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
	//std::cout << "frame time: "<<  duration_rendering << '\n';
}

void ViveRviz::display(GLContextData& contextData) const 
	{

	//TODO set it back on
	//if (! m_initialized){
	//	return;
	//}


	
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


	//m_display_mtx.lock();

	//TODO It may be possible to take this if out of the mutex
	//if (m_texture_initialized && m_texture_changed){
	//	m_actor->SetTexture(m_texture);
	//	m_texture_changed=false;
	//}
	
	//auto t1 = Clock::now();
	//m_vtk_window->Start();
	m_vtk_window->Render();
	//const_cast<ViveRviz*>( this )->custom_window();
	//auto t2 = Clock::now();
	//auto duration_rendering = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
	//std::cout << "Rendering time: "<<  duration_rendering << '\n';
	
	//m_display_mtx.unlock();


	//m_vtk_window->SetAAFrames(3);
	//m_vtk_window->SetSubFrames(3);
	//std::cout << "aa frames is" << m_vtk_window->GetAAFrames() << std::endl;
	//std::cout << "fd frames is" << m_vtk_window->GetFDFrames() << std::endl;
	//std::cout << "sub frames is" << m_vtk_window->GetSubFrames() << std::endl;
	//std::cout << "size is  " << m_vtk_window->GetSize()[0] << " " << m_vtk_window->GetSize()[1] << std::endl;
	//exit(0);
	//this->Size[0] && 0 == this->Size[1]
	//bool val= (m_vtk_window->GetStereoType() != VTK_STEREO_RIGHT);
	//std::cout << "val is: " << val << std::endl;


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


void ViveRviz::custom_window(){
	//m_vtk_window->Start();
	//m_vtk_window->SaveGLState(); //Its protected so we implement our own
    	m_vtk_window->MakeCurrent();
    	glGetIntegerv(GL_ACTIVE_TEXTURE, &this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);
    	if (this->GLStateIntegers["GL_ACTIVE_TEXTURE"] < 0 || this->GLStateIntegers["GL_ACTIVE_TEXTURE"] > m_vtk_window->GetTextureUnitManager()->GetNumberOfTextureUnits()) {
      		this->GLStateIntegers["GL_ACTIVE_TEXTURE"] = 0;
    	}

	m_vtk_window->Start();

	GLboolean SavedLighting = glIsEnabled(GL_LIGHTING);
    	GLboolean SavedDepthTest = glIsEnabled(GL_DEPTH_TEST);
	GLboolean SavedBlending = glIsEnabled(GL_BLEND);



	m_vtk_window->InvokeEvent(vtkCommand::StartEvent,NULL);

	//------Render stuff
	m_vtk_window->StereoUpdate(); //TODO--may no tb needed


	if (m_rendering_left_eye){ // render the left eye
     		 m_vtk_renderer->GetActiveCamera()->SetLeftEye(1);
		m_vtk_window->GetRenderers()->Render();
      		//m_vtk_renderer->Render();
  	}	

  	if (m_vtk_window->GetStereoRender()){
    		m_vtk_window->StereoMidpoint();
    		if (!m_rendering_left_eye){ // render the right eye
			//std::cout << "render right eye" << std::endl;
          		m_vtk_renderer->GetActiveCamera()->SetLeftEye(0);
			m_vtk_window->GetRenderers()->Render();
     			//m_vtk_renderer->Render();
    		}
    		m_vtk_window->StereoRenderComplete();
	}


	m_rendering_left_eye=!m_rendering_left_eye;








  	//m_vtk_window->InRender = 0;
	m_vtk_window->InvokeEvent(vtkCommand::EndEvent,NULL);
////	m_vtk_window->RestoreGLState();		--it's protected so we implement our own
//	std::cout << "restoring" << std::endl;
	SetGLCapability(GL_LIGHTING, SavedLighting);
    	SetGLCapability(GL_DEPTH_TEST, SavedDepthTest);
	SetGLCapability(GL_BLEND, SavedBlending);
//	std::cout << "finsihed restoring" << std::endl;


	//This may not need to be here
	// For now just re-store the texture unit
    	glActiveTexture(GL_TEXTURE0 + this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);

    	// Unuse active shader program
	m_vtk_window->GetShaderCache()->ReleaseCurrentShader();

}


void ViveRviz::custom_render(){

	 GLdouble mv[16],p[16];
  glGetDoublev(GL_MODELVIEW_MATRIX,mv);
  glGetDoublev(GL_PROJECTION_MATRIX,p);


  m_vtk_camera->SetProjectionTransformMatrix(p);
  m_vtk_camera->SetViewTransformMatrix(mv);

  vtkMatrix4x4* matrix = vtkMatrix4x4::New();
  matrix->DeepCopy(mv);
  matrix->Transpose();
  matrix->Invert();

  // Synchronize camera viewUp
  double viewUp[4] = {0.0, 1.0, 0.0, 0.0}, newViewUp[4];
  matrix->MultiplyPoint(viewUp, newViewUp);
  vtkMath::Normalize(newViewUp);
  m_vtk_camera->SetViewUp(newViewUp);

  // Synchronize camera position
  double position[4] = {0.0, 0.0, 1.0, 1.0}, newPosition[4];
  matrix->MultiplyPoint(position, newPosition);

  if (newPosition[3] != 0.0)
  {
    newPosition[0] /= newPosition[3];
    newPosition[1] /= newPosition[3];
    newPosition[2] /= newPosition[3];
    newPosition[3] = 1.0;
  }
  m_vtk_camera->SetPosition(newPosition);

  // Synchronize focal point
  double focalPoint[4] = {0.0, 0.0, 0.0, 1.0}, newFocalPoint[4];
  matrix->MultiplyPoint(focalPoint, newFocalPoint);
  m_vtk_camera->SetFocalPoint(newFocalPoint);

matrix->Delete();





}

#if 0
void ViveRviz::lights (){
	/ Lights
  // Query lights existing in the external context
  // and tweak them based on vtkExternalLight objects added by the user
  GLenum curLight;
  for (curLight = GL_LIGHT0;
       curLight < GL_LIGHT0 + MAX_LIGHTS;
       curLight++)
  {
    GLboolean status;
    glGetBooleanv(curLight, &status);

    int l_ind = static_cast<int> (curLight - GL_LIGHT0);
    vtkLight* light = NULL;
    bool light_created = false;
    light = vtkLight::SafeDownCast(
              m_vtk_renderer>GetLights()->GetItemAsObject(l_ind));


      // No matching light found in the VTK light collection
      if (status)
      {
        // Create a new light only if one is present in the external context
        light = vtkLight::New();
        // Headlight because VTK will apply transform matrices
        light->SetLightTypeToHeadlight();
        light_created = true;
      }
      



    // Find out if there is an external light object associated with this
    // light index.
    vtkCollectionSimpleIterator sit;
    vtkExternalLight* eLight;
    vtkExternalLight* curExtLight = NULL;
   

	glGetLightfv(curLight, GL_AMBIENT, info);
	light->SetAmbientColor(info[0], info[1], info[2]);








}

    if (curExtLight &&
        (curExtLight->GetReplaceMode() == vtkExternalLight::ALL_PARAMS))
    {
      // If the replace mode is all parameters, blatantly overwrite the
      // parameters of existing/new light
      light->DeepCopy(curExtLight);
    }
    else
    {

      GLfloat info[4];

      // Set color parameters
      if (curExtLight && curExtLight->GetIntensitySet())
      {
        light->SetIntensity(curExtLight->GetIntensity());
      }

      if (curExtLight && curExtLight->GetAmbientColorSet())
      {
        light->SetAmbientColor(curExtLight->GetAmbientColor());
      }
      else
      {
        glGetLightfv(curLight, GL_AMBIENT, info);
        light->SetAmbientColor(info[0], info[1], info[2]);
      }
      if (curExtLight && curExtLight->GetDiffuseColorSet())
      {
        light->SetDiffuseColor(curExtLight->GetDiffuseColor());
      }
      else
      {
        glGetLightfv(curLight, GL_DIFFUSE, info);
        light->SetDiffuseColor(info[0], info[1], info[2]);
      }
      if (curExtLight && curExtLight->GetSpecularColorSet())
      {
        light->SetSpecularColor(curExtLight->GetSpecularColor());
      }
      else
      {
        glGetLightfv(curLight, GL_SPECULAR, info);
        light->SetSpecularColor(info[0], info[1], info[2]);
      }

      // Position, focal point and positional
      glGetLightfv(curLight, GL_POSITION, info);

      if (curExtLight && curExtLight->GetPositionalSet())
      {
        light->SetPositional(curExtLight->GetPositional());
      }
      else
      {
        light->SetPositional(info[3] > 0.0 ? 1 : 0);
      }

      if (!light->GetPositional())
      {
        if (curExtLight && curExtLight->GetFocalPointSet())
        {
          light->SetFocalPoint(curExtLight->GetFocalPoint());
          if (curExtLight->GetPositionSet())
          {
            light->SetPosition(curExtLight->GetPosition());
          }
          else
          {
            light->SetPosition(info[0], info[1], info[2]);
          }
        }
        else
        {
          light->SetFocalPoint(0, 0, 0);
          if (curExtLight && curExtLight->GetPositionSet())
          {
            light->SetPosition(curExtLight->GetPosition());
          }
          else
          {
            light->SetPosition(-info[0], -info[1], -info[2]);
          }
        }
      }
      else
      {
        if (curExtLight && curExtLight->GetPositionSet())
        {
          light->SetPosition(curExtLight->GetPosition());
        }
        else
        {
          light->SetPosition(info[0], info[1], info[2]);
        }

        // Attenuation
        if (curExtLight && curExtLight->GetAttenuationValuesSet())
        {
          light->SetAttenuationValues(curExtLight->GetAttenuationValues());
        }
        else
        {
          glGetLightfv(curLight, GL_CONSTANT_ATTENUATION, &info[0]);
          glGetLightfv(curLight, GL_LINEAR_ATTENUATION, &info[1]);
          glGetLightfv(curLight, GL_QUADRATIC_ATTENUATION, &info[2]);
          light->SetAttenuationValues(info[0], info[1], info[2]);
        }

        // Cutoff
        if (curExtLight && curExtLight->GetConeAngleSet())
        {
          light->SetConeAngle(curExtLight->GetConeAngle());
        }
        else
        {
          glGetLightfv(curLight, GL_SPOT_CUTOFF, &info[0]);
          light->SetConeAngle(info[0]);
        }

        if (light->GetConeAngle() < 180.0)
        {
          // Exponent
          if (curExtLight && curExtLight->GetExponentSet())
          {
            light->SetExponent(curExtLight->GetExponent());
          }
          else
          {
            glGetLightfv(curLight, GL_SPOT_EXPONENT, &info[0]);
            light->SetExponent(info[0]);
          }

          // Direction
          if (curExtLight && curExtLight->GetFocalPointSet())
          {
            light->SetFocalPoint(curExtLight->GetFocalPoint());
          }
          else
          {
            glGetLightfv(curLight, GL_SPOT_DIRECTION, info);
            for (unsigned int i = 0; i < 3; ++i)
            {
              info[i] += light->GetPosition()[i];
            }
            light->SetFocalPoint(info[0], info[1], info[2]);
          }
        }
      }
    }

    // If we created a new VTK light, add it to the collection
    if (light_created)
    {
      this->AddLight(light);
      light->Delete();
    }
}


}


#endif

void ViveRviz::SetGLCapability(GLenum capability, GLboolean state) {

	//std::cout << "error before resting" << std::endl;
	//GLenum err;
    //while ((err = glGetError()) != GL_NO_ERROR) {
       // cerr << "OpenGL error: " << err << endl;
    //}

	
    if (state)
    {
      glEnable(capability);
    }
    else
    {
      glDisable(capability);
    }
    //vtkOpenGLStaticCheckErrorMacro("failed after SetGLCapability");
	//std::cout << "error after resting" << std::endl;

	//GLenum err2;
    //while ((err2 = glGetError()) != GL_NO_ERROR) {
      //  cerr << "OpenGL error: " << err2 << endl;
    //}

}

void ViveRviz::resetNavigation(void)
	{

	std::cout << "resetNavigation " << std::endl;

	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(1));
	}

void ViveRviz::chatterCallback(const std_msgs::String::ConstPtr& msg){
	std::cout << "I heard" << msg->data.c_str() << std::endl;
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	
	
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 }


void ViveRviz::callback2(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

        std::cout << "started processing" << std::endl;       

	//Get camera projection matrix
        proj_matrix(0,0)=cam_info_msg->P[0];
        proj_matrix(0,1)=cam_info_msg->P[1];
        proj_matrix(0,2)=cam_info_msg->P[2];
        proj_matrix(0,3)=cam_info_msg->P[3];
        proj_matrix(1,0)=cam_info_msg->P[4];
        proj_matrix(1,1)=cam_info_msg->P[5];
        proj_matrix(1,2)=cam_info_msg->P[6];
        proj_matrix(1,3)=cam_info_msg->P[7];
        proj_matrix(2,0)=cam_info_msg->P[8];
        proj_matrix(2,1)=cam_info_msg->P[9];
        proj_matrix(2,2)=cam_info_msg->P[10];
        proj_matrix(2,3)=cam_info_msg->P[11];


        pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*cloud_msg,*temp_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*temp_cloud,*cloud);

        //bilateral filter
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        //pcl::FastBilateralFilter<pcl::PointXYZRGB> filter;
        //filter.setInputCloud(cloud);
        //filter.setSigmaS(3);
        ////filter.setSigmaR(0.05);
        //filter.setSigmaR(0.03);
        //filter.applyFilter(*cloud_filtered);


	pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
        pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;
        recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
        recon.setInputCloud(cloud);
        recon.reconstruct(*pcl_mesh);

        vtkSmartPointer<vtkPolyData> temp_mesh = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
        pcl::VTKUtils::convertToVTK (*pcl_mesh, temp_mesh);

        //assign it to blank points
        vtkSmartPointer<vtkUnsignedCharArray> vtk_colors= vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);
        vtk_colors->SetName ("Colors");
        int nr_points = pcl_mesh->cloud.width * pcl_mesh->cloud.height;
        for (int i=0; i< nr_points ;i++ ){
                vtk_colors->InsertNextTuple3(255, 255, 255);
        }
        vtk_mesh->GetPointData()->SetScalars(vtk_colors);



	vtkSmartPointer<vtkQuadricDecimation> decimator = vtkSmartPointer<vtkQuadricDecimation>::New();
	decimator->SetInputData(temp_mesh);
	decimator->SetTargetReduction(.99); //10% reduction (if there was 100 triangles, now there will be 90)
  	decimator->Update();
	vtk_mesh->DeepCopy(decimator->GetOutput());



	vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
        vtk_tcoords->SetName("TCoords");
        vtk_tcoords->SetNumberOfComponents(2);


        vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points=vtk_mesh->GetPoints();
        int num_points= vtk_points->GetNumberOfPoints();
        for (int i=0; i< num_points;i ++){

                double p[3];
                vtk_mesh->GetPoint(i,p);
                Eigen::Vector3d point3D (p[0], p[1], p[2]);

                if (!point3D.allFinite()){
                        //insert a 0 for t coord
                        float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
                        continue;
                }

                Eigen::Vector3d point2D= proj_matrix*point3D.homogeneous();

                if (point2D(2)<=0){
                        float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
                        continue;
                }
                point2D(0) /= point2D(2);
                point2D(1) /= point2D(2);

                if (point2D(0)<0 || point2D(0) >1920 || point2D(1) <0 || point2D(1)>1080  ){
                        float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
                        continue;
                }

                float coords[2];
                coords[0]=point2D(0)/1920;
                coords[1]=point2D(1)/1080;


                //Fix tcoords
                double cols_multiplier=1920.0/2048.0;
                double rows_multiplier=1080.0/2048.0;


                coords[0]*=cols_multiplier;
                coords[1]*=rows_multiplier;

	         if (coords[0] <0 || coords [0]>1 || coords[1] < 0 || coords[1]>1){
                        std::cout << "TCoords is not valid!" << coords[0]  << " " << coords[1]  <<  std::endl;
                }

                vtk_tcoords->InsertNextTuple(coords);
                //std::cout << "projected points is " << coords[0] << " " << coords[1] << std::endl;
        }


        vtk_mesh->GetPointData()->SetTCoords(vtk_tcoords);

        //Get image as a texture        
        cv::Mat img_cv;
        cv::Mat img_padded;
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
                cv_ptr = cv_bridge::toCvShare( image_msg );
                cv_ptr->image.copyTo(img_cv);
                //cv::flip(img_cv,img_cv, -1); //TODO this line needs to be commented
		cv::cvtColor(img_cv, img_cv, CV_BGR2RGB);


                //padding
                img_padded.create(2048, 2048, img_cv.type());
                img_padded.setTo(cv::Scalar::all(0));


                //int offset_x=2048-1920;
                //int offset_y=2048-1080;
                img_cv.copyTo(img_padded(cv::Rect(0, 0, img_cv.cols, img_cv.rows)));

                //cv::cvtColor(matGray,matGray, CV_BGR2GRAY);
        }catch (cv_bridge::Exception& e){
                ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
        }


	vtkSmartPointer<vtkImageImport> vtk_imageImport= vtkSmartPointer<vtkImageImport>::New();
	//vtk_imageImport->SetReleaseDataFlag(0);
        vtk_imageImport->SetDataSpacing(1, 1, 1);
        vtk_imageImport->SetDataOrigin(0, 0, 0);
        vtk_imageImport->SetWholeExtent(0, img_padded.size().width-1, 0, img_padded.size().height-1, 0, 0);
        vtk_imageImport->SetDataExtentToWholeExtent();
        vtk_imageImport->SetDataScalarTypeToUnsignedChar();
        vtk_imageImport->SetNumberOfScalarComponents(img_padded.channels());
        int size_bytes=img_padded.total() * img_padded.elemSize();
        vtk_imageImport->CopyImportVoidPointer ( img_padded.data, size_bytes   );
        //vtk_imageImport->SetImportVoidPointer(img_padded.data,1);
        //m_display_mtx.lock();
        vtk_imageImport->Update();


	//Make the texture
	vtkSmartPointer<vtkTexture> vtk_texture= vtkSmartPointer<vtkTexture>::New();
	vtk_texture->SetInputConnection(vtk_imageImport->GetOutputPort());
        vtk_texture->Update();

	//mapper
	vtkSmartPointer<vtkPolyDataMapper> vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
	vtk_mapper->SetInputData(vtk_mesh);
        vtk_mapper->Update();


	//actor
	vtkSmartPointer<vtkActor> vtk_actor = vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->SetTexture(vtk_texture);
	vtk_actor->Modified();
	vtk_actor->GetMapper()->Update();
	vtk_actor->ApplyProperties();


	//add it
	//m_display_mtx.lock();	
        //m_vtk_renderer->AddActor(vtk_actor);
	//m_added_new_actor=true;
	//m_display_mtx.unlock();


	//new way of doing it
	//add this actor to a vector
	

	m_actor=vtk_actor;
	//m_actor->ShallowCopy(vtk_actor);
	//m_actor->Modified();
	//m_actor->GetMapper()->Update();
	//m_actor->GetMapper()->Modified();
	//m_actor->GetMapper()->Update();
	m_added_new_actor=true;

}



void ViveRviz::callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	std::cout << "started processing" << std::endl;	

	proj_matrix(0,0)=cam_info_msg->P[0];
        proj_matrix(0,1)=cam_info_msg->P[1];
        proj_matrix(0,2)=cam_info_msg->P[2];
        proj_matrix(0,3)=cam_info_msg->P[3];
        proj_matrix(1,0)=cam_info_msg->P[4];
        proj_matrix(1,1)=cam_info_msg->P[5];
        proj_matrix(1,2)=cam_info_msg->P[6];
        proj_matrix(1,3)=cam_info_msg->P[7];
        proj_matrix(2,0)=cam_info_msg->P[8];
        proj_matrix(2,1)=cam_info_msg->P[9];
        proj_matrix(2,2)=cam_info_msg->P[10];
        proj_matrix(2,3)=cam_info_msg->P[11];


	pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*cloud_msg,*temp_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*temp_cloud,*cloud);

	//bilateral filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::FastBilateralFilter<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud);
        filter.setSigmaS(3);
        //filter.setSigmaR(0.05);
	filter.setSigmaR(0.03);
        filter.applyFilter(*cloud_filtered);



        //std::cout << "cloud organized is: " << cloud->isOrganized() << std::endl;
        //std::cout << "cloud has nr of points: " << cloud->size() << std::endl;
        //std::cout << "cloud dimensions is: " << cloud->width << " " << cloud->height << std::endl;

        pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
        pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

        recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
        recon.setInputCloud(cloud_filtered);
        recon.reconstruct(*pcl_mesh);

        vtkSmartPointer<vtkPolyData> temp_mesh = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
        pcl::VTKUtils::convertToVTK (*pcl_mesh, vtk_mesh);

	//assign it to blank points
	vtkSmartPointer<vtkUnsignedCharArray> vtk_colors= vtkSmartPointer<vtkUnsignedCharArray>::New();
	vtk_colors->SetNumberOfComponents(3);
	vtk_colors->SetName ("Colors");
	int nr_points = pcl_mesh->cloud.width * pcl_mesh->cloud.height;
	for (int i=0; i< nr_points ;i++ ){
		vtk_colors->InsertNextTuple3(255, 255, 255);
	}	
	vtk_mesh->GetPointData()->SetScalars(vtk_colors);
	

	//vtkSmartPointer<vtkQuadricDecimation> decimator = vtkSmartPointer<vtkQuadricDecimation>::New();
	//decimator->SetInputData(temp_mesh);
	//decimator->SetTargetReduction(.99); //10% reduction (if there was 100 triangles, now there will be 90)
  	//decimator->Update();
	//vtk_mesh->DeepCopy(decimator->GetOutput());



	

	vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
  	vtk_tcoords->SetName("TCoords");
	vtk_tcoords->SetNumberOfComponents(2);


	vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points=vtk_mesh->GetPoints();
        int num_points= vtk_points->GetNumberOfPoints();
        for (int i=0; i< num_points;i ++){

		double p[3];
                vtk_mesh->GetPoint(i,p);
                Eigen::Vector3d point3D (p[0], p[1], p[2]);
	
		if (!point3D.allFinite()){
			//insert a 0 for t coord
			float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
			continue;
		}
	
		Eigen::Vector3d point2D= proj_matrix*point3D.homogeneous();

		if (point2D(2)<=0){
			float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
			continue;
		}
		point2D(0) /= point2D(2);
                point2D(1) /= point2D(2);

		if (point2D(0)<0 || point2D(0) >1920 || point2D(1) <0 || point2D(1)>1080  ){
			float coords[2]; coords[0]= coords[1]=0; vtk_tcoords->InsertNextTuple(coords);
			continue;
		}

		float coords[2];
		coords[0]=point2D(0)/1920;
		coords[1]=point2D(1)/1080;


		//Fix tcoords
		double cols_multiplier=1920.0/2048.0;
		double rows_multiplier=1080.0/2048.0;


		coords[0]*=cols_multiplier;
		coords[1]*=rows_multiplier;
		


		if (coords[0] <0 || coords [0]>1 || coords[1] < 0 || coords[1]>1){
			std::cout << "TCoords is not valid!" << coords[0]  << " " << coords[1]  <<  std::endl;
		}
		
		vtk_tcoords->InsertNextTuple(coords);
		//std::cout << "projected points is " << coords[0] << " " << coords[1] << std::endl;
        }


	vtk_mesh->GetPointData()->SetTCoords(vtk_tcoords);

	//Get image as a texture	
        cv::Mat img_cv;
	cv::Mat img_padded;
	cv_bridge::CvImageConstPtr cv_ptr;
    	try{
        	cv_ptr = cv_bridge::toCvShare( image_msg );
		cv_ptr->image.copyTo(img_cv);
		//cv::flip(img_cv,img_cv, -1); //TODO this line needs to be commented
		//cv::cvtColor(img_cv, img_cv, CV_BGR2RGB);

		//padding
		img_padded.create(2048, 2048, img_cv.type());	
		img_padded.setTo(cv::Scalar::all(0));
		

		//int offset_x=2048-1920;
		//int offset_y=2048-1080;
		img_cv.copyTo(img_padded(cv::Rect(0, 0, img_cv.cols, img_cv.rows)));
		
		//cv::cvtColor(matGray,matGray, CV_BGR2GRAY);
    	}catch (cv_bridge::Exception& e){
        	ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
    	}

	cv::cvtColor(img_padded, img_padded, CV_BGR2RGB);

	m_imageImport->SetReleaseDataFlag(1);
  	m_imageImport->SetDataSpacing(1, 1, 1);
  	m_imageImport->SetDataOrigin(0, 0, 0);
  	m_imageImport->SetWholeExtent(0, img_padded.size().width-1, 0, img_padded.size().height-1, 0, 0);
  	m_imageImport->SetDataExtentToWholeExtent();
  	m_imageImport->SetDataScalarTypeToUnsignedChar();
  	m_imageImport->SetNumberOfScalarComponents(img_padded.channels());
	int size_bytes=img_padded.total() * img_padded.elemSize();
	m_imageImport->CopyImportVoidPointer ( img_padded.data, size_bytes   );
  	//m_imageImport->SetImportVoidPointer(img_padded.data);
	//m_display_mtx.lock();
  	m_imageImport->Update();
	//m_display_mtx.unlock(); 


  	/*m_imageImport->SetDataSpacing(1, 1, 1);
  	m_imageImport->SetDataOrigin(0, 0, 0);
  	m_imageImport->SetWholeExtent(0, (image_msg->width)-1, 0, image_msg->height-1, 0, 0);
  	m_imageImport->SetDataExtentToWholeExtent();
  	m_imageImport->SetDataScalarTypeToUnsignedChar();
  	m_imageImport->SetNumberOfScalarComponents(3);
	int size_bytes=image_msg->step * image_msg->height;
	m_imageImport->CopyImportVoidPointer ( (void*)&image_msg->data, size_bytes   );
  	//m_imageImport->SetImportVoidPointer(  (void*)&image_msg->data,1  );
  	m_imageImport->Update();*/

	// update the actor an actor
	//m_display_mtx.lock();	
	//m_img_actor->SetInputData(m_imageImport->GetOutput());
	//m_img_actor->Update();

	//std::cout << "texture initialzied is" << m_texture_initialized << std::endl;
	//if (!m_texture_initialized){
	//	m_texture_initialized=true;
	//	std::cout << "adding the image actor" << std::endl;
	//	m_vtk_renderer->AddActor(m_img_actor); 
	//}
	//m_display_mtx.unlock();



	// Put as texture
	m_display_mtx.lock();
  	m_texture->SetInputConnection(m_imageImport->GetOutputPort());
	m_texture->Update();
	m_texture_initialized=true;
	m_texture_changed=true;
	m_display_mtx.unlock();

	






	//synthetic texture
	//std::string filename= "synth_tex3.png";
	//vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
    	//pngReader->SetFileName (filename.data() );
    	//pngReader->Update();
	//m_display_mtx.lock();
    	//m_texture->SetInputConnection(pngReader->GetOutputPort());
	//m_texture_initialized=true;
	//m_display_mtx.unlock();
	



	//Set mesh and draw it
	m_display_mtx.lock();
        m_mapper->SetInputData(vtk_mesh);
        m_mapper->Update();
        m_display_mtx.unlock();





	//Add the actor directly
	m_display_mtx.lock();
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(vtk_mesh);
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
	actor->SetTexture(m_texture);
        m_vtk_renderer->AddActor(actor);
	m_added_new_actor=true;

	m_display_mtx.unlock();



	//Draw a sphere at camera coordinates, in this case is at 0,0,0
	/*vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  	sphereSource->SetCenter(0.0, 0.0, 0.0);
  	sphereSource->SetRadius(0.2);
  	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  	mapper->SetInputConnection(sphereSource->GetOutputPort());
  	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  	actor->SetMapper(mapper);
  	m_vtk_renderer->AddActor(actor);*/

	std::cout << "finished processing" << std::endl;	
}


void ViveRviz::startROSCommunication(){
	std::cout << "started ros communication" << std::endl;
	ros::NodeHandle nodeH;
     	//ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
     	//ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

	
     	//ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/points", 1000, &ViveRviz::kinectCallback, this);
	//sub= nodeH.subscribe("/kinect2/qhd/camera_info", 1000, &ViveRviz::cameraInfoCallback, this);	


	//message_filters::Subscriber<Image> image_sub(nodeH, "/kinect2/hd/image_color", 1);
  	//message_filters::Subscriber<CameraInfo> info_sub(nodeH, "/kinect2/hd/camera_info", 1);
	//message_filters::Subscriber<PointCloud2> cloud_sub(nodeH, "/kinect2/qhd/points", 1);

	
	//message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeH, "/kinect2/hd/image_color_rect_uncompressed", 20);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeH, "/kinect2/hd/image_color", 20);
  	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nodeH, "/kinect2/hd/camera_info", 20);
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nodeH, "/kinect2/qhd/points", 20);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> AsyncPolicy;
	message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(200), image_sub, info_sub, cloud_sub);
  	sync.registerCallback(boost::bind(&ViveRviz::callback2,this, _1, _2,_3));
	

	std::cout << "test" << std::endl;
	
	//int q=5;
	//message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
	//no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q), image_sub, info_sub, cloud_sub);
	//no_cloud_sync_->registerCallback(boost::bind(&ViveRviz::callback, this, _1, _2, _3));

  	//TimeSynchronizer<Image, CameraInfo,PointCloud2> sync(image_sub, info_sub,cloud_sub, 10);
  	//sync.registerCallback(boost::bind(&ViveRviz::callback,this, _1, _2,_3));
  	//sync.registerCallback(boost::bind(&ViveRviz::callback2,this, _1, _2,_3));
	


	
     	//ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/points", 1000, &ViveRviz::kinectCallback, this);
	//ros::Subscriber sub2= nodeH.subscribe("/kinect2/hd/image_color_rect_uncompressed", 1000, &ViveRviz::imageCallback, this);
	//ros::Subscriber sub3= nodeH.subscribe("/kinect2/hd/camera_info", 1000, &ViveRviz::cameraCallback, this);


    	ros::spin();
	std::cout << "finished ros communication" << std::endl;
}


void ViveRviz::kinectCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	std::cout << "kienct callback" << std::endl;

}

void ViveRviz::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	std::cout << "image callback" << std::endl;
}
void ViveRviz::cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg){
	std::cout << "camera callback" << std::endl;
}

/*void ViveRviz::getCameraInfo(){
	std::cout << "started get camera info thread" << std::endl;
        ros::NodeHandle nodeH;
        //ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
        //ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

        ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/camera_info", 1000, &ViveRviz::cameraInfoCallback, this);

        ros::spin();
        std::cout << "finished get camera info thread" << std::endl;


}*/



/*void ViveRviz::getCameraInfo(){
	std::cout << "started get camera info thread" << std::endl;
        ros::NodeHandle nodeH;
        //ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
        //ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

        ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/camera_info", 1000, &ViveRviz::cameraInfoCallback, this);

        ros::spin();
        std::cout << "finished get camera info thread" << std::endl;


}*/


int main(int argc,char* argv[]){


	//setlocale(LC_ALL, "C");
	ros::init(argc, argv, "vive_rviz");
	//ros::NodeHandle n;


	ViveRviz app(argc,argv);

	//ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(); 

	boost::thread t(&ViveRviz::startROSCommunication, &app);
	//boost::thread t2(&ViveRviz::getCameraInfo, &app);

	//ros::Subscriber sub = node->subscribe("chatter", 1000, &ViveRviz::chatterCallback,&app);
//	ros::Subscriber sub=n.subscribe("mesh_topic",1000,&ViveRviz::meshCallback,&app);


	app.run(); 
	return 0;




}

//VRUI_APPLICATION_RUN(ViveRviz)

