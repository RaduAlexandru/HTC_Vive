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
	m_img_actor (vtkSmartPointer<vtkImageActor>::New())
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

		//m_vtk_renderer->AddActor(m_img_actor);
	
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

	if (m_texture_initialized){	
		m_actor->SetTexture(m_texture);
	}
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

	//std::cout << "kinect callback" << std::endl;

	auto t1 = Clock::now();


	//pcl::PCLPointCloud2 temp_cloud;
	pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
    	pcl_conversions::toPCL(*msg,*temp_cloud);


	//transform
	



	//reduce nr of points
	//sensor_msgs::PointCloud2ConstPtr temp_cloud_ptr (*temp_cloud); 
  	//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	//pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  	//sor.setInputCloud (temp_cloud);
  	//sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	//sor.filter (*cloud_filtered);



    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    	//pcl::fromPCLPointCloud2(*temp_cloud,*cloud);
    	pcl::fromPCLPointCloud2(*temp_cloud,*cloud);




	//bilateral filter
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::FastBilateralFilter<pcl::PointXYZRGB> filter;
	//filter.setInputCloud(cloud);
	//filter.setSigmaS(3);
	//filter.setSigmaR(0.05);
	//filter.applyFilter(*cloud_filtered);



	//std::cout << "cloud organized is: " << cloud->isOrganized() << std::endl;
	//std::cout << "cloud has nr of points: " << cloud->size() << std::endl;
	//std::cout << "cloud dimensions is: " << cloud->width << " " << cloud->height << std::endl;

	pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
	pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

	recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
	recon.setInputCloud(cloud);
	//recon.setMaxEdgeLength(1);
	//recon.setTrianglePixelSize (3);
	recon.reconstruct(*pcl_mesh);

	vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
	pcl::VTKUtils::convertToVTK (*pcl_mesh, vtk_mesh);
	





	//Transform
	/*tf::TransformListener listener;
	
	tf::StampedTransform transform;
	try{
     		listener.lookupTransform("/base_link", "/kinect2_rgb_optical_frame",  ros::Time(0), transform);
    	}catch (tf::TransformException ex){
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
    	}

	float x,y,z;

	x=transform.getOrigin().x();
	y=transform.getOrigin().y();
	z=transform.getOrigin().z();

	std::cout << "transform is" <<  x << " " << y << " " << z << std::endl;*/



	//go through the points in the mesh and project them into the rgb camera, that will be their uv coordinates
	//int num_points=518400;
	//uniform mat4 projMatrix; 
	//Eigen::Matrix4f modelview = Eigen::Matrix4f::Identity();
	//Eigen::Vector4f point (1,2,3,4);
	//for (int i= 0; i< num_points;i ++){
	//	auto res= modelview*point;
		//std::cout << "res is " << res << std::endl;	
	//}

		
	vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
	vtk_points=vtk_mesh->GetPoints();
	int num_points= vtk_points->GetNumberOfPoints();
	//std::cout << "num of points what wil be projected: " << num_points << std::endl;
	for (int i=0; i< num_points;i ++){
		const Eigen::Matrix3d R = proj_matrix.block<3, 3>(0, 0);
		const Eigen::Vector3d t = proj_matrix.block<3, 1>(0, 3);
	

		double p[3];
    		vtk_mesh->GetPoint(i,p);
		Eigen::Vector3d point3D (p[0], p[1], p[2]);

		Eigen::Vector3d point2Dp = R * point3D + t;
		point2Dp(0) /= point2Dp(2);
		point2Dp(1) /= point2Dp(2);

		double u= point2Dp(0);
		double v= point2Dp(1);


		std::cout << "projected points is " << u << " " << v << std::endl;
	}



	auto t2 = Clock::now();
	auto duration_process_point_cloud = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
	//std::cout << "Processing time of point cloud: "<<  duration_process_point_cloud << '\n';






	
        m_display_mtx.lock();
	m_mapper->SetInputData(vtk_mesh);
        m_mapper->Update();   
        m_display_mtx.unlock();	

}


void ViveRviz::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg ){

	std::cout << "received camera info " << std::endl;
	

	
	proj_matrix(0,0)=msg->P[0];
	proj_matrix(0,1)=msg->P[1];
	proj_matrix(0,2)=msg->P[2];
	proj_matrix(0,3)=msg->P[3];
	proj_matrix(1,0)=msg->P[4];
	proj_matrix(1,1)=msg->P[5];
	proj_matrix(1,2)=msg->P[6];
	proj_matrix(1,3)=msg->P[7];
	proj_matrix(2,0)=msg->P[8];
	proj_matrix(2,1)=msg->P[9];
	proj_matrix(2,2)=msg->P[10];
	proj_matrix(2,3)=msg->P[11];
		
		
}


void CreateColorImage(vtkImageData* image)
{
  unsigned int dim = 20;
 
  image->SetDimensions(dim, dim, 1);
#if VTK_MAJOR_VERSION <= 5
  image->SetNumberOfScalarComponents(3);
  image->SetScalarTypeToUnsignedChar();
  image->AllocateScalars();
#else
  image->AllocateScalars(VTK_UNSIGNED_CHAR,3);
#endif
  for(unsigned int x = 0; x < dim; x++)
    {
    for(unsigned int y = 0; y < dim; y++)
      {
      unsigned char* pixel = static_cast<unsigned char*>(image->GetScalarPointer(x,y,0));
	std::cout << "pixel val is: " << pixel[0] << " " << pixel[1] << " " << pixel[2] << std::endl;
      if(x < dim/2)
	{
	pixel[0] = 255;
	pixel[1] = 0;
	}
      else
	{
	pixel[0] = 0;
	pixel[1] = 255;
	}
 
      pixel[2] = 0;
      }
    }
 
  image->Modified();
}


void ViveRviz::callback(const ImageConstPtr& image_msg, const CameraInfoConstPtr& cam_info_msg, const PointCloud2ConstPtr& cloud_msg){

	//std::cout << "processing everything" << std::endl;


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


        //std::cout << "cloud organized is: " << cloud->isOrganized() << std::endl;
        //std::cout << "cloud has nr of points: " << cloud->size() << std::endl;
        //std::cout << "cloud dimensions is: " << cloud->width << " " << cloud->height << std::endl;

        pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
        pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

        recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
        recon.setInputCloud(cloud);
        recon.reconstruct(*pcl_mesh);

        vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
        pcl::VTKUtils::convertToVTK (*pcl_mesh, vtk_mesh);



	vtkSmartPointer<vtkFloatArray> vtk_tcoords = vtkSmartPointer<vtkFloatArray>::New();
  	vtk_tcoords->SetName("TCoords");
	vtk_tcoords->SetNumberOfComponents(2);


	vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
        vtk_points=vtk_mesh->GetPoints();
        int num_points= vtk_points->GetNumberOfPoints();
	//std::cout << "projection matrix is \n" << proj_matrix.matrix() << std::endl;
        //std::cout << "num of points what wil be projected: " << num_points << std::endl;
        for (int i=0; i< num_points;i ++){
                /*const Eigen::Matrix3d R = proj_matrix.block<3, 3>(0, 0);
                const Eigen::Vector3d t = proj_matrix.block<3, 1>(0, 3);


                double p[3];
                vtk_mesh->GetPoint(i,p);
                Eigen::Vector3d point3D (p[0], p[1], p[2]);

                Eigen::Vector3d point2Dp = R * point3D + t;
                point2Dp(0) /= point2Dp(2);
                point2Dp(1) /= point2Dp(2);

                double u= point2Dp(0);
                double v= point2Dp(1);*/


		double p[3];
                vtk_mesh->GetPoint(i,p);
                Eigen::Vector3d point3D (p[0], p[1], p[2]);
	
		if (!point3D.allFinite()){
			//insert a 0 for t coord
			float coords[2];
			coords[0]=0;
			coords[1]=0;
			vtk_tcoords->InsertNextTuple(coords);
			continue;
		}
	
		Eigen::Vector3d point2D= proj_matrix*point3D.homogeneous();

		if (point2D(2)<=0){
			//insert a 0 for t coords
			float coords[2];
                        coords[0]=0;
                        coords[1]=0;
                        vtk_tcoords->InsertNextTuple(coords);

			continue;
		}
		point2D(0) /= point2D(2);
                point2D(1) /= point2D(2);

		if (point2D(0)<0 || point2D(0) >1920 || point2D(1) <0 || point2D(1)>1080  ){
			//insert a 0 for t coords
			float coords[2];
                        coords[0]=0;
                        coords[1]=0;
                        vtk_tcoords->InsertNextTuple(coords);

			continue;
		}

                //double u= point2D(0);
                //double v= point2D(1);
		
		float coords[2];
		coords[0]=point2D(0)/1920;
		coords[1]=point2D(1)/1080;


		//Fix tcoords


		if (coords[0] <0 || coords [0]>1 || coords[1] < 0 || coords[1]>1){
			std::cout << "TCoords is not valid!" << coords[0]  << " " << coords[1]  <<  std::endl;
		}
		
		vtk_tcoords->InsertNextTuple(coords);
		//std::cout << "projected points is " << coords[0] << " " << coords[1] << std::endl;
        }


	vtk_mesh->GetPointData()->SetTCoords(vtk_tcoords);

	//Get image as a texture	

        cv::Mat matGray;
	cv_bridge::CvImageConstPtr cv_ptr;
    	try{
        	cv_ptr = cv_bridge::toCvShare( image_msg );
		cv_ptr->image.copyTo(matGray);
		cv::flip(matGray,matGray, -1);
		//cv::cvtColor(matGray,matGray, CV_BGR2GRAY);
    	}catch (cv_bridge::Exception& e){
        	ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
    	}


  	m_imageImport->SetDataSpacing(1, 1, 1);
  	m_imageImport->SetDataOrigin(0, 0, 0);
  	m_imageImport->SetWholeExtent(0, matGray.size().width-1, 0, matGray.size().height-1, 0, 0);
  	m_imageImport->SetDataExtentToWholeExtent();
  	m_imageImport->SetDataScalarTypeToUnsignedChar();
  	m_imageImport->SetNumberOfScalarComponents(matGray.channels());
  	m_imageImport->SetImportVoidPointer(matGray.data);
  	m_imageImport->Update();

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
	//m_texture_initialized=true;
  	//m_texture->SetInputConnection(m_imageImport->GetOutputPort());
	//m_texture->Update();
	m_display_mtx.unlock();

	






	//synthetic texture
	std::string filename= "synth_tex3.png";
	vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
    	pngReader->SetFileName (filename.data() );
    	pngReader->Update();

	m_display_mtx.lock();
    	m_texture->SetInputConnection(pngReader->GetOutputPort());
	//m_texture=texture;
	m_texture_initialized=true;
	m_display_mtx.unlock();
	



	//Set mesh and draw it
	m_display_mtx.lock();
        m_mapper->SetInputData(vtk_mesh);
        m_mapper->Update();
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


}


void ViveRviz::startROSCommunication(){
	std::cout << "started ros communication" << std::endl;
	ros::NodeHandle nodeH;
     	//ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
     	//ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

	
     	//ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/points", 1000, &ViveRviz::kinectCallback, this);
	//sub= nodeH.subscribe("/kinect2/qhd/camera_info", 1000, &ViveRviz::cameraInfoCallback, this);	


	message_filters::Subscriber<Image> image_sub(nodeH, "/kinect2/hd/image_color", 1);
  	message_filters::Subscriber<CameraInfo> info_sub(nodeH, "/kinect2/hd/camera_info", 1);
	message_filters::Subscriber<PointCloud2> cloud_sub(nodeH, "/kinect2/qhd/points", 1);

	//int q=5;
	//message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
	//no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q), image_sub, info_sub, cloud_sub);
	//no_cloud_sync_->registerCallback(boost::bind(&ViveRviz::callback, this, _1, _2, _3));

  	TimeSynchronizer<Image, CameraInfo,PointCloud2> sync(image_sub, info_sub,cloud_sub, 10);
  	sync.registerCallback(boost::bind(&ViveRviz::callback,this, _1, _2,_3));


    	ros::spin();
	std::cout << "finished ros communication" << std::endl;
}

void ViveRviz::getCameraInfo(){
	std::cout << "started get camera info thread" << std::endl;
        ros::NodeHandle nodeH;
        //ros::Subscriber sub= nodeH.subscribe("chatter", 1000, &ViveRviz::chatterCallback, this);
        //ros::Subscriber sub= nodeH.subscribe("visualization_marker", 1000, &ViveRviz::meshCallback, this);

        ros::Subscriber sub= nodeH.subscribe("/kinect2/qhd/camera_info", 1000, &ViveRviz::cameraInfoCallback, this);

        ros::spin();
        std::cout << "finished get camera info thread" << std::endl;


}


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




