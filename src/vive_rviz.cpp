#define STREAM_BUFFER_CAPACITY 8e+7
#define NUM_BUFFERS 2
#define NUM_CPU_BUFFERS 10

#define NUM_VBO_BUFFERS 10
#define NUM_PBO_BUFFERS 10
#define MAX_VBO_SIZE 1920*1080*sizeof(Vertex) * NUM_VBO_BUFFERS
#define MAX_PBO_SIZE 1920*1080*3 * NUM_PBO_BUFFERS

#include "vive_rviz.h"



/***********************************
Methods of class ViveKin::DataItem:
***********************************/

ViveKin::DataItem::DataItem(void)
	:m_rendering_buffer_0(true)
	{
	/* Initialize the GL_ARB_vertex_buffer_object extension: */
	GLARBVertexBufferObject::initExtension();

	glGenBuffersARB(NUM_BUFFERS, VBO_id);
	glGenBuffersARB(NUM_BUFFERS, indexBufferId);
	glGenBuffersARB(NUM_BUFFERS, texture);
		
	for (int i=0;i < NUM_BUFFERS; i++){
		version[i]=0;
	}




	}

ViveKin::DataItem::~DataItem(void)
	{
	/* Destroy the vertex and index buffers: */


	glDeleteBuffersARB(NUM_BUFFERS,VBO_id);
	glDeleteBuffersARB(NUM_BUFFERS,indexBufferId);
	glDeleteBuffersARB(NUM_BUFFERS,texture);
	}

/**************************
Methods of class ViveKin:
**************************/

double ViveKin::linterp ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}



void* ViveKin::read_data(void ){


	std::cout << "started ros communication" << std::endl;
        ros::NodeHandle nodeH;



        //message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeH, "/kinect2/hd/image_color_rect_uncompressed", 20);
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeH, "/kinect2/hd/image_color", 3);
        message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nodeH, "/kinect2/hd/camera_info", 3);
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nodeH, "/kinect2/qhd/points", 3);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> AsyncPolicy;
        message_filters::Synchronizer<AsyncPolicy> sync(AsyncPolicy(10), image_sub, info_sub, cloud_sub);
        sync.registerCallback(boost::bind(&ViveKin::callback,this, _1, _2,_3 ));
        //sync.registerCallback(boost::bind(&ViveKin::callback3msg,this, _1, _2,_3));


        std::cout << "test" << std::endl;

        //int q=5;
        //message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
        //no_cloud_sync_ = new message_filters::Synchronizer<NoCloudSyncPolicy>(NoCloudSyncPolicy(q), image_sub, info_sub, cloud_sub);
        //no_cloud_sync_->registerCallback(boost::bind(&ViveRviz::callback, this, _1, _2, _3));

        //TimeSynchronizer<Image, CameraInfo,PointCloud2> sync(image_sub, info_sub,cloud_sub, 10);
        //sync.registerCallback(boost::bind(&ViveRviz::callback,this, _1, _2,_3));
        //sync.registerCallback(boost::bind(&ViveRviz::callback2,this, _1, _2,_3));




        //ros::Subscriber sub= nodeH.subscribe("/kinect2/sd/points", 1000, &ViveKin::kinectCallback, this);
        //ros::Subscriber sub2= nodeH.subscribe("/kinect2/hd/image_color", 1000, &ViveKin::imageCallback, this);
        //ros::Subscriber sub3= nodeH.subscribe("/kinect2/hd/camera_info", 1000, &ViveKin::cameraCallback, this);


        ros::spin();
        std::cout << "finished ros communication" << std::endl;


}


void ViveKin::callback3msg(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	std::cout << "--------------------------------------------: " << processing_counter << std::endl;
	processing_counter++;

}


void ViveKin::kinectCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
        std::cout << "kienct callback: " << kinect_callback_counter << std::endl;
	kinect_callback_counter++;

}

void ViveKin::imageCallback(const sensor_msgs::ImageConstPtr& msg){
        std::cout << "image callback: " << img_callback_counter << std::endl;
	img_callback_counter++;
}
void ViveKin::cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg){
        std::cout << "camera callback: " << camera_callback_counter << std::endl;
	camera_callback_counter++;
}



void ViveKin::callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    std::cout << "processing" << std::endl;


    //std::cout << "--------------------------------------------: " << processing_counter << std::endl;
        processing_counter++;



	//std::cout << "processing callback: writer_idx is: " << writer_idx << std::endl;
	writer_idx=(writer_idx+1)%NUM_CPU_BUFFERS;
	//std::cout << "processing callback: writer_idx is: " << writer_idx << std::endl;


	auto t1_proc = Clock::now();

	
	auto t1_meshing = Clock::now();

// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
        pcl_conversions::toPCL(*cloud_msg,*temp_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(*temp_cloud,*cloud);
        //pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
        //pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

        //recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
        //recon.setInputCloud(cloud);
        //recon.reconstruct(*pcl_mesh);

	auto t2_meshing = Clock::now();



	//Make my own mesh 
    float thresh=0.15;
	vertices[writer_idx].clear();
	indices[writer_idx].clear();



	
	for (int x_idx=0; x_idx < cloud->width; x_idx++){
		for (int y_idx=0; y_idx < cloud->height; y_idx++){
			//triangle 1
			unsigned int idx_0= x_idx + y_idx*cloud->width;
			unsigned int idx_1= x_idx +1 + y_idx*cloud->width;
			unsigned int idx_2= x_idx + (y_idx+1)*cloud->width; 

			//triangle 2
			unsigned int idx_3= x_idx + (y_idx+1)*cloud->width;
			unsigned int idx_4= x_idx +1 + (y_idx +1)*cloud->width;
			unsigned int idx_5= x_idx +1 + y_idx*cloud->width;

	
			bool trig_1_invalid=false;
			bool trig_2_invalid=false;

			//get rid of the triangle if any of the coordinates is outside the image
			if ( (x_idx+1) > cloud->width-1 || (y_idx+1)> cloud->height -1){
				trig_1_invalid=true;	
				trig_2_invalid=true;	
				continue;
			}
		
//			std::cout << "at position" << x_idx << " " << y_idx << std::endl;

			//get rid of the triangle if any of the points is nan
			//triangle 1
			if (  isnan(cloud->points[idx_0].z) || isnan(cloud->points[idx_1].z) || isnan(cloud->points[idx_2].z)   ){
				trig_1_invalid=true;
			}
			//triangle 2
			if (  isnan(cloud->points[idx_3].z) || isnan(cloud->points[idx_4].z) || isnan(cloud->points[idx_5].z)   ){
				trig_2_invalid=true;
			}


			//get rid of the triangle if the size of any of the edges si too big
			//triangle 1
			if (!trig_1_invalid){
				float l_0=fabs(cloud->points[idx_0].z - cloud->points[idx_1].z  );
				float l_1=fabs(cloud->points[idx_1].z - cloud->points[idx_2].z  );
				float l_2=fabs(cloud->points[idx_0].z - cloud->points[idx_2].z  );

				if (l_0> thresh || l_1 >thresh || l_2 >thresh){
					trig_1_invalid=true;
				}
			}

	
			


			//triangle 2
			if (!trig_2_invalid){
				float l_3=fabs(cloud->points[idx_3].z - cloud->points[idx_4].z  );
				float l_4=fabs(cloud->points[idx_4].z - cloud->points[idx_5].z  );
				float l_5=fabs(cloud->points[idx_5].z - cloud->points[idx_3].z  );

				if (l_3> thresh || l_4 >thresh || l_5 >thresh){
					trig_2_invalid=true;
				}
			}


			//if (idx_0<0 || idx_1 <0 || idx_2 < 0 || idx_3 < 0 || idx_4 <0 ||idx_5 <0)
			//	exit(0);


			//if we reached here, add the tirangle			
			if (!trig_1_invalid){
				indices[writer_idx].push_back(idx_0);
				indices[writer_idx].push_back(idx_1);
				indices[writer_idx].push_back(idx_2);
			}
	
			if (!trig_2_invalid){
				indices[writer_idx].push_back(idx_3);
				indices[writer_idx].push_back(idx_5);
				indices[writer_idx].push_back(idx_4);
					
			}
			
			//point indexed by two indices
			/*auto indices_2= cloud->at(x_idx,y_idx);
			auto indices_1= cloud->points[idx_0];
			std::cout << "pos with 2 indices is" << indices_2.x << indices_2.y << std::endl;
			std::cout << "pos with 1 indices is" << indices_1.x << indices_1.y << std::endl;
			std::cout << "--------" << std::endl << std::endl; */

		}

	}
	num_indices[writer_idx]=indices[writer_idx].size();


	vertices[writer_idx].resize(cloud->size());
	for (int i=0; i< indices[writer_idx].size(); i++){

		int index=indices[writer_idx][i];

		float x = cloud->points[ index ].x;
                float y = cloud->points[ index ].y;
                float z = cloud->points[ index ].z;

                        vertices[writer_idx][index].x=x;
                        vertices[writer_idx][index].y=y;
                        vertices[writer_idx][index].z=z ;



                        //make the mesh bigger
                        vertices[writer_idx][index].x*=5;
                        vertices[writer_idx][index].y*=5;
                        vertices[writer_idx][index].z*=5;

	
	}

	//std::cout << "indices is" <<indices[writer_idx].size() << std::endl;
	//std::cout << "vertices is" <<vertices[writer_idx].size() << std::endl;
	

//	std::cout << "finished meshing and reading " << std::endl;



	#if 0 

	//go through the mesh add the points and the indices into the corresponding vectors

	vertices.clear();
	indices.clear();


	//std::cout << "cloud has data: " << cloud->size() << std::endl;	
	vertices.resize(cloud->size());

	auto t1_reading = Clock::now();
	//std::cout << "number of polygons is " << pcl_mesh->polygons.size() << std::endl;
	for (int poly_idx=0; poly_idx< pcl_mesh->polygons.size(); poly_idx++){
		//grab that poly, add it's points and the indices



		for (int v_idx=0; v_idx<pcl_mesh->polygons[poly_idx].vertices.size();v_idx++){
		
			unsigned int index= pcl_mesh->polygons[poly_idx].vertices[v_idx];

			float x = cloud->points[ index ].x;
			float y = cloud->points[ index ].y;
			float z = cloud->points[ index ].z;

			vertices[index].x=x;
			vertices[index].y=y;
			vertices[index].z=z ;



			//make the mesh bigger
			vertices[index].x*=5;
			vertices[index].y*=5;
			vertices[index].z*=5;
			//std::cout << "wirint vertice" <<index  << "with pos: " << vertices[index].x<< " " <<vertices[index].y << " " << vertices[index].z << std::endl;

	
			indices.push_back(index);

		}
		
	}
	num_indices=indices.size();

	#endif
	auto t2_reading = Clock::now();



	auto t1_proj = Clock::now();

	Eigen::Matrix<double, 3, 4>  proj_matrix;
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


        //Fix tcoords
        double cols_multiplier=1920.0/2048.0;
        double rows_multiplier=1080.0/2048.0;

        for (int i=0; i< vertices[writer_idx].size();i ++){

		if (  isnan (vertices[writer_idx][i].x) || isnan (vertices[writer_idx][i].y) || isnan (vertices[writer_idx][i].z) || isnan (vertices[writer_idx][i].z)  ){
			//std::cout << "vertes is nan" << std::endl;
			continue;
		}

		//std::cout << "vertex " << i << "survived first continue" << std::endl;

		Eigen::Vector3d point3D (vertices[writer_idx][i].x, vertices[writer_idx][i].y, vertices[writer_idx][i].z);
		if (!point3D.allFinite()){
            //insert a 0 for t coord
			vertices[writer_idx][i].u=vertices[writer_idx][i].v=0;
            continue;
        }

        Eigen::Vector3d point2D= proj_matrix*point3D.homogeneous();

        if (point2D(2)<=0){
			vertices[writer_idx][i].u=vertices[writer_idx][i].v=0;
            continue;
        }

        point2D(0) /= point2D(2);
        point2D(1) /= point2D(2);

        //std::cout << "point is " << point2D(0) << " " << point2D(1) << std::endl;

        if (point2D(0)<0 || point2D(0) >1920 || point2D(1) <0 || point2D(1)>1080  ){
           vertices[writer_idx][i].u=vertices[writer_idx][i].v=0;
           continue;
        }

        float coords[2];
        coords[0]=point2D(0)/1920;
        coords[1]=point2D(1)/1080;

        coords[0]*=cols_multiplier;
        coords[1]*=rows_multiplier;

        //now we need to squish the coorinates in the x axis because the depth image is 512x424 while the rgb is 1920x1080
        //by upcaling the 512x424 image up to the same height as the rgb one we get an image of 1304.1509x1080
        //therefore the coordinates need to be squished by 1304.1509/1920 = 0.67924526041
        //then the uv coorinates need to be moved to still be in the center of the rgb image , therefore we sum (1920 - 1304.1509 )/2

        //coords[0]*=1304.1509/2048;
        //((1920-1304.1509)/2)/1920
        //coords[0]+=((1920-1304.1509)/2)/2048;
        //coords[0]+=(1 - 1304.1509/1920 )/2;


        coords[0]=point2D(0)/2048;
        coords[1]=point2D(1)/2048;
        //now the uv are inside the complete rgb image
        //however we need only a small subsection in the middle of it

        //minimum u is 0
        //middle of the image has u coordinate (1920/2)/2048= 0.46875
        //maximum u is 1920/2048= 0.9375

        //however it should be
        //margin size =1920-1304.1509)/2 = 307.92455
        //minimum 307.92455 / 2048 = 0.15035378418
        //maximum (1920 - 307.92455 )/2048 = 0.78714621582


        //if (coords[0]>0.46875){
            //coords[0]=linterp(coords[0], 0.46875,0.9375, 0.46875, 0.78714621582 );
        //}else{
            //coords[0]=linterp(coords[0], 0.0, 0.46875, 0.15035378418, 0.46875 );
        //}


        //coords[0]*=cols_multiplier;
        //coords[1]*=rows_multiplier;



		

		if (coords[0] <0 || coords [0]>1 || coords[1] < 0 || coords[1]>1){
            std::cout << "TCoords is not valid!" << coords[0]  << " " << coords[1]  <<  std::endl;
            vertices[writer_idx][i].u=vertices[writer_idx][i].v=0;
            continue;
        }

		vertices[writer_idx][i].u=coords[0];
		vertices[writer_idx][i].v=coords[1];

		//std::cout << "writing uv " << vertices[i].u << " " << vertices[i].v << std::endl;

	 }
		
	
	auto t2_proj = Clock::now();
	auto t1_texture = Clock::now();

	//Get image as a texture
        cv::Mat img_cv;
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
                cv_ptr = cv_bridge::toCvShare( image_msg );
                cv_ptr->image.copyTo(img_cv);
                //cv::flip(img_cv,img_cv, -1); //TODO this line needs to be commented
          //      cv::cvtColor(img_cv, img_cv, CV_BGR2RGB);

                //padding
                img_padded[writer_idx].create(2048, 2048, img_cv.type());
                img_padded[writer_idx].setTo(cv::Scalar::all(0));


                //int offset_x=2048-1920;
                //int offset_y=2048-1080;
                img_cv.copyTo(img_padded[writer_idx](cv::Rect(0, 0, img_cv.cols, img_cv.rows)));

                //cv::cvtColor(matGray,matGray, CV_BGR2GRAY);
        }catch (cv_bridge::Exception& e){
                ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
        }

        //cv::cvtColor(img_padded, img_padded, CV_BGR2RGB);

	auto t2_texture = Clock::now();

	consume_mtx.lock();
	last_wrote_idx=writer_idx;
	version++;
	consume_mtx.unlock();
	

	m_data_available=true;


    //DataItem* dataItem=contextGlobal->retrieveDataItem<DataItem>(this);
//	std::cout << "callbakc dataItem is" << dataItem << std::endl;	




	auto t2_proc = Clock::now();

    auto duration_processing = std::chrono::duration_cast<std::chrono::milliseconds>( t2_proc - t1_proc ).count();
    //std::cout << "processing time: "<<  duration_processing << '\n';

    //auto duration_meshing = std::chrono::duration_cast<std::chrono::milliseconds>( t2_meshing - t1_meshing ).count();
    //std::cout << "meshing time: "<<  duration_meshing << '\n';

    //auto duration_reading = std::chrono::duration_cast<std::chrono::milliseconds>( t2_reading - t1_reading ).count();
   // std::cout << "reading: "<<  duration_reading << '\n';

    auto duration_proj = std::chrono::duration_cast<std::chrono::milliseconds>( t2_proj - t1_proj ).count();
    //std::cout << "projecting: "<<  duration_proj << '\n';

    auto duration_texture = std::chrono::duration_cast<std::chrono::milliseconds>( t2_texture - t1_texture ).count();
    //std::cout << "reading texture: "<<  duration_texture << '\n';


    //std::cout << "--------------------: "<<  duration_texture << '\n';






	//dataItem->m_rendering_buffer_0=!dataItem->m_rendering_buffer_0;

	

	#if 0
	//----------Now we have new vertices, indices and opencv mat ready to be read into a vbo and texture
	
	DataItem* dataItem=contextGlobal->retrieveDataItem<DataItem>(this);

	int buf_idx=-1;
	if (dataItem->m_rendering_buffer_0) {
		//fill in buffer 1
		buf_idx=1;
		//switch to render from buffer 1
	}else{
		buf_idx=0;

	}



	//Copying all the data
	/* Upload all vertices into the vertex buffer: */
        glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[buf_idx]);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices.size()*sizeof(Vertex),&vertices[0],GL_STREAM_DRAW_ARB);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);

        /* Upload all vertex indices into the index buffer: */
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[buf_idx]);
        glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices.size()*sizeof(GLuint),&indices[0],GL_STREAM_DRAW_ARB);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);	




	glBindTexture(GL_TEXTURE_2D, dataItem->texture[buf_idx]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     img_padded.cols,          // Image width  i.e. 640 for Kinect in standard mode
                     img_padded.rows,          // Image height i.e. 480 for Kinect in standard mode
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     img_padded.ptr());  
	








	num_indices=indices.size(); //TODO needs a mutex


	dataItem->m_rendering_buffer_0=!dataItem->m_rendering_buffer_0;

	#endif


	//std::cout << "finished processing " << std::endl;

}

//void ViveKin::makeCheckImage(void)
//{
//   int i, j, c;

//   for (i = 0; i < checkImageHeight; i++) {
//      for (j = 0; j < checkImageWidth; j++) {
//         c = ((((i&0x8)==0)^((j&0x8))==0))*255;
//         checkImage[i][j][0] = (GLubyte) c;
//         checkImage[i][j][1] = (GLubyte) c;
//         checkImage[i][j][2] = (GLubyte) c;
//         checkImage[i][j][3] = (GLubyte) 255;
//      }
//   }
//}

ViveKin::ViveKin(int& argc,char**& argv)
	:Vrui::Application(argc,argv)
	{

    std::cout << "max vbo: " << MAX_VBO_SIZE/ 1e+6  << "mb" << std::endl;
    std::cout << "max pbo: " << MAX_PBO_SIZE/ 1e+6 << "mb" << std::endl;

	vertices.resize (NUM_CPU_BUFFERS) ;
        indices.resize (NUM_CPU_BUFFERS);
        img_padded.resize (NUM_CPU_BUFFERS) ;
        num_indices.resize (NUM_CPU_BUFFERS) ;




    //makeCheckImage();

	
	Vertex v= Vertex (-5.0f, -5.0f, 0.0f, 0.0f, 0.0f); // { -5, -5, 0.0, 0.0, 0.0};
	vertices[reader_idx].push_back(v);
	Vertex v2=  {5, 0.0, 0.0, 0.5, 0.0};
	vertices[reader_idx].push_back(v2);
	Vertex v3=  {0.0, 5, 0.0, 0.0, 0.7};
	vertices[reader_idx].push_back(v3);
	Vertex v4=  {0.0, 10, 5.0, 1.0, 1.0};
	vertices[reader_idx].push_back(v4);


	indices[reader_idx].push_back(0);
	indices[reader_idx].push_back(2);
	indices[reader_idx].push_back(1);

	
	indices[reader_idx].push_back(1);
	indices[reader_idx].push_back(2);
	indices[reader_idx].push_back(3);
	
	num_indices[reader_idx]=indices.size(); 

    read_thread.start(this,&ViveKin::read_data);
	}

ViveKin::~ViveKin(void)
	{
    /* Shut down the background ViveKin thread: */
	read_thread.cancel();
	read_thread.join();
	
}

void ViveKin::frame(void)
	{



	


	//change buffer
	/*if (buf_idx==0)
		buf_idx=1;
	else
		buf_idx=0;
*/

    //buf_idx=(buf_idx +1 ) % NUM_BUFFERS;


	/* Check if there is a new entry in the triple buffer and lock it: */
	//if(meshVertices.lockNewValue())
	//	{
		/* Invalidate the in-GPU vertex buffer: */
	//	++version;
	//	}
	}

void ViveKin::display(GLContextData& contextData) const
	{

	/*if (m_data_available){
		m_data_available=false;
		std::cout << "we got new data!" << std::endl;		

	}*/


/*	glPushMatrix();
	
	glTranslated(-5.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(1.0f,0.5f,0.5f));
	glDrawCube(7.5f);
	
	glTranslated(10.0,0.0,0.0);
	glMaterialAmbientAndDiffuse(GLMaterialEnums::FRONT,GLColor<GLfloat,4>(0.5f,0.5f,1.0f));
	glDrawSphereIcosahedron(4.5f,6);
	
	glPopMatrix();
*/
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);



	


//	std::cout << "display dataItem is" << dataItem << std::endl;	


/*
    //---streaming
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, dataItem->VBO_id[0]);
	int val= vertices.size()*sizeof(Vertex);
	int power= pow(2, ceil(log(val)/log(2)));
	GLuint streamDataSize = power;

	if(streamOffset + streamDataSize > STREAM_BUFFER_CAPACITY){
 		// allocate new space and reset the vao
 		glBufferDataARB( GL_ARRAY_BUFFER_ARB,
 		STREAM_BUFFER_CAPACITY,
               	NULL,
              	GL_STREAM_DRAW );


  	glBindBufferARB(GL_ARRAY_BUFFER_ARB, dataItem->VBO_id[0]);

	 glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*5, NULL);
        glTexCoordPointer(2, GL_FLOAT, sizeof(GLfloat)*5, (float*)(sizeof(GLfloat)*3)); //TODO this may need to change to a 2

	
 	glBindVertexArray(0);
 	// reset offset
	 streamOffset = 0;
	}


	return

	
*/

	//---------streaming finished





	// Black background
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/*int buf_idx=-1;
	if (dataItem->m_rendering_buffer_0){
		buf_idx=0;
	}else{
		buf_idx=1;
	}*/



	glDisable(GL_CULL_FACE);
	glNormal3f(0.0f, 0.0f, -1.0f);	
	glColor3f(1.0f,1.0f,1.0f);


	glEnableClientState(GL_VERTEX_ARRAY);
 	glEnableClientState(GL_TEXTURE_COORD_ARRAY);


	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[buf_idx]);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[buf_idx]);
        glBindTexture(GL_TEXTURE_2D, dataItem->texture[buf_idx]);

    //glColor4f(1.0, 1.0, 1.0, 1.0); // reset gl color
       // glClear( GL_COLOR_BUFFER_BIT );

//we comment this part because the worer thread will now load the data
#if 1

	consume_mtx.lock();
        if(dataItem->version[buf_idx]!=version){
	auto t1_upload = Clock::now();
	//	std::cout << "indices is " << indices.size()*sizeof(GLuint) << std::endl;

                /* Upload all vertices into the vertex buffer: */
//		glBufferDataARB(GL_ARRAY_BUFFER_ARB,NULL,&vertices[0],GL_STREAM_DRAW_ARB);
  //              glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,NULL,&indices[0],GL_STREAM_DRAW_ARB);

		reader_idx=last_wrote_idx;		
		
		consume_mtx.unlock();

                glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices[reader_idx].size()*sizeof(Vertex),NULL,GL_STREAM_DRAW_ARB);
                glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices[reader_idx].size()*sizeof(GLuint),NULL,GL_STREAM_DRAW_ARB);


                glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices[reader_idx].size()*sizeof(Vertex),&vertices[reader_idx][0],GL_STREAM_DRAW_ARB);
                glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices[reader_idx].size()*sizeof(GLuint),&indices[reader_idx][0],GL_STREAM_DRAW_ARB);

                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //Important so that the texture do not appear red
                glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     img_padded[reader_idx].cols,          // Image width  i.e. 640 for Kinect in standard mode
                     img_padded[reader_idx].rows,          // Image height i.e. 480 for Kinect in standard mode
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     img_padded[reader_idx].ptr());


	
    auto t2_upload = Clock::now();
    auto duration_upload = std::chrono::duration_cast<std::chrono::milliseconds>( t2_upload - t1_upload ).count();
    //std::cout << "upload: "<<  duration_upload << '\n';



               dataItem->version[buf_idx]=version;
        }

	consume_mtx.unlock();
#endif




	//glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[buf_idx]);
 	//glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[buf_idx]);
  	//glBindTexture(GL_TEXTURE_2D, dataItem->texture[buf_idx]);  //TODO maybe  I also need to bund the texture

  	glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*5, NULL);
  	glTexCoordPointer(2, GL_FLOAT, sizeof(GLfloat)*5, (float*)(sizeof(GLfloat)*3)); //TODO this may need to change to a 2




	 glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping ( NEW )
    	glShadeModel(GL_SMOOTH);                        // Enable Smooth Shading
    glClearColor(0.0f, 0.0f, 0.0f, 0.5f);                   // Black Background
    	//glClearDepth(1.0f);                         // Depth Buffer Setup
    	//glEnable(GL_DEPTH_TEST);                        // Enables Depth Testing
    	//glDepthFunc(GL_LEQUAL);     





	//save stuff
    const_cast<ViveKin*>( this )->        saveGLState();
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


	//std::cout << "displaying num indices" << num_indices[reader_idx]<< std::endl;
	//glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, indexPtr);
	//glDrawArrays(GL_TRIANGLES, 0, 3);
	glDrawElements(GL_TRIANGLES, num_indices[reader_idx], GL_UNSIGNED_INT, NULL);
	//restore stuff

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
        const_cast<ViveKin*>( this )->        restoreGLState();


	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
  	glBindTexture(GL_TEXTURE_2D,0);

  	glDisableClientState(GL_VERTEX_ARRAY);
  	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	glDisable(GL_TEXTURE_2D);




	/*// Draw VBO
    const_cast<ViveKin*>( this )->	saveGLState();



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

	//glPushMatrix();
	glBindBufferARB(GL_ARRAY_BUFFER, dataItem->VBO_id[0]);
	glVertexPointer(3, GL_FLOAT, sizeof(Vertex), 0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_TRIANGLES, 0, 3);
	//glPopMatrix();


	
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


    const_cast<ViveKin*>( this )->	restoreGLState();*/

}


void ViveKin::saveGLState(void){

   glGetIntegerv(GL_ACTIVE_TEXTURE, &this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);




	int max;
	GLint value;
    glGetIntegerv(static_cast<GLenum>(0x8B4D),&value);
    max=static_cast<int>(value);


//	std::cout << "max units is" << max<< std::endl;
if (this->GLStateIntegers["GL_ACTIVE_TEXTURE"] < 0 || this->GLStateIntegers["GL_ACTIVE_TEXTURE"] > max ){

	this->GLStateIntegers["GL_ACTIVE_TEXTURE"] = 0;
}	
	

    this->SavedLighting = glIsEnabled(GL_LIGHTING);
    this->SavedDepthTest = glIsEnabled(GL_DEPTH_TEST);
    this->SavedBlending = glIsEnabled(GL_BLEND);


}

void ViveKin::restoreGLState(void){

	SetGLCapability(GL_LIGHTING, SavedLighting);
        SetGLCapability(GL_DEPTH_TEST, SavedDepthTest);
        SetGLCapability(GL_BLEND, SavedBlending);


	glActiveTexture(GL_TEXTURE0 + this->GLStateIntegers["GL_ACTIVE_TEXTURE"]);

}


void ViveKin::SetGLCapability(GLenum capability, GLboolean state){
    if (state)
    {
      glEnable(capability);
    }
    else
    {
      glDisable(capability);
    }
}

void ViveKin::resetNavigation(void)
	{
	/* Center and scale the object: */
	//	Vrui::setNavigationTransformation(Vrui::Point::origin,9.0*Math::Constants<double>::pi,Vrui::Vector(0,1,0));
	Vrui::setNavigationTransformation(Vrui::Point::origin,Vrui::Scalar(12),Vrui::Vector(0,1,0));
	}

void ViveKin::initContext(GLContextData& contextData) const
	{





	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);





	contextGlobal=&contextData;
    //read_thread.start(this,&ViveKin::read_data);
    //boost::bind(&ViveKin::read_data, this);
    //boost::thread t(&ViveKin::read_data,this);



/*
//-------------streaming	
	int mb_8=8e+7;
	//Putting it more than necesarry
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, dataItem->VBO_id[0]);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, 
   		     STREAM_BUFFER_CAPACITY,	
		     NULL,
              	     GL_STREAM_DRAW_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
	

	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[0]);
        glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,STREAM_BUFFER_CAPACITY,&indices[0],GL_STREAM_DRAW_ARB);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);
	
	std::cout << "alocation of indices is " << indices.size()*sizeof(GLuint) << std::endl;
//------------------finished streaming
*/

	



	/* Upload all vertices into the vertex buffer: */
 	glBindBufferARB(GL_ARRAY_BUFFER_ARB,dataItem->VBO_id[0]);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB,vertices[reader_idx].size()*sizeof(Vertex),&vertices[reader_idx][0],GL_STREAM_DRAW_ARB);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB,0);

  	/* Upload all vertex indices into the index buffer: */
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->indexBufferId[0]);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB,indices[reader_idx].size()*sizeof(GLuint),&indices[reader_idx][0],GL_STREAM_DRAW_ARB);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB,0);




	/*glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glBindTexture(GL_TEXTURE_2D, dataItem->texture[0]);

   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
   	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, checkImageWidth, 
                checkImageHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                checkImage);
*/	




	//other way of reading the image
	std::string filename="/home/system/catkin_ws/synth_tex3.png";
	cv::Mat image = cv::imread(filename);
	cv::flip(image, image, 0);
	glBindTexture(GL_TEXTURE_2D, dataItem->texture[0]);

      	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Set texture clamping method
      	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        //glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //Important so that the texture do not appear red


      glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                     0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                     GL_RGB,            // Internal colour format to convert to
                     image.cols,          // Image width  i.e. 640 for Kinect in standard mode
                     image.rows,          // Image height i.e. 480 for Kinect in standard mode
                     0,                 // Border width in pixels (can either be 1 or 0)
                     GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                     GL_UNSIGNED_BYTE,  // Image data type
                     image.ptr());        // The actual image data itself

//      glGenerateMipmap(GL_TEXTURE_2D);


	//	GLuint shaderProgram = create_program("shaders/vert.shader", "shaders/frag.shader");


}

int main(int argc,char* argv[]){

        ros::init(argc, argv, "vive_rviz");
        ViveKin app(argc,argv);

        app.run();
        return 0;

}



//VRUI_APPLICATION_RUN(ViveKin)
