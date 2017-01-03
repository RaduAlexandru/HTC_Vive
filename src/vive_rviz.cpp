//#define STREAM_BUFFER_CAPACITY 8e+7
//#define NUM_BUFFERS 2
//#define NUM_CPU_BUFFERS 10

#define NUM_VBO_BUFFERS 10
#define NUM_IBO_BUFFERS 10
#define NUM_PBO_BUFFERS 10

#define MAX_VBO_SIZE 1920*1080*sizeof(Vertex) * NUM_VBO_BUFFERS
#define MAX_IBO_SIZE 1920*1080*sizeof(GLuint) * NUM_IBO_BUFFERS
#define MAX_PBO_SIZE 1920*1080*3 * NUM_PBO_BUFFERS

#include "vive_rviz.h"



/***********************************
Methods of class ViveKin::DataItem:
***********************************/

ViveKin::DataItem::DataItem(void){

    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ibo);
    glGenBuffers(1, &m_pbo);

    glGenBuffers(1, &m_texture);

    version=0;

}

ViveKin::DataItem::~DataItem(void){
    /* Destroy all buffers: */

    glDeleteBuffers(1, &m_vbo);
    glDeleteBuffers(1, &m_ibo);
    glDeleteBuffers(1, &m_pbo);
    glDeleteBuffers(1, &m_texture);

}

/**************************
Methods of class ViveKin:
**************************/


ViveKin::ViveKin(int& argc,char**& argv)
    :Vrui::Application(argc,argv),
    m_vbo_bytes_written(0),
    m_ibo_bytes_written(0),
    m_pbo_bytes_written(0),

    m_offset_vbo_writing(0),
    m_offset_ibo_writing(0),
    m_offset_pbo_writing(0),

    m_offset_vbo_rendering(0),
    m_offset_ibo_rendering(0),
    m_offset_pbo_rendering(0)
{

    std::cout << "max vbo: " << MAX_VBO_SIZE/ 1e+6  << "mb" << std::endl;
    std::cout << "max ibo: " << MAX_IBO_SIZE/ 1e+6  << "mb" << std::endl;
    std::cout << "max pbo: " << MAX_PBO_SIZE/ 1e+6 << "mb" << std::endl;

    Vertex v= Vertex (-5.0f, -5.0f, 0.0f, 0.0f, 0.0f); // { -5, -5, 0.0, 0.0, 0.0};
    m_vertices.push_back(v);
    Vertex v2=  {5, 0.0, 0.0, 0.5, 0.0};
    m_vertices.push_back(v2);
    Vertex v3=  {0.0, 5, 0.0, 0.0, 0.7};
    m_vertices.push_back(v3);
    Vertex v4=  {0.0, 10, 5.0, 1.0, 1.0};
    m_vertices.push_back(v4);


    m_indices.push_back(0);
    m_indices.push_back(2);
    m_indices.push_back(1);


    m_indices.push_back(1);
    m_indices.push_back(2);
    m_indices.push_back(3);

    m_num_indices=m_indices.size();

    read_thread.start(this,&ViveKin::read_data);



}

ViveKin::~ViveKin(void)
    {
    /* Shut down the background ViveKin thread: */
    read_thread.cancel();
    read_thread.join();
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


    ros::spin();
    std::cout << "finished ros communication" << std::endl;


}

void ViveKin::mesh_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ){

    float thresh=0.15;
    m_vertices.clear();
    m_indices.clear();



    //PCL WAY
//    pcl::OrganizedFastMesh<pcl::PointXYZRGB> recon;
//    pcl::PolygonMesh::Ptr pcl_mesh (new pcl::PolygonMesh ) ;

//    recon.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
//    recon.setInputCloud(cloud);
//    recon.reconstruct(*pcl_mesh);

//    std::cout << "pcl reconstruction has tri count " << pcl_mesh->polygons.size() << std::endl;






    /*
    current points is at index 0 and we form 2 triangle coming outof it. The indexes are:

    0-------1
    |     /
    |   /
    | /
    2
    triangle 1


            5
          / |
        /   |
      /     |
    3-------4
    triangle 2
    */

    //the index needs to be offset to account for all the other vertices that are in the vbo that we will not render
    unsigned int idx_offset= m_vbo_bytes_written / sizeof(Vertex);


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

            //get rid of the triangle if any of the points has z nan (depth is not caputured by the camera)
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
                m_indices.push_back(idx_1 +idx_offset);
                m_indices.push_back(idx_0+ idx_offset);
                m_indices.push_back(idx_2 + idx_offset);
            }

            if (!trig_2_invalid){
                m_indices.push_back(idx_3+ idx_offset);
                m_indices.push_back(idx_4+ idx_offset);
                m_indices.push_back(idx_5+ idx_offset);

            }


        }

    }



    m_vertices.resize(cloud->size());
    for (int i=0; i< m_indices.size(); i++){

        int index=m_indices[i];
        index-=idx_offset; //revert back the offset so it points to the vertices in the m_vertices and no the ones in the vbo

        float x = cloud->points[ index ].x;
        float y = cloud->points[ index ].y;
        float z = cloud->points[ index ].z;

        m_vertices[index].x=x;
        m_vertices[index].y=y;
        m_vertices[index].z=z ;

        //make the mesh bigger
        m_vertices[index].x*=5;
        m_vertices[index].y*=5;
        m_vertices[index].z*=5;


    }


}

void ViveKin::project_points (const sensor_msgs::CameraInfoConstPtr& cam_info_msg){

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


     for (int i=0; i< m_vertices.size();i ++){

        //no need to assign any uv coordinates to these points because they are not references by any index in the IBO
        if (  isnan (m_vertices[i].z)  ){
            continue;
        }



        Eigen::Vector3d point3D (m_vertices[i].x, m_vertices[i].y, m_vertices[i].z);
        if (!point3D.allFinite()){
            //insert a 0 for t coord
            m_vertices[i].u=m_vertices[i].v=0;
            continue;
        }

        Eigen::Vector3d point2D= proj_matrix*point3D.homogeneous();

        if (point2D(2)<=0){
            m_vertices[i].u=m_vertices[i].v=0;
            continue;
        }

        point2D(0) /= point2D(2);
        point2D(1) /= point2D(2);

        //std::cout << "point is " << point2D(0) << " " << point2D(1) << std::endl;

        if (point2D(0)<0 || point2D(0) >1920 || point2D(1) <0 || point2D(1)>1080  ){
           m_vertices[i].u=m_vertices[i].v=0;
           continue;
        }

        float coords[2];
        coords[0]=point2D(0)/2048;
        coords[1]=point2D(1)/2048;


        if (coords[0] <0 || coords [0]>1 || coords[1] < 0 || coords[1]>1){
            std::cout << "TCoords is not valid!" << coords[0]  << " " << coords[1]  <<  std::endl;
            m_vertices[i].u=m_vertices[i].v=0;
            continue;
        }

        m_vertices[i].u=coords[0];
        m_vertices[i].v=coords[1];

        //std::cout << "writing uv " << vertices[i].u << " " << vertices[i].v << std::endl;

     }

}

void ViveKin::read_texture(const sensor_msgs::ImageConstPtr& image_msg){
    //Get image as a texture
    cv::Mat img_cv;
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare( image_msg );
        cv_ptr->image.copyTo(img_cv);
        //cv::flip(img_cv,img_cv, -1); //TODO this line needs to be commented

        //padding
        m_img_padded.create(2048, 2048, img_cv.type());
        m_img_padded.setTo(cv::Scalar::all(0));

        img_cv.copyTo(m_img_padded(cv::Rect(0, 0, img_cv.cols, img_cv.rows)));
    }catch (cv_bridge::Exception& e){
            ROS_ERROR( "cv_bridge exception: %s", e.what() );
    return;
    }



}

unsigned int ViveKin::upload_vbo(){

//    std::cout << "upload vbo not implemented yet" << std::endl;

    //see if we have space to write exerything
    if (  (m_vbo_bytes_written + m_vertices.size()*sizeof(Vertex)) >MAX_VBO_SIZE  ){
        //by writing the vertices.size()*sizeof(Vertex) we end up ouf ot the buffer so we start again at 0
        std::cout << "vbo is full--------" << std::endl;

        //the points will be loaded at the beggining of the vbo therefore the index should actually have an offset of 0. we need to correct this
        unsigned int idx_offset= m_vbo_bytes_written / sizeof(Vertex);
        for (int i=0;i< m_indices.size(); i++ ){
            m_indices[i]-=idx_offset;
        }




        m_offset_vbo_writing=0;
        m_vbo_bytes_written=0;


    }


    void *dst = (unsigned char*) m_vbo_ptr + m_offset_vbo_writing;
    memcpy(dst, &m_vertices[0], m_vertices.size()*sizeof(Vertex) );
    m_vbo_bytes_written+=m_vertices.size()*sizeof(Vertex);

    //return where did we start the writing from so we know where to render from
    return m_offset_vbo_writing;
}

unsigned int ViveKin::upload_ibo(){

//    std::cout << "upload ibo not implemented yet" << std::endl;

    //see if we have space to write exerything
    if (  (m_ibo_bytes_written + m_indices.size()*sizeof(GLuint)) >MAX_IBO_SIZE  ){
        //by writing the vertices.size()*sizeof(Vertex) we end up ouf ot the buffer so we start again at 0
        std::cout << "ibo is full" << std::endl;
        m_offset_ibo_writing=0;
        m_ibo_bytes_written=0;
    }


    void *dst = (unsigned char*) m_ibo_ptr + m_offset_ibo_writing;
    memcpy(dst, &m_indices[0], m_indices.size()*sizeof(GLuint) );
    m_ibo_bytes_written+=m_indices.size()*sizeof(GLuint);

    //return where did we start the writing from so we know where to render from
    return m_offset_ibo_writing;

}

unsigned int ViveKin::upload_pbo(){
  std::cout << "upload pbo not implemented yet" << std::endl;

}



void ViveKin::callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
//    std::cout << "processing" << std::endl;


    //std::cout << "--------------------------------------------: " << processing_counter << std::endl;
    processing_counter++;

    unsigned int offset_vbo_tmp, offset_ibo_tmp, offset_pbo_tmp;


    //get cloud
    pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg,*temp_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*temp_cloud,*cloud);

    //filter cloud
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::FastBilateralFilter<pcl::PointXYZRGB> filter;
//    filter.setInputCloud(cloud);
//    filter.setSigmaS(3);
//    //filter.setSigmaR(0.05);
//    filter.setSigmaR(0.03);
//    filter.applyFilter(*cloud_filtered);

    //mesh cloud
    mesh_cloud (cloud);

    //project_points
    project_points (cam_info_msg);

    //read_texture
    read_texture(image_msg);


    //upload into vbo, ibo and pbo and the corresponding buf_offset_write
    offset_vbo_tmp= upload_vbo();
    offset_ibo_tmp= upload_ibo();
//    offset_pbo_tmp= upload_pbo();


    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    VertexCacheOptimizer vco;
//    printf("Optimizing ... \n");
//    unsigned int time = GetMSec();
    int tri_count=m_indices.size()/3;
    std::cout << "tri_count " << tri_count << std::endl;
//    VertexCacheOptimizer::Result res = vco.Optimize(&m_indices[0], tri_count);
//    if (res)
//    {
//        std::cout << "error optimizing indices" << std::endl;
//        std::cout << "return is " << res <<  std::endl;
//        return;
//    }

//    std::cout << "finished optimizing" << std::endl;


    //update offsets
    m_consume_mtx.lock();
    m_num_indices=m_indices.size();
    m_offset_vbo_rendering=offset_vbo_tmp;
    m_offset_ibo_rendering=offset_ibo_tmp;
    m_offset_pbo_rendering=offset_pbo_tmp;

    version++;
    m_consume_mtx.unlock();

    m_offset_vbo_writing+=m_vertices.size()*sizeof(Vertex);
    m_offset_ibo_writing+=m_indices.size()*sizeof(GLuint);
    //m_offset_pbo_writing+=vertices.size()*sizeof(Vertex); //TODO IMPLMENT


    /* Wake up the foreground thread by requesting a Vrui frame immediately: */
    //Vrui::requestUpdate();

}



void ViveKin::frame(void){



}

void ViveKin::display(GLContextData& contextData) const{


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


    m_consume_mtx.lock();
    if (dataItem->version!=version){
        dataItem->version=version;
        m_internal_ibo_offset=m_offset_ibo_rendering;
        m_internal_num_indices=m_num_indices;
    }
    m_consume_mtx.unlock();




    //glDisable(GL_CULL_FACE); //TODO enable it so that we gain some performance
    glNormal3f(0.0f, 0.0f, -1.0f);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    //glCullFace(GL_FRONT);

    glBindBuffer(GL_ARRAY_BUFFER_ARB,dataItem->m_vbo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,dataItem->m_ibo);
    glBindTexture(GL_TEXTURE_2D, dataItem->m_texture);


    //this probably may be put into init context
    glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*5, NULL);
    glTexCoordPointer(2, GL_FLOAT, sizeof(GLfloat)*5, (float*)(sizeof(GLfloat)*3));


    glEnable(GL_TEXTURE_2D);                        // Enable Texture Mapping ( NEW )
    glShadeModel(GL_SMOOTH);                        // Enable Smooth Shading

    const_cast<ViveKin*>( this )->	saveGLState();
    //in order to skip the first triangle: BUFFER_OFFSET(3*sizeof(GLuint))
    glDrawElements(GL_TRIANGLES, m_internal_num_indices, GL_UNSIGNED_INT, BUFFER_OFFSET(m_internal_ibo_offset) );
    const_cast<ViveKin*>( this )->	restoreGLState();

    glBindTexture(GL_TEXTURE_2D,0);
    glBindBuffer(GL_ARRAY_BUFFER_ARB,0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER_ARB,0);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);

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


}

void ViveKin::restoreGLState(void){

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

void ViveKin::initContext(GLContextData& contextData) const{

	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);

    const GLbitfield flags = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;


    glBindBuffer(GL_ARRAY_BUFFER,dataItem->m_vbo);
    glBufferStorage(GL_ARRAY_BUFFER, MAX_VBO_SIZE, NULL, flags);
    m_vbo_ptr= glMapBufferRange(GL_ARRAY_BUFFER, 0 , MAX_VBO_SIZE, flags);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,dataItem->m_ibo);
    glBufferStorage(GL_ELEMENT_ARRAY_BUFFER, MAX_IBO_SIZE, NULL, flags);
    m_ibo_ptr= glMapBufferRange(GL_ELEMENT_ARRAY_BUFFER, 0 , MAX_IBO_SIZE, flags);

    //needs to be deactivated because we don't put any data in it yet and therefore the texture would be black
//    glBindBuffer(GL_PIXEL_UNPACK_BUFFER,dataItem->m_pbo);
//    glBufferStorage(GL_PIXEL_UNPACK_BUFFER, MAX_PBO_SIZE, NULL, flags);
//    m_pbo_ptr= glMapBufferRange(GL_PIXEL_UNPACK_BUFFER, 0 , MAX_PBO_SIZE, flags);



    //now we have all the pointer, so we can send data to it
    //the first bits of data will be send starting with offset 0
    memcpy(m_vbo_ptr, &m_vertices[0], m_vertices.size()*sizeof(Vertex));
    memcpy(m_ibo_ptr, &m_indices[0], m_indices.size()*sizeof(GLuint));

    m_vbo_bytes_written=m_vertices.size()*sizeof(Vertex);
    m_ibo_bytes_written=m_indices.size()*sizeof(GLuint);
    m_offset_vbo_writing=m_vertices.size()*sizeof(Vertex);
    m_offset_ibo_writing=m_indices.size()*sizeof(GLuint);




    //temporarely also add a texture
    std::string filename="/home/system/catkin_ws/synth_tex3.png";
    cv::Mat image = cv::imread(filename);
    cv::flip(image, image, 0);
    glBindTexture(GL_TEXTURE_2D, dataItem->m_texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); //Important so that the texture do not appear red

    std::cout << "synthetic texture is of size" << image.cols << " x " << image.rows << std::endl;

    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 image.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 image.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 image.ptr());        // The actual image data itself


    //TODO I don't think it's necesary to unbind then
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);


}

//TODO: move this into another file called Utils.cpp
double ViveKin::linterp ( double input , double input_start, double input_end, double output_start, double output_end){

  double output;
  output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);

  return output;

}



int main(int argc,char* argv[]){

        ros::init(argc, argv, "vive_rviz");
        ViveKin app(argc,argv);

        app.run();
        return 0;

}



//VRUI_APPLICATION_RUN(ViveKin)
