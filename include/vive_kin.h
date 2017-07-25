#include <omp.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <limits>
#include <map>

#define GL_GLEXT_PROTOTYPES


//Vrui
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Threads/Thread.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glut.h>


#include <GL/GLObject.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBVertexBufferObject.h>
#include <GL/GLMaterial.h>
#include <GL/GLVertexTemplates.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLGeometryVertex.h>
#include <GL/GLVertex.h>
#include <GL/GLModels.h>
#include <GL/GLMaterialTemplates.h>


//PCL
//#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/organized_fast_mesh.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include <pcl/filters/fast_bilateral.h>


//ROS
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//BOOST
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

// #include "vcacheopt.h"   //Vertex cache optimizer for faster optimizer
// #include "Program.h"     //to easily create  an opengl program with the corresponding shaders

float snan= std::numeric_limits<float>::signaling_NaN();
typedef std::chrono::high_resolution_clock Clock;
#define BUFFER_OFFSET(i) ((char*)NULL +(i))

// export __GL_SYNC_DISPLAY_DEVICE=DFP-1



class ViveKin :  public Vrui::Application,public GLObject {

	public:
    ViveKin(int& argc,char**& argv);
    virtual ~ViveKin(void);
    virtual void frame(void);
    virtual void display(GLContextData& contextData) const;
    virtual void resetNavigation(void);
    virtual void initContext(GLContextData& contextData) const;




    struct DataItem:public GLObject::DataItem{
        public:
//        GLuint VBO_id[NUM_BUFFERS];
//        GLuint indexBufferId[NUM_BUFFERS];
//        GLuint texture[NUM_BUFFERS];

        GLuint m_vbo, m_ibo, m_pbo;

//        GLuint m_vbo_storage;
//        GLuit m_ibo_storage;
//        GLuint m_pbo_sorage;
        GLuint m_texture; //temporary add this texture just to see something

        unsigned int version;


        DataItem(void);
        virtual ~DataItem(void);
    };

    struct Vertex{
        float x;
        float y;
        float z;
        float u;
        float v;

        Vertex() : x(snan), y(snan), z(snan), u(snan), v(snan) {}
        Vertex( float x_, float  y_, float z_, float u_, float v_ ) : x( x_ ), y( y_ ), z( z_ ), u(u_), v(v_)   {}
    };



    /* Elements: */
    private:

    void mesh_cloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud );
    void project_points (const sensor_msgs::CameraInfoConstPtr& cam_info_msg);
    void read_texture(const sensor_msgs::ImageConstPtr& image_msg);
    //TODO doesnt actually need to be mutable. It's just due to the first synthetic triangle we make
    unsigned int m_offset_vbo_rendering;
    mutable unsigned int m_offset_vbo_writing;
    unsigned int m_offset_ibo_rendering;
    mutable unsigned int m_offset_ibo_writing;
    unsigned int m_offset_pbo_rendering;
    mutable unsigned int m_offset_pbo_writing;

    mutable void* m_vbo_ptr;
    mutable void* m_ibo_ptr;
    mutable void* m_pbo_ptr;
    mutable tdogl::Program* m_program = NULL;

    //TODO doesnt actually need to be mutable. It's just due to the first synthetic triangle we make
    mutable unsigned  int m_vbo_bytes_written;
    mutable unsigned int m_ibo_bytes_written;
    mutable unsigned int m_pbo_bytes_written;

    mutable unsigned int m_internal_ibo_offset=0;
    mutable unsigned int m_internal_num_indices=0;


    std::vector <Vertex> m_vertices  ;
    std::vector<GLuint> m_indices ;
    cv::Mat m_img_padded  ;
    int m_num_indices  ;

    mutable boost::mutex m_consume_mtx;

    int processing_counter=0;



    unsigned int version=0; // Version number of mesh in the most-recently locked triple buffer slot
    Threads::Thread read_thread;



    unsigned int upload_vbo();
    unsigned int upload_ibo();
    unsigned int upload_pbo();

  //  void LoadShaders();


    void* read_data(void);
    double linterp ( double input , double input_start, double input_end, double output_start, double output_end);
    //void makeCheckImage(void);
    void saveGLState(void);
    void restoreGLState();
    void SetGLCapability(GLenum capability, GLboolean state);
    void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void callback3msg(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& cam_info_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void kinectCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg);


    GLboolean SavedLighting;
    GLboolean SavedDepthTest;
    GLboolean SavedBlending;
    std::map<std::string, int> GLStateIntegers;


};
