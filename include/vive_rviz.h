#include <omp.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <limits>
#include <map>

//Vrui
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>
#include <Threads/Thread.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <GL/gl.h>
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
//#include <pcl/surface/organized_fast_mesh.h>
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

float snan= std::numeric_limits<float>::signaling_NaN();
typedef std::chrono::high_resolution_clock Clock;



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
        GLuint VBO_id[NUM_BUFFERS];
        GLuint indexBufferId[NUM_BUFFERS];
        GLuint texture[NUM_BUFFERS];

        GLuint persistent_vbo;
        GLuint persistent_pbo;

        bool m_rendering_buffer_0=true;
        unsigned int version[NUM_BUFFERS];

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

    int writer_idx=0;
    mutable int reader_idx=0;
    int last_wrote_idx=0;
    std::vector <std::vector <Vertex>  > vertices  ;
    std::vector <std::vector<GLuint> > indices ;
    std::vector <cv::Mat> img_padded  ;
    std::vector<int> num_indices  ;

    mutable boost::mutex consume_mtx;


    int buf_idx=0;


    mutable GLContextData* contextGlobal;


    int kinect_callback_counter=0;
    int camera_callback_counter=0;
    int img_callback_counter=0;
    int processing_counter=0;


    //GLubyte checkImage[checkImageHeight][checkImageWidth][4];
    unsigned int version=0; // Version number of mesh in the most-recently locked triple buffer slot
    Threads::Thread read_thread;
    GLuint streamOffset = 0;
    GLuint drawOffset   = 0;
    mutable bool m_data_available=false;

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
