
#include <iostream>

#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/common/angles.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/fill_image.h"
#include <std_msgs/Empty.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <dynamic_reconfigure/server.h>
#include <kinfu/kinfu_Config.h>

//// Libfreenect2
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
//// /Libfreenect2

typedef pcl::ScopeTime ScopeTimeT;

using namespace std;
using namespace pcl;
using namespace pcl::gpu::kinfuLS;
using namespace pcl::gpu;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
namespace pc = pcl::console;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime();
    if (i_ % EACH == 0 && i_)
    {
      cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps)" << endl;
      time_ms_ = 0;
    }
    ++i_;
  }
private:
  int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
void
setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
  viewer.camera_.pos[0] = pos_vector[0];
  viewer.camera_.pos[1] = pos_vector[1];
  viewer.camera_.pos[2] = pos_vector[2];
  viewer.camera_.focal[0] = look_at_vector[0];
  viewer.camera_.focal[1] = look_at_vector[1];
  viewer.camera_.focal[2] = look_at_vector[2];
  viewer.camera_.view[0] = up_vector[0];
  viewer.camera_.view[1] = up_vector[1];
  viewer.camera_.view[2] = up_vector[2];
  viewer.updateCamera ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Affine3f
getViewerPose (visualization::PCLVisualizer& viewer)
{
  Eigen::Affine3f pose = viewer.getViewerPose();
  Eigen::Matrix3f rotation = pose.linear();

  Matrix3f axis_reorder;
  axis_reorder << 0,  0,  1,
                 -1,  0,  0,
                  0, -1,  0;

  rotation = rotation * axis_reorder;
  pose.linear() = rotation;
  return pose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename CloudT> void
writeCloudFile (int format, const CloudT& cloud);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
{
  typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

  pcl::copyPointCloud (points, *merged_ptr);
  for (size_t i = 0; i < colors.size (); ++i)
    merged_ptr->points[i].rgba = colors.points[i].rgba;

  return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
{
  if (triangles.empty())
      return boost::shared_ptr<pcl::PolygonMesh>();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = (int)triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
  pcl::toROSMsg(cloud, mesh_ptr->cloud);

  mesh_ptr->polygons.resize (triangles.size() / 3);
  for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.push_back(i*3+0);
    v.vertices.push_back(i*3+2);
    v.vertices.push_back(i*3+1);
    mesh_ptr->polygons[i] = v;
  }
  return mesh_ptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct CurrentFrameCloudView
{
  void
  setViewerPose (const Eigen::Affine3f& viewer_pose) {
    ::setViewerPose (cloud_viewer_, viewer_pose);
  }

  PointCloud<PointXYZ>::Ptr cloud_ptr_;
  DeviceArray2D<PointXYZ> cloud_device_;
  visualization::PCLVisualizer cloud_viewer_;
};
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ImagePublisher
{
  void
  publishScene (KinfuTracker& kinfu, std_msgs::Header header, bool registration, Eigen::Affine3f* pose_ptr = 0)
  {
//    if (pose_ptr)
//    {
//        raycaster_ptr_->run ( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () ); //says in cmake it does not know it
//        raycaster_ptr_->generateSceneView(view_device_);
//    }
//    else
    {
      kinfu.getImage (view_device_);
    }

 /*   if (paint_image_ && registration && !pose_ptr)
    {
      colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
      paint3DView (colors_device_, view_device_);
    }
*/
    int cols;
    view_device_.download (view_host_, cols);

    //convert image to sensor message
    sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
    sensor_msgs::fillImage((*msg), "rgb8", view_device_.rows(), view_device_.cols(),
    		view_device_.cols() * 3, &view_host_[0]);

    msg->header.frame_id = header.frame_id;
    pubKinfu.publish(msg);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void
  publishPose (KinfuTracker& kinfu, std_msgs::Header header)
  {
	  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = kinfu.getCameraPose().linear();
	  Eigen::Vector3f teVecs = kinfu.getCameraPose().translation();


	  //TODO: start position: init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

	  tf::Transform transform(
	      tf::Matrix3x3(erreMats(0,0),erreMats(0, 1),erreMats(0, 2),
			  erreMats(1,0),erreMats(1, 1),erreMats(1, 2),
			  erreMats(2,0),erreMats(2, 1),erreMats(2, 2)),
	  	  tf::Vector3(teVecs[0], teVecs[1], teVecs[2])
	  );

	  odom_broad.sendTransform(tf::StampedTransform(transform, header.stamp, "/odom", "/kinfu_frame"));
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void
  publishGeneratedDepth (KinfuTracker& kinfu)
  {
	const Eigen::Affine3f& pose= kinfu.getCameraPose();
    raycaster_ptr_->run(kinfu.volume(), pose, kinfu.getCyclicalBufferStructure());
    raycaster_ptr_->generateDepthImage(generated_depth_);

    int c;
    vector<unsigned short> data;
    generated_depth_.download(data, c);

    sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
    sensor_msgs::fillImage(*msg, "bgr8", view_device_.rows(), view_device_.cols(), view_device_.cols() * 3, &view_host_[0]);
    pubgen.publish(msg);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void initPublishers()
  {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubKinfu = it.advertise("/kinect2/kinfuLS/depth", 10);
    pubgen = it.advertise("/kinect2/kinfuLS/generated_depth", 50);
  }


  bool paint_image_;
  bool accumulate_views_;

  KinfuTracker::View view_device_;
  KinfuTracker::View colors_device_;
  vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;

  RayCaster::Ptr raycaster_ptr_;

  KinfuTracker::DepthMap generated_depth_;
  image_transport::Publisher pubKinfu;
  image_transport::Publisher pubgen;
  tf::TransformBroadcaster odom_broad;

};


struct KinFuLSApp
{
  enum
  {
    PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8
  };
  KinFuLSApp(float vsz, float shiftDistance, ros::NodeHandle & nodeHandle) :
      exit_(false), scan_(false), scan_mesh_(false), scan_volume_(false), independent_camera_(false), registration_(
          false), integrate_colors_(false), pcd_source_(false), focal_length_(-1.f), time_ms_(0), nh(nodeHandle)
  {
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);

    ROS_INFO("--- CURRENT SETTINGS ---\n");
    ROS_INFO("Volume size is set to %.2f meters\n", vsz);
    ROS_INFO("Volume will shift when the camera target point is farther than %.2f meters from the volume center\n", shiftDistance);
    ROS_INFO("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6*vsz);
    ROS_INFO("------------------------\n");

    // warning message if shifting distance is abnormally big compared to volume size
    if(shiftDistance > 2.5 * vsz)
    	ROS_WARN("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n", shiftDistance, vsz);

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shiftDistance);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_->setInitialCameraPose (pose);
    kinfu_->volume().setTsdfTruncDist (0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    kinfu_->setDepthTruncationForICP(3.f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);

    //Init KinFuLSApp
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    image_view_.raycaster_ptr_ = RayCaster::Ptr(new RayCaster(kinfu_->rows(), kinfu_->cols()));

//		//TODO: use camera info msg
//		float height = 480.0f;
//		float width = 640.0f;
//		//taken from gpu/kinfu_large_scale/src/internal.h
//		//temporary constant (until we make it automatic) that holds the Kinect's focal length
//		const float FOCAL_LENGTH = 575.816f;
//		screenshot_manager_.setCameraIntrinsics(FOCAL_LENGTH, height, width);

    frame_counter_ = 0;
    enable_texture_extraction_ = false;
    lastNumberOfPoses = 0;
    snapshot_rate_ = 45;

    ros::NodeHandle nh;
    pubKinfuReset = nh.advertise<std_msgs::Empty>("/kinfu_reset", 2);

    //Init ROS publishers
    image_view_.initPublishers();

  }

  ~KinFuLSApp()
  {
  }

  //callback function, called with every new depth topic message
  void execute(const cv::Mat depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
               const sensor_msgs::ImageConstPtr& rgb = sensor_msgs::ImageConstPtr())
  {
    frame_counter_++;

    if ( kinfu_->icpIsLost() )
    {
    	kinfu_->reset();
    	ROS_INFO("KinFu was reset \n");
    	pubKinfuReset.publish(std_msgs::Empty());
   	  // kinfu_->setDisableICP();
    }

    depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
     // if (integrate_colors_)
     //    image_view_.colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);


    /*
     *      [fx  0 cx]
     * K = 	[ 0 fy cy]
     * 		[ 0  0  1]
     */
    (*kinfu_).setDepthIntrinsics(cameraInfo->K[0], cameraInfo->K[4], cameraInfo->K[2], cameraInfo->K[5]);


    float focal_length = (cameraInfo->K[0] + cameraInfo->K[4]) / 2;
    screenshot_manager_.setCameraIntrinsics(focal_length, cameraInfo->height, cameraInfo->width);

    
    SampledScopeTime fps(time_ms_);
    (*kinfu_)(depth_device_);
    if (kinfu_->isFinished())
      nh.shutdown();
    

    if ( image_view_.pubKinfu.getNumSubscribers() > 0)
      image_view_.publishScene (*kinfu_, cameraInfo->header, registration_,  0);
    image_view_.publishPose(*kinfu_, cameraInfo->header);
    image_view_.publishGeneratedDepth(*kinfu_);

    //save snapshots
    if (enable_texture_extraction_)
    {
      if (frame_counter_ % snapshot_rate_ == 0)
      {
        //convert sensor_msgs::Image to pcl::gpu::PixelRGB
        unsigned pixelCount = rgb->height * rgb->width;
        pcl::gpu::kinfuLS::PixelRGB * pixelRgbs = new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
        for (unsigned i = 0; i < pixelCount; i++)
        {
          //the encoding given in the image is "bgr8"
          pixelRgbs[i].b = rgb->data[i * 3];
          pixelRgbs[i].g = rgb->data[i * 3 + 1];
          pixelRgbs[i].r = rgb->data[i * 3 + 2];
        }
        pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(rgb->height, rgb->width, pixelRgbs, rgb->step);
        screenshot_manager_.saveImage(kinfu_->getCameraPose(), rgb24);
        delete[] pixelRgbs;
      }
    }
  }

  void reconfCallback(kinfu::kinfu_Config & c, uint32_t level)
  {
    if (c.stop)
      kinfu_->performLastScan();
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  bool exit_;
  bool scan_;
  bool scan_mesh_;
  bool scan_volume_;

  bool independent_camera_;
  int frame_counter_;
  bool enable_texture_extraction_;

  bool registration_;
  bool integrate_colors_;
  bool pcd_source_;
  float focal_length_;

  KinfuTracker *kinfu_;

  ImagePublisher image_view_;

  KinfuTracker::DepthMap depth_device_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  boost::mutex data_ready_mutex_;
  boost::condition_variable data_ready_cond_;

  std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
  std::vector<unsigned short> source_depth_data_;
  PtrStepSz<const unsigned short> depth_;
  PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;

  int time_ms_;
  vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;
  KinfuTracker::DepthMap generated_depth_;
  size_t lastNumberOfPoses;
  ros::Publisher pubKinfuReset;

  pcl::kinfuLS::ScreenshotManager screenshot_manager_;
  int snapshot_rate_;

  //the ros node handle used to shut down the node and stop capturing
  ros::NodeHandle & nh;
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int print_cli_help()
{
  cout << "\nKinFu parameters:" << endl;
  cout << "    --help, -h                        : print this message" << endl;
  cout << "\nkinfuLS node parameters:" << endl;
  cout << "    volume_size <in_meters>, vs       : define integration volume size" << endl;
  cout << "    shifting_distance <in_meters>, sd : define shifting threshold (distance target-point / cube center)"
      << endl;
  cout << "    snapshot_rate <X_frames>, sr      : Extract RGB textures every <X_frames>. Default: 45" << endl;
  cout << "    extract_textures, et              : extract RGB PNG images to KinFuSnapshots folder. Default: true"
      << endl;

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	// check arguments
  if (pc::find_switch(argc, argv, "--help") || pc::find_switch(argc, argv, "-h"))
    return print_cli_help();

  ros::init(argc, argv, "kinfuLS");
  ros::NodeHandle nh("~");

  // assign value from parameter server, with default.
  int device;
  nh.param<int>("device", device, 0);
  pcl::gpu::setDevice(device);
  pcl::gpu::printShortCudaDeviceInfo(device);

  double volume_size = 3.0; //pcl::device::VOLUME_SIZE
  nh.getParam("volume_size", volume_size);
  nh.getParam("vs", volume_size);

  double shift_distance = 1.5; //pcl::device::DISTANCE_THRESHOLD;
  nh.getParam("shift_distance", shift_distance);
  nh.getParam("sd", shift_distance);

  KinFuLSApp app(volume_size, shift_distance, nh);

  //default value (45) is set in constructor of KinFuLSApp
  nh.getParam("snapshot_rate", app.snapshot_rate_);
  nh.getParam("sr", app.snapshot_rate_);

  //default value (false) is set in constructor of KinFuLSApp
  nh.getParam("extract_textures", app.enable_texture_extraction_);
  nh.getParam("et", app.enable_texture_extraction_);


  // // message_filters instead of image_transport because of synchronization over w-lan
  // typedef sync_policies::ApproximateTime<Image, CameraInfo, Image> DRGBSync;
  // message_filters::Synchronizer<DRGBSync>* texture_sync;
  // TimeSynchronizer<Image, CameraInfo>* depth_only_sync;

  // message_filters::Subscriber<Image>* rgb_sub;
  // message_filters::Subscriber<Image>* depth_sub;
  // message_filters::Subscriber<CameraInfo>* info_sub;

  // if (app.enable_texture_extraction_)
  // {
  //   depth_sub = new message_filters::Subscriber<Image>(nh, "/kinect2/depth/image", 2);
  //   info_sub  = new message_filters::Subscriber<CameraInfo>(nh, "/kinect2/depth/camera_info", 2);
  //   rgb_sub   = new message_filters::Subscriber<Image>(nh, "/kinect2/rgb/image", 2);

  //   //the depth and the rgb cameras are not hardware synchronized
  //   //hence the depth and rgb images normally do not have the EXACT timestamp
  //   //so use approximate time policy for synchronization
  //   texture_sync = new message_filters::Synchronizer<DRGBSync>(DRGBSync(500), *depth_sub, *info_sub, *rgb_sub);
  //   texture_sync->registerCallback(boost::bind(&KinFuLSApp::execute, &app, _1, _2, _3));
  //   ROS_INFO("Running KinFu with texture extraction");
  // }
  // else
  // {
  //   depth_sub = new message_filters::Subscriber<Image>(nh, "/kinect2/depth/image", 1);
  //   info_sub  = new message_filters::Subscriber<CameraInfo>(nh, "/kinect2/depth/camera_info", 1);

  //   depth_only_sync = new TimeSynchronizer<Image, CameraInfo>(*depth_sub, *info_sub, 500);
  //   depth_only_sync -> registerCallback(boost::bind(&KinFuLSApp::execute, &app, _1, _2, sensor_msgs::ImageConstPtr()));
  //   ROS_INFO("Running KinFu without texture extraction");
  // }

  dynamic_reconfigure::Server<kinfu::kinfu_Config> reconfServer(nh);
  reconfServer.setCallback(boost::bind(&KinFuLSApp::reconfCallback, &app, _1, _2));

  //// Libfreenect2
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline();
  dev = freenect2.openDefaultDevice(pipeline);
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  //// /Libfreenect2

  sensor_msgs::CameraInfo info;
  // sensor_msgs::Image msgImage;
  // sensor_msgs::ImagePtr imageptr (&msgImage);
  sensor_msgs::CameraInfoPtr cameraptr (&info);
  ros::Rate loop_rate(40);
  while (nh.ok())
  {
    ros::spinOnce();
    //// Libfreenect2
    
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    cv::Mat depthIm(depth->height, depth->width, CV_32FC1, depth->data);
    cv::Mat depthMat;
    depthIm.convertTo(depthMat, CV_16U);
    cv::flip(depthMat, depthMat, 1);
    
    info.header.stamp = ros::Time::now();
    info.header.frame_id = "kinect2_ir_optical_frame";
    info.height = 424;
    info.width = 512;
    info.distortion_model = "plumb_bob";
    info.D = {0.09965109080076218, -0.26981210708618164, 0.0, 0.0, 0.0853145569562912};
    info.K = {364.6803894042969, 0.0, 256.4530944824219, 0.0, 364.6803894042969, 209.7216033935547, 0.0, 0.0, 1.0};
    info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.P = {364.6803894042969, 0.0, 256.4530944824219, 0.0, 0.0, 364.6803894042969, 209.7216033935547, 0.0, 0.0, 0.0, 1.0, 0.0};
  
    app.execute(depthMat, cameraptr);
    
    listener.release(frames);
    loop_rate.sleep();
  }
  dev->stop();
  dev->close();

  return 0;
}
