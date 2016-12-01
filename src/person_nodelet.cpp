/******************************************************************************
 Copyright (c) 2016, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <realsense_person/person_nodelet.h>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(realsense_person::PersonNodelet, nodelet::Nodelet)

namespace realsense_person
{
  /*
   * Nodelet constructor.
   */
  PersonNodelet::PersonNodelet()
  {
    pt_video_module_.reset(PersonModuleInterface::person_tracking_video_module_factory::
        create_person_tracking_video_module(PERSON_MODULE_DATA_PATH));
    projection_interface_ = nullptr;
    caminfo_disconnected_ = false;
    tracking_id_ = -1;
    current_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_tracking_time_ = 0.0;
    publish_detection_ = false;
    publish_detection_image_ = false;
    publish_tracking_ = false;
    publish_tracking_image_ = false;
  }

  /*
   * Nodelet destructor.
   */
  PersonNodelet::~PersonNodelet()
  {
    if (projection_interface_ != nullptr)
    {
      projection_interface_->release();
    }
  }

  /*
   * Initialize Nodelet.
   */
  void PersonNodelet::onInit()
  {
    getStaticParameters();
    startDynamicReconfServer();
    definePubSub();
    subscribeToCamInfoTopics();
    advertiseServices();
  }

  /*
   * Get static Parameters.
   */
  void PersonNodelet::getStaticParameters()
  {
    nodelet_name_ = getName();
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    pnh_.param("subscribe_rate", subscribe_rate_, SUBSCRIBE_RATE);
  }

  /*
   * Start the Dynamic Reconfigure Server.
   */
  void PersonNodelet::startDynamicReconfServer()
  {
    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_person::person_paramsConfig>(pnh_));
    dynamic_reconf_server_->setCallback(boost::bind(&PersonNodelet::dynamicReconfCallback, this, _1, _2));
  }

  /*
   * Get dynamic Parameters.
   */
  void PersonNodelet::dynamicReconfCallback(realsense_person::person_paramsConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Setting detection rate to " << config.detection_rate);
    detection_rate_ = config.detection_rate;

    ROS_INFO_STREAM(nodelet_name_ << " - Setting tracking rate to " << config.tracking_rate);
    tracking_rate_ = config.tracking_rate;

    if ((config.enable_recognition) &&
        (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling recognition");
      pt_video_module_->QueryConfiguration()->QueryRecognition()->Enable();
    }
    else if ((!config.enable_recognition) &&
        (pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling recognition");
      pt_video_module_->QueryConfiguration()->QueryRecognition()->Disable();
    }

/*  TODO: Enable modifying these parameters once they are implemented.
    if ((config.enable_skeleton_joints) &&
        (!pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling skeleton joints");
      pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->Enable();
    }
    else if ((!config.enable_skeleton_joints) &&
        (pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling skeleton joints");
      pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->Disable();
    }

    if ((config.enable_gestures) &&
        (!pt_video_module_->QueryConfiguration()->QueryGestures()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling gestures");
      pt_video_module_->QueryConfiguration()->QueryGestures()->Enable();
    }
    else if ((!config.enable_gestures) &&
        (pt_video_module_->QueryConfiguration()->QueryGestures()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling gestures");
      pt_video_module_->QueryConfiguration()->QueryGestures()->Disable();
    }

    if ((config.enable_orientation) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsPersonOrientationEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling orientation");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnablePersonOrientation();
    }
    else if ((!config.enable_orientation) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsPersonOrientationEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling orientation");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisablePersonOrientation();
    }

    if ((config.enable_head_pose) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadPoseEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling head pose");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableHeadPose();
    }
    else if ((!config.enable_head_pose) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadPoseEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling head pose");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisableHeadPose();
    }

    if ((config.enable_head_bounding_box) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadBoundingBoxEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling head bounding box");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableHeadBoundingBox();
    }
    else if ((!config.enable_head_bounding_box) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadBoundingBoxEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling head bounding box");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisableHeadBoundingBox();
    }

    if ((config.enable_face_landmarks) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsFaceLandmarksEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling face landmarks");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableFaceLandmarks();
    }
    else if ((!config.enable_face_landmarks) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsFaceLandmarksEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling face landmarks");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisableFaceLandmarks();
    }
*/
  }

  /*
   * Define Publishers and Subscribers.
   */
  void PersonNodelet::definePubSub()
  {
    ros::NodeHandle color_nh(nh_, COLOR_NAMESPACE);
    ros::NodeHandle depth_nh(nh_, DEPTH_NAMESPACE);
    ros::NodeHandle person_nh(nh_, PERSON_NAMESPACE);

    color_caminfo_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
        (new message_filters::Subscriber<sensor_msgs::CameraInfo>(color_nh, CAMERA_INFO_TOPIC, 1));
    depth_caminfo_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
        (new message_filters::Subscriber<sensor_msgs::CameraInfo>(depth_nh, CAMERA_INFO_TOPIC, 1));

    color_image_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
        (new message_filters::Subscriber<sensor_msgs::Image>(color_nh, COLOR_TOPIC, 1));
    depth_image_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
        (new message_filters::Subscriber<sensor_msgs::Image>(depth_nh, DEPTH_TOPIC, 1));

    detection_pub_ = person_nh.advertise<realsense_person::PersonDetection>(DETECTION_TOPIC, 1);
    detection_image_pub_ = person_nh.advertise<sensor_msgs::Image>(DETECTION_IMAGE_TOPIC, 1);

/*  TODO: Advertise these topics once they are implemented in the next release.
    tracking_pub_ = person_nh.advertise<realsense_person::PersonTracking>(TRACKING_TOPIC, 1);
    tracking_image_pub_ = person_nh.advertise<sensor_msgs::Image>(TRACKING_IMAGE_TOPIC, 1);
*/
  }

  /*
   * Subscribe to Camera Info Topics.
   */
  void PersonNodelet::subscribeToCamInfoTopics()
  {
    caminfo_tsync_ =
        std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>>
        (new message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
        (*color_caminfo_sub_, *depth_caminfo_sub_, 40));
    caminfo_connection_ = message_filters::Connection(caminfo_tsync_->registerCallback(
        boost::bind(&PersonNodelet::caminfoCallback, this, _1, _2)));
  }

  /*
   * Advertise Services.
   */
  void PersonNodelet::advertiseServices()
  {
    tracking_id_server_ = pnh_.advertiseService(realsense_person::GET_TRACKING_ID_SERVICE,
        &PersonNodelet::getTrackingIdServiceHandler, this);
    register_server_ = pnh_.advertiseService(realsense_person::REGISTER_SERVICE,
        &PersonNodelet::registerServiceHandler, this);
    recognize_server_ = pnh_.advertiseService(realsense_person::RECOGNIZE_SERVICE,
        &PersonNodelet::recognizeServiceHandler, this);
    reinforce_server_ = pnh_.advertiseService(realsense_person::REINFORCE_SERVICE,
        &PersonNodelet::reinforceServiceHandler, this);

/*  TODO: Adverstise these services once they are implemented in the next release.
    start_tracking_server_ = pnh_.advertiseService(realsense_person::START_TRACKING_SERVICE,
        &PersonNodelet::startTrackingServiceHandler, this);
    stop_tracking_server_ = pnh_.advertiseService(realsense_person::STOP_TRACKING_SERVICE,
        &PersonNodelet::stopTrackingServiceHandler, this);
    serialize_server_ = pnh_.advertiseService(realsense_person::SERIALIZE_SERVICE,
        &PersonNodelet::serializeServiceHandler, this);
    deserialize_server_ = pnh_.advertiseService(realsense_person::DESERIALIZE_SERVICE,
        &PersonNodelet::deserializeServiceHandler, this);
*/
  }

  /*
   * Callback for Camera Info Topics.
   */
  void PersonNodelet::caminfoCallback(const sensor_msgs::CameraInfoConstPtr& color_caminfo,
      const sensor_msgs::CameraInfoConstPtr& depth_caminfo)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - color_caminfo:" << " width = " << color_caminfo->width <<
        " height = " << color_caminfo->height);
    ROS_INFO_STREAM(nodelet_name_ << " - depth_caminfo:" << " width = " << depth_caminfo->width <<
        " height = " << depth_caminfo->height);

    RSCore::intrinsics color_intrinsics = {};
    RSCore::intrinsics depth_intrinsics = {};
    RSCore::extrinsics extrinsics = {};
    memset(&color_intrinsics, 0, sizeof(color_intrinsics));
    memset(&depth_intrinsics, 0, sizeof(depth_intrinsics));
    memset(&extrinsics, 0, sizeof(extrinsics));

    getImageIntrinsics(color_caminfo, color_intrinsics);
    getImageIntrinsics(depth_caminfo, depth_intrinsics);
    getImageExtrinsics(depth_caminfo, extrinsics);

    RSCore::video_module_interface::actual_module_config module_config = {};

    setModuleConfig(static_cast<int>(RSCore::stream_type::color), color_intrinsics, module_config);
    setModuleConfig(static_cast<int>(RSCore::stream_type::depth), depth_intrinsics, module_config);

    module_config.projection = RSCore::projection_interface::create_instance(&color_intrinsics,
        &depth_intrinsics, &extrinsics);

    projection_interface_ = module_config.projection;
    pt_video_module_->set_module_config(module_config);

    ROS_INFO_STREAM(nodelet_name_ << " - Subscribing to image topics...");
    subscribeToImageTopics();
  }

  /*
   * Extract Intrinsics from Camera Info Message.
   */
  void PersonNodelet::getImageIntrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo,
      RSCore::intrinsics& intrinsics)
  {
    intrinsics.width = caminfo->width;
    intrinsics.height = caminfo->height;
    intrinsics.fx = caminfo->K[0];
    intrinsics.fy = caminfo->K[4];
    intrinsics.ppx = caminfo->K[2];
    intrinsics.ppy = caminfo->K[5];
  }

  /*
   * Extract Extrinsics from Camera Info Message.
   */
  void PersonNodelet::getImageExtrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo,
      RSCore::extrinsics& extrinsics)
  {
    extrinsics.translation[0] = caminfo->P[3];
    extrinsics.translation[1] = caminfo->P[7];
    extrinsics.translation[2] = caminfo->P[11];
    for (int i = 0; i < 9; ++i)
    {
      extrinsics.rotation[i] = caminfo->R[i];
    }
  }

  /*
   * Set Video Module Interface configuration.
   */
  void PersonNodelet::setModuleConfig(int image_type, RSCore::intrinsics intrinsics,
      RSCore::video_module_interface::actual_module_config& module_config)
  {
    module_config.image_streams_configs[image_type].size.height = intrinsics.height;
    module_config.image_streams_configs[image_type].size.width = intrinsics.width;
    module_config.image_streams_configs[image_type].frame_rate = subscribe_rate_;
    module_config.image_streams_configs[image_type].is_enabled = true;
  }

  /*
   * Subscribe to Image Topics.
   */
  void PersonNodelet::subscribeToImageTopics()
  {
    image_timesync_ = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
        (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
        (*color_image_sub_, *depth_image_sub_, 40));
    image_timesync_->registerCallback(boost::bind(&PersonNodelet::imageCallback, this, _1, _2));
  }

  /*
   * Callback for Image Topics.
   */
  void PersonNodelet::imageCallback(const sensor_msgs::ImageConstPtr& color_image,
      const sensor_msgs::ImageConstPtr& depth_image)
  {
    RSCore::correlated_sample_set sample_set;
    generateSampleSet(RSCore::stream_type::color, RSCore::pixel_format::rgb8, color_image, sample_set);
    generateSampleSet(RSCore::stream_type::depth, RSCore::pixel_format::z16, depth_image, sample_set);

    processFrame(sample_set);

    current_time_ = (std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count() / 1000.00);
    int current_detection_rate = 1.0 / (double)(current_time_ - last_detection_time_);
    int current_tracking_rate = 1.0 / (double)(current_time_ - last_tracking_time_);

    publish_detection_ = ((detection_pub_.getNumSubscribers() > 0) &&
        (current_detection_rate <= detection_rate_));
    publish_detection_image_ = ((detection_image_pub_.getNumSubscribers() > 0) &&
        (current_detection_rate <= detection_rate_));
    publish_tracking_ = ((tracking_pub_.getNumSubscribers() > 0) &&
        (current_tracking_rate <= tracking_rate_));
    publish_tracking_image_ = ((tracking_image_pub_.getNumSubscribers() > 0) &&
        (current_tracking_rate <= tracking_rate_));

    if (publish_detection_ || publish_detection_image_ || publish_tracking_ || publish_tracking_image_)
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        ROS_WARN_STREAM(nodelet_name_ << " - Could not get person data");
      }
      else
      {
        prepareMsgs(person_data, color_image, current_time_);
      }
    }

    // Disconnect Camera Info callback after it has been processed once
    if (caminfo_disconnected_ == false)
    {
      caminfo_connection_.disconnect();
      caminfo_disconnected_ = true;
    }
  }

  /*
   * Generate sample set.
   */
  void PersonNodelet::generateSampleSet(RSCore::stream_type image_type, RSCore::pixel_format image_format,
      const sensor_msgs::ImageConstPtr& image, RSCore::correlated_sample_set &sample_set)
  {
    RSCore::image_info image_info = {static_cast<int32_t>(image->width), static_cast<int32_t>(image->height),
        image_format, image->step};
    sample_set[image_type] = RSCore::image_interface::create_instance_from_raw_data(&image_info,
        RSCore::image_interface::image_data_with_data_releaser(image->data.data(), nullptr),
        RSCore::stream_type::color, RSCore::image_interface::flag::any, 0, 0, nullptr);
  }

  /*
   * Process Frame.
   */
  void PersonNodelet::processFrame(RSCore::correlated_sample_set sample_set)
  {
    std::unique_lock<std::mutex> lock(frame_lock_);
    auto status = pt_video_module_->process_sample_set_sync(&sample_set);

    if (status != RSCore::status::status_no_error)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Error while processing input frames: " << status);
    }
  }

  /*
   * Get Person Data.
   */
  PersonModule::PersonTrackingData* PersonNodelet::getPersonData()
  {
    std::unique_lock<std::mutex> lock(frame_lock_);
    return pt_video_module_->QueryOutput();
  }

  /*
   * Prepare and publish messages.
   */
  void PersonNodelet::prepareMsgs(PersonModule::PersonTrackingData* person_data,
      const sensor_msgs::ImageConstPtr& color_image, double msg_received_time)
  {
    PersonDetection detection_msg;
    PersonTracking tracking_msg;
    ros::Time header_stamp = ros::Time::now();

    int detected_person_cnt = person_data->QueryNumberOfPeople();
    if (detected_person_cnt > 0)
    {
      for (int i = 0; i < detected_person_cnt; ++i)
      {
        PersonModule::PersonTrackingData::Person* single_person_data =
            person_data->QueryPersonData(PersonModule::PersonTrackingData::ACCESS_ORDER_BY_ID, i);
        PersonModule::PersonTrackingData::PersonTracking* detection_data = single_person_data->QueryTracking();
        int tracking_id = detection_data->QueryId();
        PersonModule::PersonTrackingData::BoundingBox2D b_box = detection_data->Query2DBoundingBox();
        PersonModule::PersonTrackingData::PointCombined com = detection_data->QueryCenterMass();
        Person person_msg = preparePersonMsg(tracking_id, b_box, com);

        if (publish_detection_ || publish_detection_image_)
        {
          last_detection_time_ = msg_received_time;
          detection_msg.persons.push_back(person_msg);
        }

        if ((publish_tracking_ || publish_tracking_image_) && (tracking_id_ == tracking_id))
        {
          last_tracking_time_ = msg_received_time;
          tracking_msg.person = person_msg;
          tracking_msg.person_face = preparePersonFaceMsg(tracking_id_, detection_data, single_person_data);
          tracking_msg.person_body = preparePersonBodyMsg(tracking_id_, single_person_data);
        }
      }
    }

    // Publish Detection message
    if (publish_detection_)
    {
      detection_msg.header.stamp = header_stamp;
      detection_msg.header.frame_id = DETECTION_FRAME_ID;
      detection_msg.detected_person_count = detected_person_cnt;
      detection_pub_.publish(detection_msg);
    }

    // Publish Tracking message
    if (publish_tracking_)
    {
      tracking_msg.header.stamp = header_stamp;
      tracking_msg.header.frame_id = TRACKING_FRAME_ID;
      tracking_pub_.publish(tracking_msg);
    }

    // Publish Detection Image message
    if (publish_detection_image_)
    {
      try
      {
        cv_bridge::CvImagePtr detection_cv_img_ptr;
        detection_cv_img_ptr = cv_bridge::toCvCopy(color_image);
        prepareDetectionImageMsg(detection_msg, detection_cv_img_ptr);
        sensor_msgs::ImagePtr detection_image_msg = detection_cv_img_ptr->toImageMsg();
        detection_image_msg->header.stamp = header_stamp;
        detection_image_msg->header.frame_id = DETECTION_IMAGE_FRAME_ID;
        detection_image_pub_.publish(detection_image_msg);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM(nodelet_name_ << " - Error converting color_msg to cv: " << e.what());
      }
    }

    // Publish Tracking Image message
    if (publish_tracking_image_)
    {
      try
      {
        cv_bridge::CvImagePtr tracking_cv_img_ptr;
        tracking_cv_img_ptr = cv_bridge::toCvCopy(color_image);
        prepareTrackingImageMsg(tracking_msg, tracking_cv_img_ptr);
        sensor_msgs::ImagePtr tracking_image_msg = tracking_cv_img_ptr->toImageMsg();
        tracking_image_msg->header.stamp = header_stamp;
        tracking_image_msg->header.frame_id = TRACKING_IMAGE_FRAME_ID;
        tracking_image_pub_.publish(tracking_image_msg);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM(nodelet_name_ << " - Error converting color_msg to cv: " << e.what());
      }
    }
  }

  /*
   * Prepare Person Message.
   */
  Person PersonNodelet::preparePersonMsg(int tracking_id, PersonModule::PersonTrackingData::BoundingBox2D b_box,
      PersonModule::PersonTrackingData::PointCombined com)
  {
    Person person_msg;
    PersonId person_id_msg;
    BoundingBox b_box_msg;
    RegisteredPoint com_msg;

    person_id_msg.tracking_id = tracking_id;
    person_id_msg.recognition_id = -1; // Default value of recognition_id

    b_box_msg.x = b_box.rect.x;
    b_box_msg.y = b_box.rect.y;
    b_box_msg.w = b_box.rect.w;
    b_box_msg.h = b_box.rect.h;
    b_box_msg.confidence = b_box.confidence;

    com_msg.world.x = com.world.point.x;
    com_msg.world.y = com.world.point.y;
    com_msg.world.z = com.world.point.z;
    com_msg.world_confidence = com.world.confidence;
    com_msg.image.x = com.image.point.x;
    com_msg.image.y = com.image.point.y;
    com_msg.image.z = com.image.point.z;
    com_msg.image_confidence = com.image.confidence;

    person_msg.person_id = person_id_msg;
    person_msg.bounding_box = b_box_msg;
    person_msg.center_of_mass = com_msg;
    return person_msg;
  }

  /*
   * Prepare DetectionImage Message.
   */
  void PersonNodelet::prepareDetectionImageMsg(PersonDetection detection_msg, cv_bridge::CvImagePtr& cv_ptr)
  {
    auto color = cv::Scalar(0, 255, 0); // green
    for (Person person_msg: detection_msg.persons)
    {
      cv::rectangle(cv_ptr->image, cv::Rect(person_msg.bounding_box.x, person_msg.bounding_box.y,
          (person_msg.bounding_box.x + person_msg.bounding_box.w),
          (person_msg.bounding_box.y + person_msg.bounding_box.h)), color, 3);
      cv::circle(cv_ptr->image, cv::Point(person_msg.center_of_mass.image.x,
          person_msg.center_of_mass.image.y), 5, color, 3);

      std::stringstream id;
      id << "tid: " << person_msg.person_id.tracking_id;

      /* TODO: Get the real recognition id from the db once that feature is implemented in the next release.
      if (person_msg.person_id.recognition_id >= 0)
      {
        id << ", rid: " << person_msg.person_id.recognition_id;
      }
      */

      cv::putText(cv_ptr->image, id.str(), cv::Point(person_msg.bounding_box.x, person_msg.bounding_box.y),
          cv::FONT_HERSHEY_PLAIN, 3, color);
    }
  }

  /*
   * Prepare PersonFace Message.
   */
  PersonFace PersonNodelet::preparePersonFaceMsg(int tracking_id,
      PersonModule::PersonTrackingData::PersonTracking* detection_data,
      PersonModule::PersonTrackingData::Person* single_person_data)
  {
    PersonFace person_face_msg;
    //TODO: Implement the logic to populate the person_face_msg in the next release.
    return person_face_msg;
  }

  /*
   * Prepare PersonBody Message.
   */
  PersonBody PersonNodelet::preparePersonBodyMsg(int tracking_id,
      PersonModule::PersonTrackingData::Person* single_person_data)
  {
    PersonBody person_body_msg;
    //TODO: Implement the logic to populate the person_body_msg in the next release.
    return person_body_msg;
  }

  /*
   * Prepare TrackingImage Message.
   */
  void PersonNodelet::prepareTrackingImageMsg(PersonTracking tracking_msg, cv_bridge::CvImagePtr& cv_ptr)
  {
    //TODO: Implement the logic to populate the tracking image in the next release.
  }

  /*
   * Handle GetTrackingId Service call.
   */
  bool PersonNodelet::getTrackingIdServiceHandler(realsense_person::GetTrackingId::Request &req,
      realsense_person::GetTrackingId::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << GET_TRACKING_ID_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.status = -1;
      res.status_desc = "Could not get person data";
    }
    else
    {
      res.status = 0;
      res.status_desc = "Success";
      int detected_person_cnt = person_data->QueryNumberOfPeople();
      res.detected_person_count = detected_person_cnt;
      for (int i = 0; i < detected_person_cnt; ++i)
      {
        PersonModule::PersonTrackingData::Person* single_person_data =
            person_data->QueryPersonData(PersonModule::PersonTrackingData::ACCESS_ORDER_BY_ID, i);
        PersonModule::PersonTrackingData::PersonTracking* detection_data = single_person_data->QueryTracking();
        int tracking_id = detection_data->QueryId();
        res.tracking_ids.push_back(tracking_id);
      }
    }
    return true;
  }

  /*
   * Handle StartTracking Service call.
   */
  bool PersonNodelet::startTrackingServiceHandler(realsense_person::StartTracking::Request &req,
      realsense_person::StartTracking::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << START_TRACKING_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.status = -1;
      res.status_desc = "Could not get person_data";
    }
    else
    {
      tracking_id_ = req.tracking_id;
      pt_video_module_->QueryConfiguration()->QueryTracking()->Enable();
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableFaceLandmarks();
      person_data->StartTracking(tracking_id_);
      res.status = 0;
      res.status_desc = "Started tracking person with tracking_id " + std::to_string(tracking_id_);
    }
    return true;
  }

  /*
   * Handle StopTracking Service call.
   */
  bool PersonNodelet::stopTrackingServiceHandler(realsense_person::StopTracking::Request &req,
      realsense_person::StopTracking::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << STOP_TRACKING_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.status = -1;
      res.status_desc = "Could not get person data";
    }
    else
    {
      person_data->StopTracking(tracking_id_);
      res.status = 0;
      res.status_desc = "Stopped tracking person with tracking_id " + std::to_string(tracking_id_);
    }
    return true;
  }

  /*
   * Handle Register Service call.
   */
  bool PersonNodelet::registerServiceHandler(realsense_person::Register::Request &req,
      realsense_person::Register::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << REGISTER_SERVICE);
    res.status = -1;
    res.recognition_id = -1;
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status_desc = "Recognition Not Enabled";
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status_desc = "Tracking Id Not Found";
        }
        else
        {
          int recognition_id = -1;
          res.status = person->QueryRecognition()->RegisterUser(&recognition_id);
          res.status_desc = REGISTRATION_DESC[res.status];
          res.recognition_id = recognition_id;
        }
      }
    }
    return true;
  }

  /*
   * Handle Recognize Service call.
   */
  bool PersonNodelet::recognizeServiceHandler(realsense_person::Recognize::Request &req,
      realsense_person::Recognize::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << RECOGNIZE_SERVICE);
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status = -1;
      res.status_desc = "Recognition Not Enabled";
      res.recognition_id = -1;
      res.confidence = -1.0;
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status = -1;
          res.status_desc = "Tracking Id Not Found";
          res.recognition_id = -1;
          res.confidence = -1.0;
        }
        else
        {
          PersonModule::PersonTrackingData::RecognizerData result;
          res.status = person->QueryRecognition()->RecognizeUser(&result);
          res.status_desc = RECOGNITION_DESC[res.status];
          res.recognition_id = result.recognitionId;
          res.confidence = result.similarityScore;
        }
      }
    }
    return true;
  }

  /*
   * Handle Reinforce Service call.
   */
  bool PersonNodelet::reinforceServiceHandler(realsense_person::Reinforce::Request &req,
      realsense_person::Reinforce::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << REINFORCE_SERVICE);
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status = -1;
      res.status_desc = "Recognition Not Enabled";
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status = -1;
          res.status_desc = "Tracking Id Not Found";
        }
        else
        {
          res.status = person->QueryRecognition()->ReinforceUserRegistration(req.recognition_id,
              PersonModule::PersonTrackingData::PersonRecognition::RegisterPolicyManualAdd);
          res.status_desc = REGISTRATION_DESC[res.status];
        }
      }
    }
    return true;
  }

  /*
   * Handle Serialize Service call.
   */
  bool PersonNodelet::serializeServiceHandler(realsense_person::Serialize::Request &req,
        realsense_person::Serialize::Response &res)
  {
    //TODO: Implement the logic in the next release.
    return true;
  }

  /*
   * Handle Deserialize Service call
   */
  bool PersonNodelet::deserializeServiceHandler(realsense_person::Deserialize::Request &req,
        realsense_person::Deserialize::Response &res)
  {
    //TODO: Implement the logic in the next release.
    return true;
  }

}

