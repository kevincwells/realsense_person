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

#include <iostream>

#pragma once
#ifndef NODELET_CONSTANTS
#define NODELET_CONSTANTS

namespace realsense_person
{
  // Default Constants.
  const int COLOR_FPS = 30;
  const int DEPTH_FPS = 30;

  const std::string COLOR_NAMESPACE = "color";
  const std::string COLOR_TOPIC = "image_raw";
  const std::string DEPTH_NAMESPACE = "depth";
  const std::string DEPTH_TOPIC = "image_raw";
  const std::string CAMERA_INFO_TOPIC = "camera_info";
  const std::string PERSON_NAMESPACE = "person";

  const std::string DETECTION_TOPIC = "detection_data";
  const std::string DETECTION_FRAME_ID = "detection_data";

  const std::string DETECTION_IMAGE_TOPIC = "detection_image";
  const std::string DETECTION_IMAGE_FRAME_ID = "detection_image";

  const std::string TRACKING_TOPIC = "tracking_data";
  const std::string TRACKING_FRAME_ID = "tracking_data";

  const std::string TRACKING_IMAGE_TOPIC = "tracking_image";
  const std::string TRACKING_IMAGE_FRAME_ID = "tracking_image";

  const std::string GET_TRACKING_ID_SERVICE = "get_tracking_ids";
  const std::string REGISTER_SERVICE = "register_person";
  const std::string RECOGNIZE_SERVICE = "recognize_person";
  const std::string REINFORCE_SERVICE = "reinforce_person";
  const std::string START_TRACKING_SERVICE = "start_tracking_person";
  const std::string STOP_TRACKING_SERVICE = "stop_tracking";
  const std::string SERIALIZE_SERVICE = "serialize_db";
  const std::string DESERIALIZE_SERVICE = "deserialize_db";

  const std::string REGISTRATION_DESC[7] = {"Registration Successful", "Failed", "Already Registered",
      "Face Not Detected", "Face Not Clear", "Person Too Far", "Person Too Close"};
  const std::string RECOGNITION_DESC[8] = {"Recognition Successful", "Not Recognized", "Failed", "Face Not Detected",
      "Face Not Clear", "Person Too Far", "Person Too Close", "Face Ambiguity"};
  const std::string ORIENTATION_DESC[6] = {"Frontal", "45 degree right", "45 degree left", "Right", "Left", "Rear"};
  const std::string JOINT_TYPE_DESC[25] = {"joint ankle left", "joint ankle right", "joint elbow left",
      "joint elbow right", "joint foot left", "joint foot right", "joint hand left", "joint hand right",
      "joint hand tip left", "joint hand tip right", "joint head", "joint hip left", "joint hip right",
      "joint knee left", "joint knee right", "joint neck", "joint shoulder left", "joint shoulder right",
      "joint spine base", "joint spine mid", "joint spine shoulder", "joint thumb left", "joint thumb right",
      "joint wrist left", "joint wrist right"};

  const float HEAD_BOUNDING_BOX_THR = 50.0f; // TODO: Person MW team to make this a public constant
  const float ORIENTATION_CONFIDENCE_THR = 1.0f; // TODO: Person MW team to make this a public constant
  const float LANDMARKS_CONFIDENCE_THR = 50.0f;  // TODO: Person MW team to make this a public constant
  const float SKELETON_POINT_CONFIDENCE_THR = 50.0f; // TODO: Person MW team to make this a public constant

  const wchar_t* PERSON_MODULE_DATA_PATH = L"/usr/share/librealsense/pt/data/";

}
#endif
