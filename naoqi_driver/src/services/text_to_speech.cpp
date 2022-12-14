/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "text_to_speech.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

void TextToSpeechSayService::reset( ros::NodeHandle& nh )
{
  service_ = nh.advertiseService(topic_, &TextToSpeechSayService::callback, this);
}

bool TextToSpeechSayService::callback(nao_interaction_msgs::SayRequest& req, nao_interaction_msgs::SayResponse& resp)
{
  qi::Future<void> f = p_tts_.async<void>(func_, req.text);
  while(f.isRunning()) {
      ros::spinOnce();
  }
  return true;
}

}
}
