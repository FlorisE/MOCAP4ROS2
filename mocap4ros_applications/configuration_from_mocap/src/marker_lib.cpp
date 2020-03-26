// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Floris Erich <floris.erich@aist.go.jp>

#include <rclcpp/rclcpp.hpp>
#include <tinyxml.h>
#include <string>
#include <map>
#include <memory>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include "configuration_from_mocap/marker_lib.hpp"

namespace marker_lib
{

bool operator==(const Marker& left, const Marker& right) { return left.id == right.id; }

bool operator<(const Marker& left, const Marker& right) { return left.id < right.id; }

std::shared_ptr<Marker> find_marker(
  int marker_id, const std::vector<std::shared_ptr<Marker>>& markers
)
{
  for (std::shared_ptr<Marker> marker : markers)
  {
    if (marker->id == marker_id)
    {
      return marker;
    }
  }
  return nullptr;
}

bool parse_markers(
  const std::string& input, std::multimap<std::string, std::shared_ptr<Marker>>& res
)
{
  TiXmlDocument doc(input);
  bool loaded = doc.LoadFile();
  if (!loaded)
  {
    std::cerr << "Failed to load marker definitions.\n";
    return false;
  }

  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);
  pElem = hDoc.FirstChildElement().Element();
  if (!pElem) return false;
  if (pElem->ValueStr() != "markerFrames")
  {
    std::cerr << "Root element should be <markerFrames>, got " << pElem->Value() << '\n';
    return false;
  }
  hRoot = TiXmlHandle(pElem);
  pElem = hRoot.FirstChild("frame").Element();
  std::cout << "Parsing frames." << std::endl;
  for (pElem; pElem; pElem = pElem->NextSiblingElement())
  {
    TiXmlElement* pElemMarker;
    TiXmlHandle hElem(pElem);
    std::string frame_id = pElem->Attribute("id");
    std::string jointType = pElem->Attribute("joint_type");
    pElemMarker = hElem.FirstChildElement().Element();
    std::cout << "Parsing markers." << std::endl;
    for (pElemMarker; pElemMarker; pElemMarker = pElemMarker->NextSiblingElement())
    {
      Marker marker;
      std::string xyzText;
      marker.label = pElemMarker->Attribute("label");
      marker.frame = frame_id;
      pElemMarker->QueryIntAttribute("mocap_id", &marker.id);
      xyzText = pElemMarker->GetText();
      std::stringstream ss(xyzText);
      std::string token;
      int counter = 0;
      char delim = ' ';
      while (std::getline(ss, token, delim))
      {
        if (counter > 2)
        {
          std::cerr << "Too many values, expected three (x, y, z).\n";
        }
        if (counter == 0)
        {
          marker.x = stod(token);
        }
        else if (counter == 1)
        {
          marker.y = stod(token);
        }
        else
        {
          marker.z = stod(token);
        }
        ++counter;
      }
      if (counter < 3)
      {
        std::cerr << "Too few values, expected three (x, y, z).\n";
      }
      if ( jointType == "prismatic" )
      {
        marker.joint_type = JointType::PRISMATIC;
      }
      else if ( jointType == "fixed" )
      {
        marker.joint_type = JointType::FIXED;
      }
      else
      {
        marker.joint_type = JointType::REVOLUTE; // treat as default
      }

      res.insert( std::make_pair(frame_id, std::make_shared<Marker>(marker)) );
    }
  }

  return true;
}

bool parse_markers(const std::string& input, std::vector<std::shared_ptr<Marker>>& res)
{
  std::multimap<std::string, std::shared_ptr<Marker>> res2;
  bool success = parse_markers(input, res2);
  if (success)
  {
    std::cout << "Flattening markers" << std::endl;
    for (std::pair<std::string, std::shared_ptr<Marker>> p : res2)
    {
      res.push_back(p.second);
    }
  }
  else
  {
    return false;
  }
  return true;
}

bool write_markers(const std::string& output, const std::map<int, int>& markers)
{
  TiXmlDocument doc(output);
  bool loaded = doc.LoadFile();
  if (loaded)
  {
    TiXmlHandle hDoc(&doc);
    TiXmlElement* pElem;
    TiXmlHandle hRoot(0);
    pElem = hDoc.FirstChildElement().Element();
    if (!pElem) return false;
    if (pElem->ValueStr() != "markerFrames")
    {
      std::cerr << "Root element should be <markerFrames>, got " << pElem->Value() << '\n';
      return false;
    }
    hRoot = TiXmlHandle(pElem);
    pElem = hRoot.FirstChild("frame").Element();
    std::cout << "Parsing frames." << std::endl;
    for (pElem; pElem; pElem = pElem->NextSiblingElement())
    {
      TiXmlElement* pElemMarker;
      TiXmlHandle hElem(pElem);
      pElemMarker = hElem.FirstChildElement().Element();
      for (pElemMarker; pElemMarker; pElemMarker = pElemMarker->NextSiblingElement())
      {
        int oldMarkerId;
        pElemMarker->QueryIntAttribute("mocap_id", &oldMarkerId);
        for (const std::pair<int, int>& marker_pair : markers)
        {
          if (marker_pair.first == oldMarkerId)
          {
            pElemMarker->SetAttribute("mocap_id", marker_pair.second);
            break;
          }
        }
      }
    }
  }
  else
  {
    std::cerr << "Failed to reload marker file.\n";
    return false;
  }
  doc.SaveFile(output);
  return true;
}

void print_markers(const std::vector<Marker>& markers)
{
  for (const Marker& marker : markers)
  {
    std::cout << "marker id: " << marker.id 
         << ", marker x: " << marker.x
         << ", marker y: " << marker.y
         << ", marker z: " << marker.z
         << ", marker label: " << marker.label
         << ", marker frame: " << marker.frame
         << '\n';
  }
}

}
