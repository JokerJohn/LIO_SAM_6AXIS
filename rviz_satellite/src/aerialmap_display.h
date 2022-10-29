/* Copyright 2014 Gareth Cross

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#ifndef AERIAL_MAP_DISPLAY_H
#define AERIAL_MAP_DISPLAY_H

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreTexture.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>
#endif  //  Q_MOC_RUN

#include <tileloader.h>

#include <QByteArray>
#include <QFile>
#include <QFuture>
#include <QNetworkRequest>
#include <QObject>
#include <QtConcurrentRun>
#include <memory>

namespace Ogre {
class ManualObject;
}

namespace rviz {
class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class StringProperty;
class TfFrameProperty;
class EnumProperty;

/**
 * @class AerialMapDisplay
 * @brief Displays a satellite map along the XY plane.
 */
class AerialMapDisplay : public Display {
  Q_OBJECT
 public:
  AerialMapDisplay();
  ~AerialMapDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void fixedFrameChanged() override;
  void reset() override;
  void update(float, float) override;

 protected Q_SLOTS:
  void updateDynamicReload();
  void updateAlpha();
  void updateTopic();
  void updateFrame();
  void updateDrawUnder();
  void updateObjectURI();
  void updateZoom();
  void updateBlocks();
  void updateFrameConvention();

  //  slots for TileLoader messages
  void initiatedRequest(QNetworkRequest request);
  void receivedImage(QNetworkRequest request);
  void finishedLoading();
  void errorOcurred(QString description);

 protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  virtual void subscribe();
  virtual void unsubscribe();

  void navFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);

  void loadImagery();

  void assembleScene();

  void clear();

  void clearGeometry();

  void transformAerialMap();

  unsigned int map_id_;
  unsigned int scene_id_;

  /// Instance of a tile w/ associated ogre data
  struct MapObject {
    Ogre::ManualObject* object;
    Ogre::TexturePtr texture;
    Ogre::MaterialPtr material;
  };
  std::vector<MapObject> objects_;

  ros::Subscriber coord_sub_;

  //  properties
  RosTopicProperty* topic_property_;
  TfFrameProperty* frame_property_;
  Property* dynamic_reload_property_;
  StringProperty* object_uri_property_;
  IntProperty* zoom_property_;
  IntProperty* blocks_property_;
  FloatProperty* resolution_property_;
  FloatProperty* alpha_property_;
  Property* draw_under_property_;
  EnumProperty* frame_convention_property_;

  float alpha_;
  bool draw_under_;
  std::string object_uri_;
  int zoom_;
  int blocks_;

  //  tile management
  bool dirty_;
  bool received_msg_;
  sensor_msgs::NavSatFix ref_fix_;
  std::shared_ptr<TileLoader> loader_;
};

}  // namespace rviz

#endif
