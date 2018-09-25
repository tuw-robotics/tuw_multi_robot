#ifndef TUW_RQT_MAPTRANSFORMATION_H
#define TUW_RQT_MAPTRANSFORMATION_H

#include <geometry_msgs/Pose.h>

namespace tuw_rqt_ordermanager
{

class MapTransformation
{

public:
  MapTransformation();

  float transformMapToScene(int ax, float v);
  float transformSceneToMap(int ax, float v);
  geometry_msgs::Pose transformSceneToMap(geometry_msgs::Pose);
  geometry_msgs::Pose transformMapToScene(geometry_msgs::Pose);

  void setMapHeight(int);
  void setMapOriginPositionX(float);
  void setMapOriginPositionY(float);
  void setMapOriginPositionZ(float);
  void setMapResolution(float);

  const static int TRANSFORM_X = 0;
  const static int TRANSFORM_Y = 1;
  const static int TRANSFORM_Z = 2;
  const static int TRANSFORM_SCALAR = 3;

private:
  float map_height_;
  float map_origin_position_x_;
  float map_origin_position_y_;
  float map_origin_position_z_;
  float map_resolution_;
};

}

# endif
