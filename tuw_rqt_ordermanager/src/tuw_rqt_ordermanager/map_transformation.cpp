#include "tuw_rqt_ordermanager/map_transformation.h"

namespace tuw_rqt_ordermanager
{

MapTransformation::MapTransformation()
{
}

void MapTransformation::setMapHeight(int map_height)
{
  map_height_ = map_height;
}
void MapTransformation::setMapOriginPositionX(float position_x)
{
  map_origin_position_x_ = position_x;
}
void MapTransformation::setMapOriginPositionY(float position_y)
{
  map_origin_position_y_ = position_y;
}
void MapTransformation::setMapOriginPositionZ(float position_z)
{
  map_origin_position_z_ = position_z;
}
void MapTransformation::setMapResolution(float map_resolution)
{
  map_resolution_ = map_resolution;
}

float MapTransformation::transformMapToScene(int ax, float v)
{
  if (ax == TRANSFORM_X)
    return (v - map_origin_position_x_) / map_resolution_;
  if (ax == TRANSFORM_Y)
    return map_height_ - (v - map_origin_position_y_) / map_resolution_;
  if (ax == TRANSFORM_Z)
    return (v - map_origin_position_z_) / map_resolution_;
  return v / map_resolution_;
}

float MapTransformation::transformSceneToMap(int ax, float v)
{
  if (ax == TRANSFORM_X)
    return map_origin_position_x_ + v * map_resolution_;
  if (ax == TRANSFORM_Y)
    return map_origin_position_y_ - (v - map_height_) * map_resolution_;
  if (ax == TRANSFORM_Z)
    return map_origin_position_z_ + v * map_resolution_;
  return v * map_resolution_;
}

geometry_msgs::Pose MapTransformation::transformMapToScene(geometry_msgs::Pose pose)
{
  pose.position.x = transformMapToScene(TRANSFORM_X, pose.position.x);
  pose.position.y = transformMapToScene(TRANSFORM_Y, pose.position.y);
  pose.position.z = transformMapToScene(TRANSFORM_Z, pose.position.z);
  return pose;
}

geometry_msgs::Pose MapTransformation::transformSceneToMap(geometry_msgs::Pose pose)
{
  pose.position.x = transformSceneToMap(TRANSFORM_X, pose.position.x);
  pose.position.y = transformSceneToMap(TRANSFORM_Y, pose.position.y);
  pose.position.z = transformSceneToMap(TRANSFORM_Z, pose.position.z);
  return pose;
}

}
