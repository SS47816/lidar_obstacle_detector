/* box.hpp

 * Copyright (C) 2021 SS47816
 
 * Implementation of the Box and BoxQ class

**/

#pragma once

#include <Eigen/Geometry> 

struct Box
{
 public:
	int id;
	int color;
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};

struct BoxQ
{
 public:
  int id;
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
  float cube_width;
  float cube_height;
};