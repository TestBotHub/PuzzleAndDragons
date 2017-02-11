#ifndef CONTROL_H
#define CONTROL_H

#include "bits/stdc++.h"
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
using namespace std;
typedef pair<int, int> P;

class Control {
public:
  int basex, basey, stepx, stepy, x, y;
  Control();
  vector<geometry_msgs::Quaternion> getMsgs(vector<P> path);
  std_msgs::Float32MultiArray getMsgs2(vector<P> path);
  std_msgs::Float32MultiArray setup();
};

#endif
