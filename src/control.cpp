#include "control.h"

Control::Control() {
  x = y = 0;
  basex = -148000;
  basey = -157000;
  stepx = 14400;
  stepy = 14400;
}

vector<geometry_msgs::Quaternion> Control::getMsgs(vector<P> path) {
  vector<geometry_msgs::Quaternion> v;
  for(int i=0;i<path.size();i++) {
    geometry_msgs::Quaternion msg;
    P p = path[i];
    cout << p.first << " " << p.second << endl;
    if (!i) {
      msg.x = basex - stepx * p.first;
      msg.y = basey - stepy * p.second;
      msg.z = -1.0;
      msg.w = 500.0;
      x = p.first;
      y = p.second;
    } else {
      msg.x = -stepx * (p.first - x);
      msg.y = -stepy * (p.second - y);
      msg.z = 1.0;
      msg.w = 40.0;
      x = p.first;
      y = p.second;
    }
    v.push_back(msg);
    if (!i) {
      msg.x = 0;
      msg.y = 0;
      msg.z = 1.0;
      msg.w = 400.0;
      v.push_back(msg);
    }
  }
  geometry_msgs::Quaternion msg;
  msg.x = 0;
  msg.y = 0;
  msg.z = -1.0;
  msg.w = 600.0;
  v.push_back(msg);
  msg.x = stepx * x - basex;
  msg.y = stepy * y - basey;
  msg.z = -1.0;
  msg.w = 500.0;
  v.push_back(msg);
  return v;
}

std_msgs::Float32MultiArray Control::setup() {
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.data.push_back(-144000.0);
  msg.data.push_back(-205000.0);
  msg.data.push_back(0.0);
  msg.data.push_back(2.0);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(1.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(8.0);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(1.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.1);

  msg.data.push_back(28000.0);
  msg.data.push_back(48000.0);
  msg.data.push_back(0.0);
  msg.data.push_back(2.0);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(1.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(-48000.0);
  msg.data.push_back(0.0);
  msg.data.push_back(2.0);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(1.0);
  msg.data.push_back(0.1);

  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.0);
  msg.data.push_back(0.1);

  msg.data.push_back(116000.0);
  msg.data.push_back(205000.0);
  msg.data.push_back(0.0);
  msg.data.push_back(3.0);

  msg.layout.dim[0].size = msg.data.size();
  msg.layout.dim[0].stride=1;
  msg.layout.dim[0].label ="x";
  return msg;
}


std_msgs::Float32MultiArray Control::getMsgs2(vector<P> path) {
  std_msgs::Float32MultiArray msg;
  msg.data.clear();
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  for(int i=0;i<path.size();i++) {
    P p = path[i];
    cout << p.first << " " << p.second << endl;
    if (!i) {
      msg.data.push_back(basex + stepx * p.first);
      msg.data.push_back(basey + stepy * p.second);
      msg.data.push_back(0);
      msg.data.push_back(0.1);
      x = p.first;
      y = p.second;
    } else {
      msg.data.push_back(stepx * (p.first - x));
      msg.data.push_back(stepy * (p.second - y));
      msg.data.push_back(1);
      msg.data.push_back(0.01);
      x = p.first;
      y = p.second;
    }
  }
  // msg.data.push_back(- stepx * x + basex);
  // msg.data.push_back(- stepy * y + basey);
  // msg.data.push_back(0);
  // msg.data.push_back(0.3);
  msg.layout.dim[0].size = msg.data.size();
  msg.layout.dim[0].stride=1;
  msg.layout.dim[0].label ="x";
  return msg;
}
