#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include "search.h"
#include "control.h"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

bool executing = false;

class SubscribeAndPublish {
public:
  SubscribeAndPublish() {
    pub3 = nh.advertise<std_msgs::Float32MultiArray>("route2", 1.0);
    pub4 = nh.advertise<std_msgs::Float32MultiArray>("setup", 1.0);
    sub = nh.subscribe("board", 10, &SubscribeAndPublish::boardCallback2, this);
    sub2 = nh.subscribe("done", 10, &SubscribeAndPublish::doneCallback, this);
    executing = true;
    ros::Rate loop_rate(1.0);
    std_msgs::Float32MultiArray msgs = ctl.setup();
    for(int i=0;i<10;i++) {
      pub4.publish(msgs);
      loop_rate.sleep();
    }
  }
  void doneCallback(const std_msgs::Empty& msg) {
    sleep(20);
    cout << "done" << endl;
    executing = false;
  }

  void boardCallback2(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (!executing) {
      executing = true;
      ros::Rate loop_rate(1.0);
      int* board = (int*)calloc(sizeof(int), 30);
      for (int i=0;i<30;i++) {
        board[i] = msg->data[i];
        cout << " " << board[i];
      }
      cout << endl;
      Search* search = new Search(board, 0, 0);
      Board* best = search->beam_search();

      // best->disp_path(board);
      cout << best->score << endl;
      cout << best->combo_num << endl;
      std_msgs::Float32MultiArray msgs = ctl.getMsgs2(best->path);
      pub3.publish(msgs);
      loop_rate.sleep();
    }
  }
private:
  Control ctl;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mainNode");
  SubscribeAndPublish SAPObject;
  ros::spin();
  /*
  int bd[30] = {1, 2, 3, 4 ,5, 6, 1, 2, 3, 4 ,5, 6, 6, 1, 2, 3, 4 ,5, 1, 2, 3, 4 ,5, 6, 1, 2, 3, 4 ,5, 6};
  // Search* search = new Search(bd, 0, 0);
  Search* search = new Search("sample.dat", 0, 0);
  clock_t start = clock();    // スタート時間
  Board* best = search->beam_search(10);
  clock_t end = clock();     // 終了時間
  cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
  // best->print_board();
  // best->print_simulate_board();
  // best->print_path();
  cout << best->score << endl;
  cout << best->combo_num << endl;
  // best->disp(false, 5000);
  best->disp_path(search->load_from_file("sample.dat"));
  int cnt = 1;
  while(cnt) {
    best->_mark_combo();
    best->disp(true, 1000);
    cnt = best->_delete_drop();
    best->disp(true, 1000);
    best->_fill_drop();
    best->disp(true, 1000);
  }
  */
}
