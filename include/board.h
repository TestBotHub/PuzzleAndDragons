#ifndef BOARD_H
#define BOARD_H
#include "bits/stdc++.h"
#include "params.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
// definition of abribiation
typedef pair<int, int> P;

// definition of the structs
enum drop_t {
  EMPTY = 0, DARK, FIRE, HEART, LEAF, LIGHT, WATER,
};

class Board {
public:
  int* board; // (x,y) -> x + y * W
  int* simulate_board;
  int curx, cury; // current position of cursor
  int combo_num;
  double  score;
  double* _drop_score;
  vector<P> path; // path of the cursor
  /**
   * Creates a default Board object.
   */
  Board(int* _board, int _curx, int _cury, vector<P> _path);

  ~Board();
  /**
   * Run simulation on current board.
   */
  int simulate();

  double get_score();
  /**
   * Based on variable "path", visualize route of the drop on a board.
   */
  void visualize_path();

  void print_board();

  void print_simulate_board();

  void print_path();

  void disp(bool debug, int duration);

  void disp_path(int* start);

  void _swap(int x, int y);

  int* _copy_board(int* _board);

  bool operator < (const Board &board) const {
		return score < board.score;
	}

  bool operator > (const Board &board) const {
		return score > board.score;
	}
// private:
  Scalar color[_N] = {
    Scalar(0, 0, 0),
    Scalar(153, 0, 153),
    Scalar(0, 0, 255),
    Scalar(255, 51, 255),
    Scalar(0, 255, 0),
    Scalar(0, 255, 255),
    Scalar(255, 0, 0)
  };
  int _count_drop_in_combo;
  /**
   *
   */
  void _bfs(int x, int y, int val);
  /**
   *
   */
  // int* _copy_board(int* _board);
  /**
   *
   */
  int _delete_drop();
  /**
   *
   */
  void _fill_drop();
  /**
   *
   */
  int _get_index(int x, int y);
  /**
   *
   */
  bool _in_range(int x, int y);
  /**
   *
   */
  void _mark_combo();
  /**
   *
   */
  int _simulate();
};
#endif
