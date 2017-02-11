#include "board.h"

// private functions
void Board::_bfs(int x, int y, int val) {
  simulate_board[_get_index(x, y)] = EMPTY;
  _count_drop_in_combo++;
  int dx[] = {-1, 0, 1, 0};
  int dy[] = {0, -1, 0 ,1};
  for (int i=0;i<4;i++) {
    int nx = x + dx[i], ny = y + dy[i];
    if (!_in_range(nx, ny)) {
      continue;
    }
    if (simulate_board[_get_index(nx, ny)] > 10 && simulate_board[_get_index(nx, ny)] % 10 == val) {
      _bfs(nx, ny, val);
    }
  }
}

int* Board::_copy_board(int* bd) {
  int* cp_bd = (int*)calloc(sizeof(int) * _H * _W, 1);
  for (int i=0;i<_H*_W;i++) {
    cp_bd[i] = bd[i];
  }
  return cp_bd;
}

int Board::_delete_drop() {
  int cnt = 0;
  for (int x=0;x<_W;x++) {
    for (int y=0;y<_H;y++) {
      if (simulate_board[_get_index(x, y)] > 10) {
        int type = simulate_board[_get_index(x, y)] % 10;
        _count_drop_in_combo = 0;
        _bfs(x, y, type);
        _drop_score[type] += 1 + 0.25 * (_count_drop_in_combo - 3);
        cnt++;
      }
    }
  }
  return cnt;
}

void Board::_fill_drop() {
  for (int x=0;x<_W;x++) {
    int empty_pos = _H - 1;
    while (true) {
      if (!empty_pos) break;
      if (simulate_board[_get_index(x, empty_pos)] == EMPTY) {
        int drop_pos = empty_pos - 1;
        while (true) {
          if (drop_pos < 0) break;
          if (simulate_board[_get_index(x, drop_pos)]) {
            swap(simulate_board[_get_index(x, drop_pos)], simulate_board[_get_index(x, empty_pos)]);
            break;
          } else {
            drop_pos--;
          }
        }
        if (drop_pos < 0) break;
      } else {
        empty_pos--;
      }
    }
  }
}

int Board::_get_index(int x, int y) {
  return x + y * _W;
}

bool Board::_in_range(int x, int y) {
  return 0 <= x && x < _W && 0 <= y && y < _H;
}

void Board::_mark_combo() {
  for (int y=0;y<_H;y++) {
    for (int x=0;x<_W;x++) {
      int len = 0;
      if (!simulate_board[_get_index(x, y)]) continue;
      while (true) {
        if (!_in_range(x+len, y) || simulate_board[_get_index(x, y)] % 10 != simulate_board[_get_index(x+len, y)] % 10) break;
        len++;
      }
      if (len >= 3) {
        for (int i=0;i<len;i++) {
          simulate_board[_get_index(x+i, y)] += 10;
        }
      }
    }
  }

  for (int x=0;x<_W;x++) {
    for (int y=0;y<_H;y++) {
      int len = 0;
      if (!simulate_board[_get_index(x, y)]) continue;
      while (true) {
        if (!_in_range(x, y+len) || simulate_board[_get_index(x, y)] % 10 != simulate_board[_get_index(x, y+len)] % 10) break;
        len++;
      }
      if (len >= 3) {
        for (int i=0;i<len;i++) {
          simulate_board[_get_index(x, y+i)] += 10;
        }
      }
    }
  }
}

int Board::_simulate() {
  // print_simulate_board();
  _mark_combo();
  // print_simulate_board();
  int cnt = _delete_drop();
  // print_simulate_board();
  _fill_drop();
  // print_simulate_board();
  return cnt;
}

// public functions

Board::Board(int* _board, int _curx, int _cury, vector<P> _path) {
  board = _copy_board(_board);
  simulate_board = _copy_board(_board);
  curx = _curx, cury = _cury;
  path = _path;
  path.push_back(P(curx, cury));
  combo_num = 0;
  get_score();
}

Board::~Board() {
  free(board);
  free(simulate_board);
  free(_drop_score);
}

int Board::simulate() {
  int cnt = 0, tmp = 0;
  while (true) {
    tmp = _simulate();
    cnt += tmp;
    if (!tmp)break;
  }
  return cnt;
}

double Board::get_score() {
  // if (score != -1) return score;
  score = 0;
  _drop_score = (double*)calloc(sizeof(double), _N);
  combo_num = simulate();
  if (!combo_num) {
    score = 0;
    return score;
  }
  for (int i=1;i<_N;i++) {
    score += _drop_score[i];
  }
  score *= (1 + 0.25 * (combo_num - 1));
  int cnt = 0;
  for (int i=1;i<_N;i++) {
    if (_drop_score[i] > 0 && i != 3) {
      cnt++;
    }
  }
  if (cnt >= 4) {
    score *= 16;
  }
  return score;
}

void Board::print_board() {
  for (int i=0;i<_H;i++) {
    for (int j=0;j<_W;j++) {
      cout << " " << board[i * _W + j];
    }
    cout << endl;
  }
  cout << endl;
}

void Board::print_simulate_board() {
  for (int i=0;i<_H;i++) {
    for (int j=0;j<_W;j++) {
      cout << " " << simulate_board[i * _W + j];
    }
    cout << endl;
  }
  cout << endl;
}

void Board::print_path() {
  for (int i=0;i<path.size();i++) {
    cout << " (" << path[i].first << "," << path[i].second << ")";
  }
  cout << endl;
}

void Board::_swap(int x, int y) {
  if (_in_range(x, y)) {
    swap(board[_get_index(curx, cury)], board[_get_index(x, y)]);
  }
}


void Board::disp(bool debug, int duration) {
  Mat img(250, 300, CV_8UC3, Scalar(0, 0, 0));
  int* bd = simulate_board;
  if (!debug) {
    bd = board;
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
  }
  for (int y=0;y<_H;y++) {
    for (int x=0;x<_W;x++) {
      img(Rect(x * 50, y * 50, 50, 50)) = color[bd[_get_index(x, y)]];
    }
  }
  int x = curx * 50 + 25, y = cury * 50 + 25;
  circle(img, Point(x, y), 4, Scalar(0));
  cv::imshow("image", img);
  cv::waitKey(duration);
}

void Board::disp_path(int* start) {
  simulate_board = _copy_board(start);
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
  P p = path[0];
  curx = p.first, cury = p.second;
  disp(true, 1000);
  for (int i=1;i<path.size();i++) {
    P p1 = path[i-1];
    P p2 = path[i];
    curx = p2.first, cury = p2.second;
    swap(simulate_board[_get_index(p1.first, p1.second)], simulate_board[_get_index(p2.first, p2.second)]);
    if (i==path.size()-1) disp(true, 500);
    else disp(true, 500);
  }
}
