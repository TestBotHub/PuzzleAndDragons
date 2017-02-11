#include "search.h"

Search::Search(int* _start, int _max_depth, int _beam) {
  max_depth = _max_depth;
  beam = _beam;
  start = _start;
}

Search::Search(string path, int _max_depth, int _beam) {
  max_depth = _max_depth;
  beam = _beam;
  start = load_from_file(path);
}

int* Search::load_from_file(string path) {
  ifstream fp;
  fp.open(path);
  int* board = (int*)calloc(sizeof(int), _H*_W);
  int pos = 0;
  if (fp.is_open()) {
    while(!fp.eof()) {
      fp >> board[pos++];
    }
  }
  fp.close();
  return board;
}

Board* Search::beam_search() {
  ZobristHash zh;
  priority_queue<Board*, vector<Board*>, BoardCompare> current_states_pq, good_states_pq;
  set<int> visited_states;

  // initialize cursor position
  // for (int x=1;x<_W-1;x++) {
  for (int x=0;x<_W;x++) {
    for (int y=0;y<_H;y++) {
      Board* board = new Board(start, x, y, vector<P>());
      current_states_pq.push(board);
    }
  }
  for (int d=0;d<BEAM_DEPTH;d++) {
    // cout << d << endl;
    priority_queue<Board*, vector<Board*>, BoardCompare> next_states_pq;
    for (int i=0;i<BEAM_SIZE;i++) {
      if (current_states_pq.empty())break;
      Board* state = current_states_pq.top();current_states_pq.pop();
      good_states_pq.push(state);
      visited_states.insert(zh.hash(state->board));
      vector<Board*> next_states = get_all_next_states(state);
      for (int j=0;j<next_states.size();j++) {
        Board* nstate = next_states[j];
        int nstate_hash_val = zh.hash(nstate->board);
        if (visited_states.find(nstate_hash_val) != visited_states.end()) continue;
        next_states_pq.push(nstate);
      }
    }
    current_states_pq = next_states_pq;
  }
  // while(!good_states_pq.empty()) {
  //   Board* bd = good_states_pq.top();good_states_pq.pop();
  //   cout << bd->score << endl;
  // }
  return good_states_pq.top();
}

Board* Search::beam_search(int size) {
  // initialize cursor position
  vector<thread> threads;
  for (int i=0;i<30/size;i++) {
    thread tmp = thread(&Search::worker, this, i, size);
    threads.push_back(move(tmp));
  }
  for (int i=0;i<threads.size();i++) {
    threads[i].join();
  }
  // while(!good_states_pq.empty()) {
  //   Board* bd = good_states_pq.top();good_states_pq.pop();
  //   cout << bd->score << endl;
  // }
  return good_states_pq.top();
}

void Search::worker(int id, int size) {
  priority_queue<Board*, vector<Board*>, BoardCompare> current_states_pq;
  int base = size * id;
  for (int i=0;i<size;i++) {
    int x = (i + base) % _W;
    int y = (i + base - x) / _W;
    Board* board = new Board(start, x, y, vector<P>());
    current_states_pq.push(board);
  }
  for (int d=0;d<BEAM_DEPTH;d++) {
    // cout << d << endl;
    priority_queue<Board*, vector<Board*>, BoardCompare> next_states_pq;
    for (int i=0;i<BEAM_SIZE;i++) {
      if (current_states_pq.empty())break;
      Board* state = current_states_pq.top();current_states_pq.pop();
      good_mutex.lock();
      good_states_pq.push(state);
      good_mutex.unlock();
      visited_mutex.lock();
      visited_states.insert(zh.hash(state->board));
      visited_mutex.unlock();
      vector<Board*> next_states = get_all_next_states(state);
      for (int j=0;j<next_states.size();j++) {
        Board* nstate = next_states[j];
        int nstate_hash_val = zh.hash(nstate->board);
        bool visited = false;
        visited_mutex.lock();
        visited = visited_states.find(nstate_hash_val) != visited_states.end();
        visited_mutex.unlock();
        if(visited) continue;
        next_states_pq.push(nstate);
      }
    }
    current_states_pq = next_states_pq;
  }
}

vector<Board*> Search::get_all_next_states(Board* board) {
  vector<Board*> next_states;
  int dx[] = {-1, 0, 1, 0};
  int dy[] = {0, -1, 0 ,1};
  for (int i=0;i<4;i++) {
    int nx = board->curx + dx[i], ny = board->cury + dy[i];
    board->_swap(nx, ny);
    Board* next_state = new Board(board->board, nx, ny, board->path);
    board->_swap(nx, ny);
    next_states.push_back(next_state);
  }
  return next_states;
}
