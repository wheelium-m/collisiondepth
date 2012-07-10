#include "Intervals.h"
using namespace std;

// Add an interval to a list of intervals for a particular scanline.
inline void addInterval(int start, int stop, list<pair<int,int> >& spans) {
  for(listIt it = spans.begin(); it != spans.end(); it++) {
    if(start < it->first) {
      if(stop >= it->first && stop <= it->second) {
        // Merge the new interval with an existing one.
        it->first = start;
      } else {
        // Insert the new interval
        spans.insert(it, make_pair(start,stop));
      }
      return;
    }
  }
}

// Increment an index parameter to point at either the next integer
// value, or the next value beyond a pointer into an interval list.
inline void nextFreeIndex(const listCIt& stop, listCIt& curr, int& i) {
  if(curr != stop && curr->first == i + 1) {
    i = curr->second + 1;
    curr++;
    nextFreeIndex(stop, curr, i);
  } else i++;
}
