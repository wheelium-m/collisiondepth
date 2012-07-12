#ifndef INTERVALS_H
#define INTERVALS_H
#include <list>

typedef std::list<std::pair<int,int> >::iterator listIt;
typedef std::list<std::pair<int,int> >::const_iterator listCIt;

// Add an interval to a list of intervals for a particular scanline.
inline void addInterval(int start, int stop, std::list<std::pair<int,int> >* spans) {
  if(0 == spans->size()) {
    spans->push_front(std::make_pair(start, stop));
  } else {
    for(listIt it = spans->begin(); it != spans->end(); it++) {
      if(start <= it->first) {
        if(stop >= it->first) {
          // Merge the new interval with an existing one.
          it->first = start;
          it->second = std::max(stop, it->second);
        } else {
          // Insert the new interval
          spans->insert(it, std::make_pair(start,stop));
        }
        return;
      }
    }
    spans->push_back(std::make_pair(start,stop));
  }
}

// Increment an index parameter to point at either the next integer
// value, or the next value beyond a pointer into an interval list.
inline int nextFreeIndex(const listCIt& stop, listCIt& curr, int i) {
  if(curr != stop && curr->first <= i + 1) {
    i = curr->second > i ? curr->second + 1 : i + 1;
    curr++;
    while(curr != stop && curr->first <= i) {
      i = curr->second > i ? curr->second + 1 : i + 1;
      curr++;
    }
  } else i++;
  return i;
}

#endif
