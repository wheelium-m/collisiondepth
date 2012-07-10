#ifndef INTERVALS_H
#define INTERVALS_H
#include <list>

typedef std::list<std::pair<int,int> >::iterator listIt;
typedef std::list<std::pair<int,int> >::const_iterator listCIt;

// Add an interval to a list of intervals for a particular scanline.
inline void addInterval(int start, int stop, std::list<std::pair<int,int> >& spans);

// Increment an index parameter to point at either the next integer
// value, or the next value beyond a pointer into an interval list.
inline void nextFreeIndex(const listCIt& stop, listCIt& curr, int& i);

#endif
