#ifndef TREENODE_H
#define TREENODE_H

#include <vector>

template<class T>
class TreeNode {
  std::vector<TreeNode<T>*> children;

public:
  T* const curr;
  void add_child(TreeNode<T>* c) { children.push_back(c); };
  TreeNode(T* const c) : curr(c) { children = std::vector<TreeNode<T>*>(); };
  typename std::vector<TreeNode<T>*>::const_iterator begin() const { 
    return children.begin(); 
  }
  typename std::vector<TreeNode<T>*>::const_iterator end() const { 
    return children.end();
  }
  typedef typename std::vector<TreeNode<T>*>::const_iterator child_iterator;
};

#endif
