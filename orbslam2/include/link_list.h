/**
 * @file link_list.h
 * @author cggos (cggos@outlook.com)
 * @brief
 * @version 0.1
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef LINK_LIST_H
#define LINK_LIST_H

#include <iostream>

namespace cg {

typedef int _T;

class DoubleLinkedList {
 private:
  typedef struct Node {
    _T data;
    Node* last;
    Node* next;
    Node() : last(nullptr), next(nullptr) {}
  } Node;

 public:
  DoubleLinkedList();
  ~DoubleLinkedList();

  bool empty() const;
  int size() const;

  Node* make_node();

  _T front() const;
  _T back() const;

  void push_front(const _T& data);
  void push_back(const _T& data);

 private:
  Node* head_;
  Node* tail_;
};

template <typename T>
class List {
  // ---------------------- Node -----------------------
 private:
  struct Node {
    T data;
    Node* prev;
    Node* next;
    // 再插入数据时可以直接对prev和next赋值
    Node(const T& d = T(), Node* p = 0, Node* n = 0) : data(d), prev(p), next(n) {}
  };
  // ---------------------------------------------------

  // -------------------- iterator ---------------------
 public:
  class iterator  // *  ++  !=  ==
  {
    friend class List;

   public:
    iterator() : current(0) {}
    T& operator*()  // *it   *it = x 可能通过*修改节点的data
    {
      return current->data;
    }
    iterator& operator++()  // 返回引用
    {
      current = current->next;
      return *this;
    }
    iterator operator++(int)  // 返回临时变量
    {
      iterator old = current;
      ++(*this);  // current = current->next;
      return old;
    }
    bool operator==(const iterator& itr) { return (itr.current == current); }
    bool operator!=(const iterator& itr) { return !(*this == itr); }  // return (&itr.current != current);
   protected:
    const List<T>* theList;  // ?
    Node* current;           // 随着节点滑动的节点指针
    // 用Node构造iterator，在begin end中返回
    iterator(Node* n) : current(n) {}
    iterator(const List<T>& lst, Node* n)  // ?
        : theList(&lst), current(n) {}
  };
  // ---------------------------------------------------

 public:
  List() { __init(); }   // 默认构造
  List(const List& rhs)  // 拷贝构造
  {
    *this = rhs;
  }
  List& operator=(const List& rhs)  // 赋值构造
  {
    // 若把自己赋值给自己
    if (this == &rhs) return *this;
    // List对象赋值给this这个对象，先清空this对象再依次按照rhs放入
    clear();
    for (iterator it = rhs.begin(); it != rhs.end(); ++it) {
      (*this).push_back(*it);
    }
    return *this;
  }
  ~List()  // 析构
  {
    clear();      // 释放所有数据节点
    delete head;  // 释放头尾节点
    delete tail;
  }
  // -------- begin and end --------
  iterator begin() const  // 通过Node构造iterator返回
  {
    return iterator(head->next);
  }
  iterator end() const { return iterator(tail); }
  // -------------------------------

  // 获取链表前后数据
  T& front() const { return *begin(); }              // cg: remove const when return
  T& back() const { return *iterator(tail->prev); }  // cg: remove const when return
  // 从前后插入
  void push_front(const T& d) {
    // Node* newNode = new Node(d);
    // newNode->next = head->next;
    // newNode->prev = head;
    // head->next->prev = newNode;
    // head->next = newNode;
    // ++theSize;
    insert(begin(), d);
  }
  void push_back(const T& d) {
    // Node* newNode = new Node(d);
    // newNode->next = tail;
    // newNode->prev = tail->prev;
    // tail->prev->next = newNode;
    // tail->prev = newNode;
    // ++theSize;
    insert(end(), d);
  }
  // 删除前后
  void pop_front() { erase(begin()); }
  void pop_back() { erase(iterator(tail->prev)); }

  // 擦除，返回下一个
  iterator erase(const iterator& itr) {
    Node* p = itr.current;    // 取出itr的Node
    iterator ret(p->next);    // 用next构造一个itr作为返回值
    p->next->prev = p->prev;  // 去掉该Node的链接关系
    p->prev->next = p->next;  //
    delete p;                 // 释放该Node
    --theSize;
    return ret;
  }
  iterator erase(const iterator& start, const iterator& end) {
    // 加const原因 可能传入的是个临时对象，如list.begin()
    //  而引用& 不能绑定到常量或者临时对象
    for (auto it = start; it != end; ++it) {
      erase(it);
    }
    return end;
  }
  // 在itr前插入新节点, 返回新节点的itr
  iterator insert(const iterator& itr, const T& d) {
    // Node* newNode = new Node(d);
    // Node* rNode = itr.current;
    // newNode->next = rNode;
    // newNode->prev = rNode->prev;
    // rNode->prev->next = newNode;
    // rNode->prev = newNode;
    // ++theSize;
    // return iterator(newNode);
    Node* rNode = itr.current;
    Node* newNode = new Node(d, rNode->prev, rNode);
    rNode->prev = rNode->prev->next = newNode;
    ++theSize;
    return iterator(newNode);
  }
  void clear() {
    while (!ismpty()) pop_front();
  }
  int size() const { return theSize; }
  bool ismpty() const { return (size() == 0); }
  void show() const {
    for (auto it = begin(); it != end(); ++it) {
      std::cout << *it << " ";
    }
    std::cout << std::endl;
  }

 private:
  int theSize;  // list节点个数，插入 删除 empty pop用到
  Node* head;   // 头尾结点，不存数据
  Node* tail;
  void __init() {
    theSize = 0;
    head = new Node;
    tail = new Node;
    head->next = tail;  // 非循环链表，head->prev和tail->next不用设置
    tail->prev = head;
  }
};

}  // namespace cg

#endif  // LINK_LIST_H