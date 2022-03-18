# ORB Feature Test

---

## ORB Detector

主要实现了 特征均匀化分布的函数 `distribute_quadtree_c()`。

* [x] 实现了List类，替换掉STL的 `std::list`
* [x] 实现了部分数组，替换掉STL的 `std::vector`
* [ ] 还有一个vector待替换，目前用数组实现会有 `double free` 的错误
  ```cpp
  vector<pair<int, ExtractorNodeCG *> > vSizeAndPointerToNode;
  ```
* [x] 测试
  - 单图片测试，并与原始算法结果比较
  - 集成到ORBSLAM2中数据集测试，并解决内存溢出问题

### Build

```sh
mkdir build
cd build

# WITH_ORB_C=OFF: output key_pts.raw in data dir
# WITH_ORB_C=ON:  output key_pts.new in data dir
cmake -DWITH_ORB_C=ON ..

make -j4
```

### Run

```sh
./test_orb_detector
```