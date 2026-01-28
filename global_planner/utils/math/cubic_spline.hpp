#ifndef _CUBIC_SPLINE_H
#define _CUBIC_SPLINE_H

#include <Eigen/Eigen>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "cpprobotics_types.hpp"

namespace cpprobotics {

inline Vec_d vec_diff(Vec_d input) {
  Vec_d output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
};

inline Vec_d cum_sum(Vec_d input) {
  Vec_d output;
  double temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

/**
* @brief 三次样条插值 根据插值原理进行插值
条件1：n段三次函数必须穿过所有已知节点
条件2：在所有节点(除了第一节点和最后一个节点)处0阶连续(保证数据不间断，无跳变),即，前一段方程在节点处的函数值和后一段方程在相同节点处的函数值相等。
条件3：在所有节点（除了第一节点和最后一个节点）处1阶连续（保证节点处有相同的斜率,原函数曲线上没有剧烈的跳变）
条件4：在所有节点（除了第一节点和最后一个节点）处2阶连续（保证节点处有相同的曲率，即，相同的弯曲程度）
*
*       三次样条的原理：
          三次样条的原理和二次样条的原理相同，我们用函数aX^3+bX^2+cX+d这个函数来进行操作，
        这里一共是4个点，分为3个区间，每个区间一个三次样条函数的话，一共是12个方程，只要我们找出这12个方程，这个问题就解决了。

        要求：
          1>内部节点处的函数值应该相等，这里一共是4个方程。

          2>函数的第一个端点和最后一个端点，应该分别在第一个方程和最后一个方程中。这里是2个方程。

          3>两个函数在节点处的一阶导数应该相等。这里是两个方程。

          4>两个函数在节点处的二阶导数应该相等，这里是两个方程。

          5>端点处的二阶导数为零，这里是两个方程。
              a1=0

              b1=0
*/
class Spline {
 public:
  Vec_d x;
  Vec_d y;
  int nx;
  Vec_d h;
  Vec_d a;
  Vec_d b;
  Vec_d c;
  // Eigen::VectorXf c;
  Vec_d d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(Vec_d x_, Vec_d y_)
      : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_) {
    // Matrix表示矩阵，Vector表示向量，数字表示维度，最后的f和i分别表示单精度和整型数据类型
    // A是一个动态大小的矩阵，目前的大小是0*0，它的元素数组完全没有分配
    // B是动态向量
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    // colPivHouseholderQr().solve()函数，求解的就是Ax=b中的x
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    // 存储顺序默认使用列存储，可以使用data()函数获得存储数据的首地址
    double *c_pointer = c_eigen.data();
    // Map类用于通过C++中普通的连续指针或者数组 （raw C/C++
    // arrays）来构造Eigen里的Matrix类，
    // 这就好比Eigen里的Matrix类的数据和raw C++array
    // 共享了一片地址，也就是引用。
    // 比如有个API只接受普通的C++数组，但又要对普通数组进行线性代数操作，那么用它构造为Map类，直接操作Map就等于操作了原始普通数组，省时省力。

    // STL中不同容器之间是不能直接赋值的，assign（）可以实现不同容器但相容的类型赋值
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] -
                  h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  };

  double calc(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
           d[seg_id] * dx * dx * dx;
  };

  double calc_d(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_dd(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

 private:
  Eigen::MatrixXd calc_A() {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
      }
      A(i + 1, i) = h[i];
      A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    return A;
  };

  Eigen::VectorXd calc_B() {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for (int i = 0; i < nx - 2; i++) {
      B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] -
                 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  };

  int bisect(float t, int start, int end) {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D {
 public:
  Spline sx;
  Spline sy;
  // s为弧长
  Vec_d s;

  //
  Spline2D(Vec_d x, Vec_d y) {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };

  Poi_d calc_position(double s_t) {
    double x = sx.calc(s_t);
    double y = sy.calc(s_t);
    return {{x, y}};
  };

  double calc_curvature(double s_t) {
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy) /
           ((dx * dx + dy * dy) * std::sqrt(dx * dx + dy * dy));
  };

  double calc_yaw(double s_t) {
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

 private:
  // 计算s值
  Vec_d calc_s(Vec_d x, Vec_d y) {
    Vec_d ds;
    Vec_d out_s{0};
    // 调用两两数值的差值
    Vec_d dx = vec_diff(x);
    Vec_d dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      // 求两两数之间的距离
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }
    // 调用数组累加
    Vec_d cum_ds = cum_sum(ds);
    // 在out_s头部插入cum_ds.begin()个cum_ds.end()
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};
}  // namespace cpprobotics
#endif
