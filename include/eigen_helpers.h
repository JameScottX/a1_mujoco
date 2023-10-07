/**
 * @file eigen_helpers.h
 * @author Junwen Cui / JameScottX (jun_wencui@126.com)
 * @brief 
 * @version 0.1
 * @date 2022-03-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MATH_EIGEN_HELPERS_H_
#define MATH_EIGEN_HELPERS_H_

#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <cstdio>
#include <functional>
#include <cmath>
#include <math.h>
#include <time.h>
#include <ctime>
#include <chrono>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "Eigen/StdVector"

#define SHOW_VECTOR(name,v) do{std::cout<< " --- " << name <<  " --- " << std::endl;std::cout<< v << std::endl;std::cout<< ""<< std::endl;}while(0)
#define SHOW_INFO(info) do{std::cout<< " - " << info <<  " - " << std::endl;std::cout<< ""<< std::endl;}while(0)
#define SHOW_VAL1(name,info) do{std::cout<< " --- " << name <<  " --- " << std::endl;std::cout<< "  " << info <<  "  " << std::endl;std::cout<< ""<< std::endl;}while(0)
#define SHOW_VAL2(name,info1,info2) do{std::cout<< " --- " << name <<  " --- " << std::endl;std::cout<< "  " << info1 <<  " " << info2 << " " << std::endl;std::cout<<""<< std::endl;}while(0)
#define SHOW_VAL3(name,info1,info2,info3) do{std::cout<< " --- " << name << " --- " << std::endl;std::cout<< "  " << info1 <<  " " << info2 << " " << info3 <<  " " << std::endl;std::cout<<""<< std::endl;}while(0)
#define SHOW_VAL4(name,info1,info2,info3,info4) do{std::cout<< " --- " << name << " --- " << std::endl;std::cout<< "  " << info1 <<  " " << info2 << " " << info3 << " " << info4 << " "<< std::endl;std::cout<<""<< std::endl;}while(0)
#define SHOW_VAL5(name,info1,info2,info3,info4,info5) do{std::cout<< " --- " << name << " --- " << std::endl;std::cout<< "  " << info1 <<  " " << info2 << " " << info3 << " " << info4 << " "<< info5 << " "<< std::endl;std::cout<<""<< std::endl;}while(0)

#define SHOW_INT1(name,info1) printf(" --- %s  --- \n %d \n",name,info1);
#define SHOW_INT2(name,info1,info2) printf(" --- %s  --- \n %d %d \n",name,info1,info2);
#define SHOW_INT3(name,info1,info2,info3) printf(" --- %s  --- \n %d %d %d \n",name,info1,info2,info3);
#define SHOW_INT4(name,info1,info2,info3,info4) printf(" --- %s  --- \n %d %d %d %d \n",name,info1,info2,info3,info4);
#define SHOW_INT5(name,info1,info2,info3,info4,info5) printf(" --- %s  --- \n %d %d %d %d %d\n",name,info1,info2,info3,info4,info5);

#define VecXd           Eigen::VectorXd
#define Vec3d           Eigen::Vector3d
#define Vec2d           Eigen::Vector2d
#define Vec4d           Eigen::Vector4d

#define MatXd           Eigen::MatrixXd
#define Mat2d           Eigen::Matrix2d
#define Mat3d           Eigen::Matrix3d
#define Mat4d           Eigen::Matrix4d

#define VecXdZero(n)    Eigen::VectorXd::Zero(n)
#define Vec2dZero       Eigen::Vector2d::Zero()
#define Vec3dZero       Eigen::Vector3d::Zero()
#define Vec4dZero       Eigen::Vector4d::Zero()

#define VecXdOnes(n)    Eigen::VectorXd::Ones(n)
#define Vec2dOnes       Eigen::Vector2d::Ones()
#define Vec3dOnes       Eigen::Vector3d::Ones()
#define Vec4dOnes       Eigen::Vector4d::Ones()

#define MatXdZero(r,c)  Eigen::MatrixXd::Zero(r,c)
#define Mat2dZero       Eigen::Matrix2d::Zero()
#define Mat3dZero       Eigen::Matrix3d::Zero()
#define Mat4dZero       Eigen::Matrix4d::Zero()

#define MatXdIdt(r,c)   Eigen::MatrixXd::Identity(r,c)
#define Mat2dIdt        Eigen::Matrix2d::Identity()
#define Mat3dIdt        Eigen::Matrix3d::Identity()
#define Mat4dIdt        Eigen::Matrix4d::Identity()

#define VecXdMap(v,n)   Eigen::Map<Eigen::VectorXd>(v,n,1)
#define MatXdMap(v,r,c) Eigen::Map<Eigen::MatrixXd>(v,r,c)
#define Mat4dMap(v)     Eigen::Map<Eigen::Matrix4d>(v,4,4)
#define Mat3dMap(v)     Eigen::Map<Eigen::Matrix3d>(v,3,3)
#define Mat2dMap(v)     Eigen::Map<Eigen::Matrix2d>(v,2,2)


using Eigen::VectorXi;
using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using scalar_t = double; 
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector_2t = Eigen::Matrix<scalar_t, 2, 1>;
using vector_3t = Eigen::Matrix<scalar_t, 3, 1>;
using vector_4t = Eigen::Matrix<scalar_t, 4, 1>;
using vector_6t = Eigen::Matrix<scalar_t, 6, 1>;
using matrix_2t = Eigen::Matrix<scalar_t, 2, 2>;
using matrix_3t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix_4t = Eigen::Matrix<scalar_t, 4, 4>;
using matrix_6t = Eigen::Matrix<scalar_t, 6, 6>;

#define vector_tMap(v,n)    Eigen::Map<vector_t>(v,n,1)
#define vector_tZero(n)     vector_t::Zero(n)

//---------------------------------
// Math helper functions

const double pi = M_PI;

template<typename T>
inline T _sqr(const T &val){ return val*val; }

template<typename T>
inline T _cube(const T &val){ return val*val*val; }

template <typename T>
inline int _sgn(T &val) {return (T(0) < val) - (val < T(0)); }

template <typename T>
inline T _sum(const T *val, short size_) {
  T val_sum = 0;
  for(char i = 0; i< size_; i++){
      val_sum += val[i];
  }
  return val_sum;
}


inline double smooth_abs(double x, double alpha=1.0) {
  //Differentiable "soft" absolute value function
  return sqrt(_sqr(x)+_sqr(alpha))-alpha;
}

//---------------------------------
// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
// From http://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
template<typename T>
inline T Mod(T x, T y) {
  //static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");
  if (0. == y)
    return x;
  double m= x - y * floor(x/y);
  // handle boundary cases resulted from floating-point cut off:
  if (y > 0) {             // modulo range: [0..y)
    if (m>=y)           // Mod(-1e-16, 360.): m= 360.
      return 0;

    if (m<0) {
      if (y+m == y)
        return 0; // just in case...
      else
        return y+m; // Mod(106.81415022205296, _TWO_PI ): m= -1.421e-14
    }
  }
  else {                    // modulo range: (y..0]
    if (m<=y)           // Mod(1e-16, -360.): m= -360.
      return 0;

    if (m>0) {
      if (y+m == y)
        return 0; // just in case...
      else
        return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14
    }
  }

  return m;
}

inline double wrap_to_pi(double angle)
{
  return Mod(angle+pi, 2*pi) - pi;
}


inline double my_sign(double val)
{
  return val >= 0.0 ? 1.0 : -1.0;
}

// TODO change name to subvec_w_bools
// TODO use Eigen::Ref instead of specific types
// TODO replace with more legit implementation like: 
// https://eigen.tuxfamily.org/dox-devel/TopicCustomizing_NullaryExpr.html#title1
// Extracts elements of vec for which indices is non-zero
inline VectorXd subvec_w_ind(const VectorXd& vec, const VectorXi& indices) {
	assert(vec.size() == indices.size());
	std::vector<double> vals;
	for(int i=0; i<vec.size(); i++){
		if(indices(i)>0){
			vals.push_back(vec(i));
		}
	}
	Eigen::Map<VectorXd> subvec(vals.data(), vals.size());
	return subvec;
}

// TODO same thing as above - don't need double for this..
//TODO make this faster, with no resizes, or use std
inline MatrixXd rows_w_ind(MatrixXd& mat, VectorXi& indices) {
  MatrixXd submat;
  if(mat.rows() != indices.size()) {
    std::cout << "mat.rows != indices.size\n";
    return submat;
  }
	for(int i=0; i<indices.size(); i++) {
		if(indices(i)>0) {
      submat.conservativeResizeLike(MatrixXd(submat.rows()+1, mat.cols()));
			submat.row(submat.rows()-1) = mat.row(i);
		}
	}
	return submat;
}

//TODO there has to be a better way to do this
// Equivalent of `mat = mat(bool_vec, bool_vec)` in matlab
inline MatrixXd extract_bool_rowsandcols(const MatrixXd& mat, const VectorXi& bool_vec){
  int n_dims = bool_vec.sum();
  MatrixXd small_mat(n_dims, n_dims);

  int row_idx = 0;
  for(int i=0; i<mat.rows(); i++) {
    int col_idx = 0;
    if(bool_vec(i) != 0) {
      for(int j=0; j<mat.cols(); j++) {
        if(bool_vec(j) != 0) small_mat(row_idx, col_idx++) = mat(i,j);
      }
      row_idx++;
    }
  }
  return small_mat;
}


const Eigen::IOFormat CleanFmt(3, 0, ", ", "\n", "", "");

inline void print_eigen(const std::string name, const Eigen::Ref<const Eigen::MatrixXd>& mat) {
  if(mat.cols() == 1)
  {
    std::cout << name << ": " << mat.transpose().format(CleanFmt) << ";" << std::endl;
  }
  else{
    std::cout << name << ":\n" << mat.format(CleanFmt) << std::endl;
  }
}


inline VectorXd elem_square(const VectorXd &vec) {
  return vec.array().square().matrix();
}

inline VectorXd elem_sqrt(const VectorXd &vec) {
  return vec.array().sqrt().matrix();
}

// 向量元素相乘 
inline VectorXd elem_mult(const VectorXd &vec1, const VectorXd &vec2) {
  return vec1.array() * vec2.array();
}

// Differentiable "soft" absolute value function
inline VectorXd sabs(const VectorXd &vec, const VectorXd &p) {
  VectorXd sum = elem_sqrt(elem_square(vec)+elem_square(p));
  return sum - p;
}

inline VecXd vec_normalized(VecXd &vec){
  return vec/vec.sum();
}
inline Vec3d vec_normalized(Vec3d &vec){
  return vec/vec.sum();
}


inline double stod98(const std::string &s) {
    return atof(s.c_str());
}

inline int stoi98(const std::string &s) {
    return atoi(s.c_str());
}


#endif



