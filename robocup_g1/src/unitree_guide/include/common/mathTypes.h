/**********************************************************************
 Copyright (c) 2020-2025, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>

// 3x3 Identity Matrix
#define I3 Eigen::MatrixXd::Identity(3, 3)

// 12x12 Identity Matrix
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 Identity Matrix
#define I18 Eigen::MatrixXd::Identity(18, 18)

/************************/
/******** Vector ********/
/************************/
// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 Vector
template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

// 8x1 Vector
template <typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// Quaternion
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// 4x1 Integer Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

// 2x1 Integer Vector
using VecInt2 = typename Eigen::Matrix<int, 2, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = typename Eigen::Matrix<T, 12, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = typename Eigen::Matrix<T, 18, 1>;

// 23x1 Vector
template <typename T>
using Vec23 = typename Eigen::Matrix<T, 23, 1>;

// Dynamic Length Vector
template <typename T>
using VecX = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/
// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// Homogenous Matrix
template <typename T>
using HomoMat = typename Eigen::Matrix<T, 4, 4>;

// 2x2 Matrix
template <typename T>
using Mat2 = typename Eigen::Matrix<T, 2, 2>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 3x4 Matrix, each column is a 3x1 vector
template <typename T>
using Vec34 = typename Eigen::Matrix<T, 3, 4>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 3x6 Matrix, each column is a 3x1 vector
template <typename T>
using Vec36 = typename Eigen::Matrix<T, 3, 6>;

// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 12x3 Matrix
template <typename T>
using Mat123 = typename Eigen::Matrix<T, 12, 3>;

template <typename T>
using Mat18 = typename Eigen::Matrix<T, 18, 18>;

// Dynamic Size Matrix
template <typename T>
using MatX = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

/************************/
/****** Functions *******/
/************************/
inline Vec34<double> vec12ToVec34(Vec12<double> vec12)
{
    Vec34<double> vec34;
    for (int i(0); i < 4; ++i)
    {
        vec34.col(i) = vec12.segment(3 * i, 3);
    }
    return vec34;
}

inline Vec12<double> vec34ToVec12(Vec34<double> vec34)
{
    Vec12<double> vec12;
    for (int i(0); i < 4; ++i)
    {
        vec12.segment(3 * i, 3) = vec34.col(i);
    }
    return vec12;
}

#endif // MATHTYPES_H