#include "Matrix.h"

#include <iomanip>
#include <iostream>

namespace edu
{

Matrix::Matrix(Mat M)
{
  _M.resize(M.size());
  for (unsigned int i = 0; i < M.size(); i++)
    _M[i] = M[i];
}

Matrix::Matrix(Matrix& M)
{
  _M = M._M;
}

Matrix::~Matrix()
{
}

int Matrix::rows() const
{
  return _M.size();
}

int Matrix::cols() const
{
  if (rows() == 0)
    return 0;
  return _M[0].size();
}

double& Matrix::operator()(unsigned int row, unsigned int col)
{
  return _M[row][col];
}

Matrix operator*(Matrix& A, Matrix& B)
{
  Mat M(A.rows(), Vec(B.cols(), 0.0));
  Matrix C(M);
  for (int i = 0; i < A.rows(); i++)
  {
    for (int j = 0; j < B.cols(); j++)
    {
      for (int k = 0; k < A.cols(); k++)
      {
        C(i, j) += A(i, k) * B(k, j);
      }
    }
  }
  return C;
}

Vec operator*(Matrix& A, Vec& v)
{
  Vec vResult(A.rows(), 0.0);
  for (int i = 0; i < A.rows(); i++)
  {
    for (int j = 0; j < A.cols(); j++)
    {
      vResult[i] += A(i, j) * v[j];
    }
  }
  return vResult;
}

Matrix operator-(Matrix& A, Matrix& B)
{
  Mat M(A.rows(), Vec(B.cols(), 0.0));
  Matrix C(M);
  for (int r = 0; r < A.rows(); r++)
  {
    for (int c = 0; c < B.cols(); c++)
    {
      C(r, c) = A(r, c) - B(r, c);
    }
  }
  return C;
}

Matrix Matrix::subMatrix(int row1, int row2, int col1, int col2)
{
  int rows = row2 - row1 + 1, cols = col2 - col1 + 1;
  Mat M(rows, Vec(cols));
  for (int i = row1, r = 0; i <= row2; i++, r++)
  {
    auto it1 = _M[i].begin() + col1, it2 = _M[i].begin() + col2 + 1;
    copy(it1, it2, M[r].begin());
  }
  return Matrix(M);
}

double Matrix::determinant()
{
  if (rows() != cols())
  {
    std::cout << "WARNING: Determinat can only be computed for square matrices" << std::endl;
    return 0.0;
  }
  double det = 1.0;
  double n   = cols();
  for (int i = 0; i < n; i++)
  {
    int pivot = i;
    for (int j = i + 1; j < n; j++)
    {
      if (abs(_M[j][i]) > abs(_M[pivot][i]))
      {
        pivot = j;
      }
    }
    if (pivot != i)
    {
      std::swap(_M[i], _M[pivot]);
      det *= -1;
    }
    if (_M[i][i] == 0)
    {
      return 0;
    }
    det *= _M[i][i];
    for (int j = i + 1; j < n; j++)
    {
      double factor = _M[j][i] / _M[i][i];
      for (int k = i + 1; k < n; k++)
      {
        _M[j][k] -= factor * _M[i][k];
      }
    }
  }
  return det;
}

Matrix Matrix::compose(const Matrix& A11, const Matrix& A12, const Matrix& A21, const Matrix& A22)
{
  int k = A11.rows();
  int n = k + A22.rows();
  Mat A(n, Vec(n));

  for (int i = 0; i < k; i++)
  {
    copy(A11._M[i].begin(), A11._M[i].end(), A[i].begin());
    copy(A12._M[i].begin(), A12._M[i].end(), A[i].begin() + k);
  }

  for (int i = k; i < n; i++)
  {
    copy(A21._M[i - k].begin(), A21._M[i - k].end(), A[i].begin());
    copy(A22._M[i - k].begin(), A22._M[i - k].end(), A[i].begin() + k);
  }

  return Matrix(A);
}

Matrix Matrix::transposed()
{
  Mat B(cols(), Vec(rows()));
  for (int r = 0; r < rows(); r++)
    for (int c = 0; c < cols(); c++)
      B[c][r] = _M[r][c];
  return Matrix(B);
}

Matrix Matrix::inverse()
{
  int r = rows();
  if (r == 1)
  {
    double value = _M[0][0];
    // avoid division by zero
    if (std::abs(value) < 1e-16)
    {
      std::cerr << "WARNING: Non-invertible matrix passed!" << std::endl;
      Mat A(1, Vec(1, 0.0));
      return Matrix(A);
    }
    Mat A(1, Vec(1, 1.0 / value));
    return Matrix(A);
  }

  // Strassen's matrix multiplication
  // Partition matrix into four sub matrices
  // See Weisstein, Eric W. "Strassen's Formulas". MathWorld.
  // URL: https://mathworld.wolfram.com/StrassenFormulas.html
  int k      = r / 2;
  Matrix A11 = subMatrix(0, k - 1, 0, k - 1);
  Matrix A12 = subMatrix(0, k - 1, k, r - 1);
  Matrix A21 = subMatrix(k, r - 1, 0, k - 1);
  Matrix A22 = subMatrix(k, r - 1, k, r - 1);
  Matrix R1  = A11.inverse();
  Matrix R2  = A21 * R1;
  Matrix R3  = R1 * A12;
  Matrix R4  = A21 * R3;
  Matrix R5  = R4 - A22;
  Matrix R6  = R5.inverse();
  Matrix C12 = R3 * R6;
  Matrix C21 = R6 * R2;
  Matrix R7  = R3 * C21;
  Matrix C11 = R1 - R7;
  Matrix C22(R6);
  for (auto& row : C22._M)
  {
    for (auto& e : row)
      e = -e;
  }
  return compose(C11, C12, C21, C22);
}

Matrix Matrix::pseudoInverse()
{
  Matrix At       = transposed();
  edu::Matrix AtA = At * *this;
  edu::Matrix B   = AtA.inverse();
  edu::Matrix C   = B * At;
  return C;
}

void Matrix::print(std::string name)
{
  std::cout << name << std::endl;
  std::cout << std::string(13 * _M[0].size() + 1, '-') << std::endl;
  for (auto& row : _M)
  {
    std::cout << "|";
    for (auto x : row)
      std::cout << std::setw(12) << (std::abs(x) < 1e-9 ? 0.0 : x) << "|";
    std::cout << std::endl;
  }
  std::cout << std::string(13 * _M[0].size() + 1, '-') << std::endl << std::endl;
}

} // namespace edu
