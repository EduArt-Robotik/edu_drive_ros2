#pragma once

#include <vector>
#include <string>

namespace edu
{

using Vec = std::vector<double>;
using Mat = std::vector<Vec>;

/**
 * @class Matrix
 * @brief Simple matrix algebra to avoid the binding to 3rd party libraries. This class is capable of inverting kinemtic matrices as used in our drive stack.
 * @author Stefan May
 * @date 29.01.2025
 */
class Matrix
{

public:

    /**
     * Constructor
     * @param[in] M Matrix elements as 2-dimensional vector
     */
    Matrix(Mat M);

    /**
     * @brief Copy constructor
     * @param[in] M Instanciate matrix and copy values of M 
     */
    Matrix(Matrix& M);

    /**
     * @brief Destructor
     */
    ~Matrix();

    /**
     * @brief Accessor to number of rows
     * @return number of rows
     */
    int rows() const;

    /**
     * @brief Accessor to number of columns
     * @return number of colums
     */
    int cols() const;

    /**
     * @brief Element access operator
     * @param[in] row row index of element
     * @param[in] col col index of element
     * @return reference to element
     */
    double& operator () (unsigned int row, unsigned int col);

    /**
     * @brief Matrix-Matrix muliply operator
     * @param[in] A left multiplier 
     * @param[in] B right multilier 
     * @return Resulting Matrix A x B
     */
    friend Matrix operator * (Matrix &A, Matrix &B);

    /**
     * @brief Matrix-Vector muliply operator
     * @param[in] A left multiplier 
     * @param[in] B right multilier 
     * @return Resulting Matrix A x v
     */
    friend Vec operator * (Matrix &A, Vec &v);

    /**
     * @brief Instanciante submatrix
     * @param row1 Start row
     * @param row2 Last row
     * @param col1 Start column
     * @param col2 Last column
     * @return Submatrix 
     */
    Matrix subMatrix(int row1, int row2, int col1, int col2);

    /**
     * @brief Calculate determinant
     * @return determinant of Matrix
     */
    double determinant();

    /**
     * @brief Get transposed matrix as new instance
     * @return Transpose of matrix 
     */
    Matrix transposed();

    /**
     * @brief Get inverted matrix as new instance
     * @return Inverse of matrix
     */
    Matrix inverse();

    /**
     * @brief Get pseudoinverse as new instance
     * @return Pseudoinverse
     */
    Matrix pseudoInverse();

    /**
     * @brief Print matrix elements to stdout
     * @param name Give output a readable title, ideally the name of the matrix
     */
    void print(std::string name);

private:

    /**
     * @brief Compose a matrix from four submatrices
     * @param A11 left-upper part 
     * @param A12 right-upper part
     * @param A21 left-lower part
     * @param A22 right-lower part
     * @return Composed matrix 
     */
    Matrix compose( const Matrix &A11, const Matrix &A12, const Matrix &A21, const Matrix &A22 );

    /**
     * Internal data representation of matrix
     */
    std::vector<Vec> _M;

};

}
