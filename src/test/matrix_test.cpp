#include <iostream>

#include "../Matrix.h"

/**
 * @brief Simple test program to check functionality of Matrix class.
 *        It shows how a kinematic matrix could be inverted in order to
 *        get the inverse for odometry calculation. The model is tested with
 *        2-, 4-, 6- and 8-wheeled systems.
 *        Compile this program in this folder with:
 *        $ g++ matrix_test.cpp ../Matrix.cpp -o matrix_test
 * @author Stefan May
 * @date 30.1.2025
 */
int main()
{
  double r   = 0.1;
  double lx  = 0.3;
  double ly  = 0.25;
  double fy  = 1.0; // factor for y dimension (fy=1: mecanum, fy=0: skid)
  edu::Mat K = {
    { 1 / r,  -fy / r, -(lx + ly) / r     },
    { -1 / r, -fy / r, -(lx + ly) / r     },
    { 1 / r,  fy / r,  -(lx + ly) / r     },
    { -1 / r, fy / r,  -(lx + ly) / r     },
    { 1 / r,  -fy / r, -(ly + lx / 2) / r },
    { -1 / r, -fy / r, -(ly + lx / 2) / r },
    { 1 / r,  fy / r,  -(ly + lx / 2) / r },
    { -1 / r, fy / r,  -(ly + lx / 2) / r }
  };
  edu::Matrix A(K);
  A.print("A =");
  edu::Matrix At = A.transposed();
  At.print("A' =");
  edu::Matrix AtA = At * A;
  AtA.print("A'A =");
  std::cout << "det(AtA)=" << AtA.determinant() << std::endl << std::endl;
  edu::Matrix B = AtA.inverse();
  B.print("inv(A'A) =");

  // C is the inverted robot kinematics (step-by-step calculation)
  edu::Matrix C = B * At;
  C.print("inv(A'A)A' =");

  // D is the inverted robot kinematics
  edu::Matrix D = A.pseudoInverse();
  D.print("pinv(A)");

  // Test vector
  edu::Vec v(A.rows(), 1);

  edu::Vec vTwist = D * v;
  std::cout << "vx=" << vTwist[0] << std::endl;
  std::cout << "vy=" << vTwist[1] << std::endl;
  std::cout << "omega=" << vTwist[2] << std::endl;
}