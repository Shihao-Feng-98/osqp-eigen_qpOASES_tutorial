#include <iostream> 
using namespace std;
#include <Eigen/Dense> // version 3.3.7
using namespace Eigen;
#include <OsqpEigen/OsqpEigen.h> // osqp-eigen


SparseMatrix<double> hessian;
VectorXd gradient;
SparseMatrix<double> linearMatrix;
VectorXd lowerBound;
VectorXd upperBound;

void test_osqp_eigen()
{
    OsqpEigen::Solver solver;

    unsigned int numOfVar = 3;
    unsigned int numOfCons = 4;
    solver.data()->setNumberOfVariables(numOfVar);
    solver.data()->setNumberOfConstraints(numOfCons);
 
    hessian.resize(numOfVar, numOfVar);
    gradient.resize(numOfVar);
    linearMatrix.resize(numOfCons, numOfVar);
    lowerBound.resize(numOfCons);
    upperBound.resize(numOfCons);
 
    hessian.insert(0, 0) = 1;
    hessian.insert(0, 1) = -1;
    hessian.insert(0, 2) = 1;
    hessian.insert(1, 0) = -1;
    hessian.insert(1, 1) = 2;
    hessian.insert(1, 2) = -2;
    hessian.insert(2, 0) = 1;
    hessian.insert(2, 1) = -2;
    hessian.insert(2, 2) = 4;
   //cout << "hessian" << hessian << endl;
   /* hessian << 1, -1, 1,
              -1, 2, -2,
               1, -2, 4;*/
 
    gradient << 2, -3, 1;
    
    linearMatrix.insert(0, 0) = 1;
    linearMatrix.insert(1, 1) = 1;
    linearMatrix.insert(2, 2) = 1;
    linearMatrix.insert(3, 0) = 1;
    linearMatrix.insert(3, 1) = 1;
    linearMatrix.insert(3, 2) = 1;
    /*linearMatrix << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1,
                    1, 1, 1;*/
 
    lowerBound << 0, 0, 0, 0.5;
    upperBound << 1, 1, 1, 0.5;

    solver.settings()->setVerbosity(true);
    solver.settings()->setWarmStart(true);

    solver.data()->setHessianMatrix(hessian);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(linearMatrix);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    solver.initSolver();
    solver.solveProblem();
    cout << "res: " << solver.getSolution().transpose() << endl;
}

int main()
{
    test_osqp_eigen();

    return 0;
}
