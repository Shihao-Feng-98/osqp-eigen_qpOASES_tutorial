#include <qpOASES.hpp>

/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	// using namespace qpOASES
	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
    // Data is stored row by row in a one-dimensional array
	real_t H[2*2] = {1., 0., 
                    0., 0.5};
	real_t A[1*2] = {1., 
                    1.};
	real_t g[2] = {1.5, 1.};
	real_t lb[2] = {0.5, -2.};
	real_t ub[2] = {5., 2.};
	real_t lbA[1] = {-1.};
	real_t ubA[1] = {2.};

	/* Setup data of second QP. */
	real_t g_new[2] = {1., 1.5};
	real_t lb_new[2] = {0., -1.};
	real_t ub_new[2] = {5., -0.5};
	real_t lbA_new[1] = {-2.};
	real_t ubA_new[1] = {1.};

	/* Setting up QProblem object. 
	QProblem(int_t nV, int_t nC)
    nv: num of variables
    nc: num of constraints */
	QProblem example(2, 1);

	Options options;
	example.setOptions(options);

	/* Solve first QP
	example.init（H，g，A，lb，ub，lbA，ubA，nWSR，cputime）
    H: hessian matrix
	g: gradient vector
	A: constraint matrix
	lb ub: lower bound and upper bound variables vector
	lbA ubA: lower bound and upper bound constraint vector
	nWSR: maximum iterations
	cputime: if is not the null pointer, it contains the maximum allowed CPU time in seconds for the whole initialisation */
	int_t nWSR = 10; 
	example.init(H, g, A, lb, ub, lbA, ubA, nWSR);

	/* If there are no bound in you QP formulation, pass a null pointer instead.
	All init functions make deep copies of all vector arguments, thus afterwards you have to free their memory yourself. 
	The matrix arguments H and A are not deep copied, so they must not be changed between consecutive calls to qpOASES. */

	/*
	SUCCESSFUL RETURN: initialisation successful (including solution of first QP),
	RET MAX NWSR REACHED: initial QP could not be solved within the given number of working set recalculations,
	RET INIT FAILED (or a more detailed error code): initialisation failed.
	*/

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	/* Writes the optimal primal solution vector (dimension: nV) into the array xOpt, 
	which has to be allocated (and freed) by the user */
	example.getPrimalSolution(xOpt);
	/* Writes the optimal dual solution vector 2 (dimension: nV + nC) into the array yOpt, 
	which has to be allocated (and freed) by the user */
	example.getDualSolution(yOpt); 
	printf( "xOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0], xOpt[1], yOpt[0], yOpt[1] ,yOpt[2], example.getObjVal());
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart(g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR);

	/* Get and print solution of second QP. */
	example.getPrimalSolution(xOpt);
	example.getDualSolution(yOpt);
	printf( "xOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0], xOpt[1], yOpt[0], yOpt[1] ,yOpt[2], example.getObjVal());

	// example.printOptions();
	// example.printProperties();
	// getGlobalMessageHandler()->listAllMessages();
	return 0;
}