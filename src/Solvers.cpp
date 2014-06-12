#include <Moby/select>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/VectorNd.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <fstream>
#include <string>

using namespace std;
using namespace Ravelin;
using namespace Moby;

/// Iterative method for solving linear complementarity problems w/symmetric M
/**
 * Solves problems of the form Ax + b = w, where x'w = 0, x >= 0 and w >= 0
 * \note this method is via [Murty 1988]
 */

const double NEAR_ZERO = std::numeric_limits<double>::epsilon();

bool lcp_symm_iter(const Ravelin::MatrixNd& M, const Ravelin::VectorNd& q, Ravelin::VectorNd& z, double lambda, double omega, unsigned MAX_ITER)
{
  unsigned n = q.size();

  // verify size of z
  z.resize(n);

  // http://ioe.engin.umich.edu/people/fac/books/murty/linear_complementarity_webbook/lcp-complete.pdf
  // NOTE: uses the projected symmetric SOR scheme (p. 373)
  // NOTE: we use E as identity
  // NOTE: we use L/U alternately as K

  // setup work vectors and matrices
  static VectorNd row, znew, w;
  static vector<VectorNd> m;
  static MatrixNd L, G, U;

  // get rows of M
  m.resize(n);
  for (unsigned i=0; i< n; i++)
  {
    M.get_row(i, row);
    m[i] = row;
  }

  // compute L, U and G
  L.set_zero(n,n);
  G.set_zero(n,n);
  U.set_zero(n,n);
  for (unsigned i=0; i< n; i++)
    for (unsigned j=0; j< n; j++)
    {
      if (i > j)
        L(i,j) = M(i,j);
      else if (i == j)
        G(i,j) = M(i,j);
      else
        U(i,j) = M(i,j);
    }

  // setup new z
  znew.resize(n);

  // iterate..
  for (unsigned i=0; i< MAX_ITER; i++)
  {
    // determine the new z (p. 367)
    znew[0] = std::max(lambda*(z[0] - omega*(m[0].dot(z)+q[0])),(double) 0.0) + (1-lambda)*z[0];
    for (unsigned j=1; j< n; j++)
    {
      double subsum = 0;
      if (i % 2 == 0)
        for (unsigned l=0; l< j; l++)
          subsum += L(j,l)*(znew[l] - z[l]);
      else
        for (unsigned l=0; l< j; l++)
          subsum += U(j,l)*(znew[l] - z[l]);
      znew[j] = std::max(lambda*(z[j] - omega*(m[j].dot(z)+q[j]+subsum)),(double) 0.0) + (1-lambda)*z[j];
    }

    // swap the two
    std::swap(z, znew);

    // if there is little difference between the two, quit
    if ((znew -= z).norm() < NEAR_ZERO)
      break;
  }

  return true;
}
/*
void pgs(A, b, x0 = zeros(N,1), max_iter = floor(N/2), tol_rel = 0.0001, tol_abs= 10*eps, profile = false,
         x, err, iter, flag, convergence, msg){
//% Copyright 2011, Kenny Erleben, DIKU

//% Just a list of human readable text strings to convert the flag return
//% code into something readable by writing msg(flag) onto the screen.
msg = {'preprocessing';  % flag = 1
  'iterating';      % flag = 2
  'relative';       % flag = 3
  'absolute';       % flag = 4
  'stagnation';     % flag = 5
  'local minima';   % flag = 6
  'nondescent';     % flag = 7
  'maxlimit'        % flag = 8
  };

if nargin < 2
  error('Too few arguments');
end

N    = length(b); % Number of variables
flag = 1;

//%--- Make sure all values are valid ---------------------------------------
max_iter = max(max_iter,1);
tol_rel  = max(tol_rel,0);
tol_abs  = max(tol_abs,0);
x0       = max(0,x0);

//%--- Setup values needed while iterating ------------------------------------
convergence = zeros(max_iter,1); % Used when profiling to measure the convergence rate
err         = Inf;               % Current error measure
x           = x0;                % Current iterate
iter        = 1;                 % Iteration counter
flag        = 2;

while iter <= max_iter

  dx = 0;
  for i=1:N
    old_xi = x(i);
    ri     = b(i) + A(i,:)*x;
    Aii    = A(i,i);

    x(i) = max( 0, old_xi - (ri / Aii) );
    dx = max(dx, abs(x(i) - old_xi));
  end

  old_err = err;

  y   = abs( A*x + b );   % Abs is used to fix problem with negative y-values.
  err = x'*y;

  if profile
    convergence(iter) = err;
  end

  % Relative stopping criteria
  if (abs(err - old_err) / abs(old_err)) < tol_rel
    flag = 3;
    break;
  end

  % Absolute stopping criteria
  if err < tol_abs
    flag = 4;
    break;
  end

  % Stagnation testing
  if dx < eps
    flag = 5;
    break;
  end

  iter = iter + 1;
end

if iter>=max_iter
  flag = 8;
  iter = iter - 1;
end
msg = msg{flag};
}
*/


