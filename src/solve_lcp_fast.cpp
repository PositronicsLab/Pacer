/****************************************************************************
 * Copyright 2015 Evan M. Drumwright 
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/

#include <algorithm>
#include <Moby/insertion_sort>
#include <Pacer/project_common.h>

using namespace Ravelin;

/// Get the minimum index of vector v; if there are multiple minima (within zero_tol), returns one randomly 
unsigned rand_min(const VectorNd& v, double zero_tol)
{
  static std::vector<unsigned> minima;
  minima.clear();
  unsigned minv = std::min_element(v.begin(), v.end()) - v.begin();
  minima.push_back(minv);
  for (unsigned i=0; i< v.rows(); i++)
    if (i != minv && v[i] < v[minv] + zero_tol)
      minima.push_back(i);
  return minima[rand() % minima.size()];
}

/// Pivoting rule
unsigned select_pivot(const VectorNd& znbas, const std::vector<unsigned>& nonbas, const std::vector<unsigned>& indices, double zero_tol)
{
  // if nonbas is empty, return INF
  if (nonbas.empty())
    return std::numeric_limits<unsigned>::max();

  // find all indices i of znbas for which znbas[i] < 0
  static std::vector<unsigned> neg;
  neg.clear();
  for (unsigned i=0; i< znbas.size(); i++)
    if (znbas[i] < -zero_tol)
      neg.push_back(i); 

  // of all negative indices, find those which have a contact point with the
  // same link in nonbas
  static std::vector<unsigned> repeated;
  repeated.clear();
  for (unsigned i=0; i< neg.size(); i++)
  {
    // get the z index
    unsigned z_index = nonbas[neg[i]];

    // get the link index
    unsigned link_idx = indices[z_index];

    // loop through all other z indices
    for (unsigned j=0; j< nonbas.size(); j++)
      if (z_index != nonbas[j] && indices[nonbas[j]] == link_idx)
      {
        repeated.push_back(i);
        break;
      }
  }

  // if there are no such contact points, ...
  if (repeated.empty())
  {
    // pick the minimum entry of znbas
    return rand_min(znbas, zero_tol); 
  }
  else if (repeated.size() > 1)
  {
/*
    // there are multiple such contact points, pick the one with the most
    // negative z value
    unsigned most_neg = repeated.front();
    for (unsigned i=1; i< repeated.size(); i++)
      if (znbas[repeated[i]] < znbas[repeated[most_neg]])
        most_neg = repeated[i];
*/
    // there are multiple such contact points, pick cardinally 
    return repeated.front();
  }
  else
  {
    // exactly one entry, return it
    return repeated.front();
  }
}

/// Fast pivoting algorithm for denerate, monotone LCPs with few nonzero, nonbasic variables 
bool lcp_fast(const MatrixNd& M, const VectorNd& q, const std::vector<unsigned>& indices, VectorNd& z, double zero_tol)
{
  const unsigned N = q.rows();
  const unsigned UINF = std::numeric_limits<unsigned>::max();

  // setup static variables
  static std::vector<unsigned> _nonbas, _bas;
  static std::vector<bool> _represented;
  static MatrixNd _Msub, _Mmix;
  static VectorNd _z, _qbas, _w;
  static LinAlgd _LA;

  // verify that indices are the right size
  assert(indices.size() == N);

  // determine the number of links represented
  _represented.clear();
  for (unsigned i=0; i< indices.size(); i++)
    if (indices[i] >= _represented.size())
      _represented.resize(indices[i]+1);

  // indicate that no link is represented in the non-basic set
  std::fill(_represented.begin(), _represented.end(), false);

  // look for trivial solution
  if (N == 0)
  {
    z.set_zero(0);
    return true;
  }

  // set zero tolerance if necessary
  if (zero_tol < 0.0)
    zero_tol = M.rows() * M.norm_inf() * std::numeric_limits<double>::epsilon();

  // get minimum element of q (really w)
  unsigned minw = std::min_element(q.begin(), q.end()) - q.begin();
  if (q[minw] > -zero_tol)
  {
    z.set_zero(N);
    return true;
  }

  // set initial nonbasic set to have a negative variable from each link index
  _nonbas.clear();
  for (unsigned i=0; i< N; i++)
    if (q[i] < zero_tol && !_represented[indices[i]])
    {
      _nonbas.push_back(i);
      _represented[indices[i]] = true;
    }

  // now add contacts from all links not represented
  for (unsigned i=0; i< N; i++)
    if (std::binary_search(_nonbas.begin(), _nonbas.end(), i) && 
        !_represented[indices[i]])
    {
      _nonbas.push_back(i);
      Moby::insertion_sort(_nonbas.begin(), _nonbas.end());
      _represented[indices[i]] = true;
    }

  // setup basic indices
  _bas.clear();
  for (unsigned i=0; i< N; i++)
    if (!std::binary_search(_nonbas.begin(), _nonbas.end(), i))
      _bas.push_back(i);

  // loop for maximum number of pivots
  const unsigned MAX_PIV = std::max(N*N, (unsigned) 1000);
  for (unsigned piv=0; piv < MAX_PIV; piv++)
  {
    // select nonbasic indices
    M.select_square(_nonbas.begin(), _nonbas.end(), _Msub);
    M.select(_bas.begin(), _bas.end(), _nonbas.begin(), _nonbas.end(), _Mmix);
    q.select(_nonbas.begin(), _nonbas.end(), _z);
    q.select(_bas.begin(), _bas.end(), _qbas);
    _z.negate();

    // solve for nonbasic z
    try
    {
      _LA.solve_fast(_Msub, _z);
    }
    catch (SingularException e)
    {
      return false;
    }

    // compute w and find minimum value
    _Mmix.mult(_z, _w) += _qbas;
    minw = (_w.rows() > 0) ? rand_min(_w, zero_tol) : UINF;

    // if w >= 0, check whether any component of z < 0
    if (minw == UINF || _w[minw] > -zero_tol)
    {
      // find the (a) minimum of z
      unsigned minz = select_pivot(_z, _nonbas, indices, zero_tol); 
      if (minz < UINF && _z[minz] < -zero_tol)
      {
        // get the original index and remove it from the nonbasic set
        unsigned idx = _nonbas[minz];
        _nonbas.erase(_nonbas.begin()+minz);
        
        // move index to basic set and continue looping
        _bas.push_back(idx);
        Moby::insertion_sort(_bas.begin(), _bas.end());
      }
      else
      {
        // found the solution
        z.set_zero(N);

        // set values of z corresponding to _z
        for (unsigned i=0, j=0; j < _nonbas.size(); i++, j++)
          z[_nonbas[j]] = _z[i];

        return true;
      }
    }
    else
    {
      // one or more components of w violating w >= 0
      // move component of w from basic set to nonbasic set
      unsigned idx = _bas[minw];
      _bas.erase(_bas.begin()+minw);
      _nonbas.push_back(idx);
      Moby::insertion_sort(_nonbas.begin(), _nonbas.end());

      // look whether any component of z needs to move to basic set
      unsigned minz = select_pivot(_z, _nonbas, indices, zero_tol); 
      if (minz < UINF &&_z[minz] < -zero_tol)
      {
        // move index to basic set and continue looping
        unsigned idx = _nonbas[minz];
        _nonbas.erase(_nonbas.begin()+minz);
        _bas.push_back(idx);
        Moby::insertion_sort(_bas.begin(), _bas.end());
      }
    }
  }

  // if we're here, then the maximum number of pivots has been exceeded
  return false;
}

