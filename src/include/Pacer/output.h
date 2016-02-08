/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef OUTPUT_H
#define OUTPUT_H

#include <Pacer/Log.h>

#include <map>
#include <vector>
#include <string>

#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Vector3d.h>
#include <Ravelin/Pose3d.h>

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
  os << "("<< v.size() <<")[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
  {
    os << " " << *ii;
  }
  os << "]";
  return os;
}

template < class T >
std::ostream& operator << (std::ostream& os, const std::map<std::string, T >& v)
{
  os << "("<< v.size() <<")[" << std::endl;
  for (typename std::map<std::string, T >::const_iterator ii = v.begin(); ii != v.end(); ++ii)
  {
    os << " " << ii->first << " = " << ii->second << std::endl;
  }
  os << "]";
  return os;
}

#ifdef LOG_TO_FILE

void OUTLOG(const std::map<std::string,Ravelin::VectorNd>& z, std::string name,TLogLevel LL);
void OUTLOG(const std::map<std::string,double>& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::MatrixNd& M, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SharedConstMatrixNd& M, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Matrix3d& M, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Pose3d& P, std::string name,TLogLevel LL);
void OUTLOG(const boost::shared_ptr<Ravelin::Pose3d>& P, std::string name,TLogLevel LL);
void OUTLOG(const boost::shared_ptr<const Ravelin::Pose3d>& P, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::VectorNd& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<double>& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<int>& z, std::string name,TLogLevel LL);
void OUTLOG(const std::vector<std::string>& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SVector6d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Origin3d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Vector3d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Vector2d& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::SharedVectorNd& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::AAngled& z, std::string name,TLogLevel LL);
void OUTLOG(const Ravelin::Quatd& z, std::string name,TLogLevel LL);
void OUTLOG(const std::string& z, std::string name,TLogLevel LL);
void OUTLOG(double x, std::string name,TLogLevel LL);
#else
#define OUTLOG(value,name,stream)
#endif

#endif
