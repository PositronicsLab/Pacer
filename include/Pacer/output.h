/****************************************************************************
 * Copyright 2014 Samuel Zapolsky
 * This library is distributed under the terms of the Apache V2.0
 * License (obtainable from http://www.apache.org/licenses/LICENSE-2.0).
 ****************************************************************************/
#ifndef OUTPUT_H
#define OUTPUT_H

#include <Pacer/Log.h>

#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include <map>
#include <vector>
#include <string>

#include <Ravelin/VectorNd.h>
#include <Ravelin/MatrixNd.h>
#include <Ravelin/Transform3d.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Vector3d.h>

template < class T >
std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
{
  os << "[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
  {
    os << " " << *ii;
  }
  os << "]";
  return os;
}

static void OUTLOG(const std::map<std::string,Ravelin::VectorNd>& z, std::string name,TLogLevel LL){
  OUT_LOG(LL) << name << " = [\n";
  for (std::map<std::string,Ravelin::VectorNd>::const_iterator it = z.begin(); it != z.end(); it++) {
    std::ostringstream str;
    for(int i=0;i<(*it).second.rows();i++){
      str << std::setprecision(9) << (*it).second[i] << " ";
    }
    OUT_LOG(LL) << (*it).first << " = [" << str.str() << "]\n";
  }
  OUT_LOG(LL) << "]";
}

static void OUTLOG(const std::map<std::string,double>& z, std::string name,TLogLevel LL){
  std::ostringstream str;
  for (std::map<std::string,double>::const_iterator it = z.begin(); it != z.end(); it++) {
    str << (*it).first << " = [" << (*it).second << "]';\n";
  }
  OUT_LOG(LL) << name << " = [\n" << str.str() << "]';";
}

static void OUTLOG(const Ravelin::MatrixNd& M, std::string name,TLogLevel LL){

  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";

}

static void OUTLOG(const Ravelin::SharedConstMatrixNd& M, std::string name,TLogLevel LL){

  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";

}

static void OUTLOG(const Ravelin::Matrix3d& M, std::string name,TLogLevel LL){

  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";

}

static void OUTLOG(const Ravelin::Pose3d& P, std::string name,TLogLevel LL){

  static Ravelin::Matrix3d R;
  R = Ravelin::Matrix3d(P.q);
  std::ostringstream str;
    str << std::setprecision(9)
        << R(0,0) << " "<< R(0,1) << " "<< R(0,2) << " "<< P.x[0] << ";" << std::endl
        << R(1,0) << " "<< R(1,1) << " "<< R(1,2) << " "<< P.x[1] << ";" << std::endl
        << R(2,0) << " "<< R(2,1) << " "<< R(2,2) << " "<< P.x[2] << ";" << std::endl
        << 0  << " "<< 0 << " "<< 0 << " "<< 1 << std::endl;
    OUT_LOG(LL) << name << " = [ %"  << 4 << "x" << 4 << "\n" << str.str() << "];";

}

static void OUTLOG(const Ravelin::VectorNd& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
//    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const std::vector<double>& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const std::vector<int>& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const std::vector<std::string>& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;

}

static void OUTLOG(const std::string& z, std::string name,TLogLevel LL){

    std::ostringstream str;

    OUT_LOG(LL) << name << " = [" << z << "]';" << std::endl;

}

static void OUTLOG(const Ravelin::SVector6d& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const Ravelin::Origin3d& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<3;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const Ravelin::Vector3d& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<3;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const Ravelin::Vector2d& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<2;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";

}

static void OUTLOG(const Ravelin::SharedVectorNd& z, std::string name,TLogLevel LL){

    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';";

}

static void OUTLOG(const Ravelin::AAngled& z, std::string name,TLogLevel LL){

    OUT_LOG(LL) << std::setprecision(9)
                << name << " = <"
                << z.x << " " << z.y << " " << z.z << "> (" << z.angle << ");";

}

static void OUTLOG(double x, std::string name,TLogLevel LL){

    OUT_LOG(LL) << name << " = " << x << ";" << std::endl;

}

#endif
