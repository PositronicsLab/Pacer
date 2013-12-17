#include <project_common.h>

void OUTLOG(const Ravelin::MatrixNd& M, std::string name){
#ifndef NDEBUG
  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(logINFO) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];" << std::endl;
#endif
}

void OUTLOG(const Ravelin::VectorNd& z, std::string name){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(logINFO) << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';" << std::endl;
#endif
}

void OUTLOG(const Ravelin::SharedVectorNd& z, std::string name){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(logINFO) << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';" << std::endl;
#endif
}
