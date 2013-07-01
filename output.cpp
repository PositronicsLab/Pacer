#include <iomanip>
#include <sstream>
#include <Moby/MatrixN.h>
#include <Moby/VectorN.h>

#define NO_OUTPUT

void outlog(const Moby::MatrixN& M, std::string name){
#ifndef NO_OUTPUT
    std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    std::cout << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];" << std::endl;
#endif
}

void outlog(const Moby::VectorN& z, std::string name){
#ifndef NO_OUTPUT
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    std::cout << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';" << std::endl;
#endif
}

void outlog2(const Moby::MatrixN& M, std::string name){
    std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    std::cout << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];" << std::endl;
}

void outlog2(const Moby::VectorN& z, std::string name){
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    std::cout << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';" << std::endl;
}
