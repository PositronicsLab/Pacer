#include <project_common.h>

void OUTLOG(const Ravelin::MatrixNd& M, std::string name,TLogLevel LL){
#ifndef NDEBUG
  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";
#endif
}

void OUTLOG(const Ravelin::SharedConstMatrixNd& M, std::string name,TLogLevel LL){
#ifndef NDEBUG
  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";
#endif
}

void OUTLOG(const Ravelin::Matrix3d& M, std::string name,TLogLevel LL){
#ifndef NDEBUG
  std::ostringstream str;
    for(int i=0;i<M.rows();i++){
        for(int j=0;j<M.columns();j++)
            str << std::setprecision(9) << M(i,j) << " ";
        if(i+1 != M.rows()) str << ";" << std::endl;
    }
    OUT_LOG(LL) << name << " = [ %"  << M.rows() << "x" << M.columns() << "\n" << str.str() << "];";
#endif
}

void OUTLOG(const Ravelin::Pose3d& P, std::string name,TLogLevel LL){
#ifndef NDEBUG
  static Ravelin::Matrix3d R;
  R = Ravelin::Matrix3d(P.q);
  std::ostringstream str;
    str << std::setprecision(9)
        << R(0,0) << " "<< R(0,1) << " "<< R(0,2) << " "<< P.x[0] << ";" << std::endl
        << R(1,0) << " "<< R(1,1) << " "<< R(1,2) << " "<< P.x[1] << ";" << std::endl
        << R(2,0) << " "<< R(2,1) << " "<< R(2,2) << " "<< P.x[2] << ";" << std::endl
        << 0  << " "<< 0 << " "<< 0 << " "<< 1 << std::endl;
    OUT_LOG(LL) << name << " = [ %"  << 4 << "x" << 4 << "\n" << str.str() << "];";
#endif
}

void OUTLOG(const Ravelin::VectorNd& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
//    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const std::vector<double>& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const std::vector<int>& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const std::vector<std::string>& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.size();i++)
        str << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
#endif
}

void OUTLOG(const Ravelin::SVector6d& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " %{"  << z.size() << "%} = [" << str.str() << "]';" << std::endl;
//    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const Ravelin::Origin3d& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<3;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const Ravelin::Vector3d& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<3;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const Ravelin::Vector2d& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<2;i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [" << str.str() << "]';";
#endif
}

void OUTLOG(const Ravelin::SharedVectorNd& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    std::ostringstream str;
    for(int i=0;i<z.rows();i++)
        str << std::setprecision(9) << z[i] << " ";
    OUT_LOG(LL) << name << " = [ %"  << z.rows() << "\n" << str.str() << "]';";
#endif
}

void OUTLOG(const Ravelin::AAngled& z, std::string name,TLogLevel LL){
#ifndef NDEBUG
    OUT_LOG(LL) << std::setprecision(9)
                << name << " = <"
                << z.x << " " << z.y << " " << z.z << "> (" << z.angle << ");";
#endif
}

void OUTLOG(double x, std::string name,TLogLevel LL){
#ifndef NDEBUG
    OUT_LOG(LL) << name << " = " << x << ";" << std::endl;
#endif
}
