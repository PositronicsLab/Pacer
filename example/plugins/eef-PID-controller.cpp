
class eefPID : public ControllerModule {
public:
  eefPID(){}
  eefPID(std::string n) : name(n){ init(); }

  std::string name;

  std::vector<EndEffector>
                    eefs;

  std::vector<Ravelin::Vector3d>
                    x_des,
                    xd_des,
                    x,
                    xd;

  std::vector<std::string> eef_names;

  void init(){
    std::vector<std::string>
        &eef_names = CVarUtils::GetCVarRef<std::vector<std::string> >(plugin_namespace+".id");

    std::vector<double>
        &Kp = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Ki = CVarUtils::GetCVarRef<std::vector<double> >(plugin_namespace+".gains.kp");

    for(int i=0;i<eef_names.size();i++){
        gains[eef_names[i]].kp = Kp[i];
        gains[eef_names[i]].kv = Kv[i];
        gains[eef_names[i]].ki = Ki[i];
    }
  }

  std::string to_string(unsigned i)
  {
    std::ostringstream oss;
    oss << i;
    return oss.str();
  }

  void update(){
    // get the two gains
    for (unsigned i=0; i< eefs.size(); i++)
    {
      Ravelin::Vector3d err;
      const std::string& eef_name = eefs[i].id;
      for (unsigned j=0; j< 3; j++)
      {
        const double KP = gains[eef_name+to_string(j)].kp;
        const double KV = gains[eef_name+to_string(j)].kv;
        const double KI = gains[eef_name+to_string(j)].ki;

        // add feedback torque to joints
        double perr = x_des[i][j] - x[i][j];
        gains[eef_name+to_string(j)].perr_sum += perr;
        double ierr = gains[eef_name].perr_sum;
        double derr = xd_des[i][j] - xd[i][j];

        err[j] = perr*KP + derr*KV + ierr*KI;
      }
    }
  }
};
