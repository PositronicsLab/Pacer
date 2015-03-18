
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
        &eef_names = Utility::get_variable<std::vector<std::string> >(plugin_namespace+".id");

    std::vector<double>
        &Kp = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Kv = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kp"),
        &Ki = Utility::get_variable<std::vector<double> >(plugin_namespace+".gains.kp");

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
/*
    // --------------------------- WORKSPACE FEEDBACK --------------------------
    static int &WORKSPACE_FEEDBACK = Utility::get_variable<int>("controller.error-feedback.operational-space.active");
    if(WORKSPACE_FEEDBACK){
      // CURRENTLY THIS IS ONLY FORCE
      // BUT IT CAN BE ACCELERATIONS TOO
      static int &FEEDBACK_ACCEL = Utility::get_variable<int>("controller.error-feedback.operational-space.accel");
      std::vector<Ravelin::Matrix3d> W(boost::assign::list_of(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity())(Ravelin::Matrix3d::identity()).convert_to_container<std::vector<Ravelin::Matrix3d> >() );
      static std::vector<double>
          &Kp = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.kp"),
          &Kv = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.kv"),
          &Ki = Utility::get_variable<std::vector<double> >("controller.error-feedback.operational-space.gains.ki");

      Ravelin::VectorNd fb = Ravelin::VectorNd::zero(NUM_JOINT_DOFS);
      eef_stiffness_fb(Kp,Kv,Ki,x_des,xd_des,data->q,data->qd,fb);

      if(FEEDBACK_ACCEL)
        qdd_des += fb;
      else
        ufb += fb;
    }
  }
  
  */
