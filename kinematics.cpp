#include <Control.h>

static std::vector<Moby::RigidBodyPtr> links,eefs;
unsigned num_feet;

/// 4 foot (body-fixed) state-space traj to joint-space traj
std::vector<Vec>& trajectoryIK(Moby::RCArticulatedBody abrobot, const std::vector<std::vector<Ravelin::Vector3d> >& feet_trajectory,
                               std::vector<Vec>& joint_trajectory,const std::vector<std::string>& eef_names){
    if(eefs.size() != eef_names.size()){
        links = abrobot.get_links();
        num_feet = eef_names.size();
        eefs(num_feet);

          for(unsigned i=0;i<links.size();i++)
              for(unsigned j=0;j<eef_names_.size();j++)
                  if(eef_names_[j].compare(links_[i]->id) == 0)
                      eefs[j] = links_[i];
    }

    static Mat workM;
    static Vec workv;

    // NST format 4-foot trajectory (fixed base)
    Vec foot_vec(num_feet*3),foot_pos(num_feet*3);
    num_steps =  feet_trajectory.size();
    feet_trajectory.resize(num_steps);
    for(int i=0;i<num_steps;i++){
        static std::vector<Ravelin::Vector3d> foot_pos(num_feet);
        for(int f = 0;f<num_feet;f++){
            Ravelin::Pose3d link_pose = *(eefs[f]->get_pose());
            link_pose.x[0] += 0.04;
            link_pose.update_relative_pose(Moby::GLOBAL);

            foot_vec[f] = feet_trajectory[i][f][0];
            foot_vec[f+n] = feet_trajectory[i][f][1];
            foot_vec[f+n*2] = feet_trajectory[i][f][2];
            foot_pos[f] = link_pose.x[0];
            foot_pos[f+n] = link_pose.x[1];
            foot_pos[f+n*2] = link_pose.x[2];
        }

        // find IK solution
        static Vec foot_position_err = foot_pos,dstep,dq,q;
        foot_position_err -= foot_vec;
        double pos_error = foot_position_err.norm();
        while(pos_error > 1e-4){
            // step toward IK solution
            determine_N_D2(feet,N,ST);
            static Mat J(3,num_feet*3);
            J.set_sub_mat(0,0,ST.get_sub_mat(0,3,0,num_feet*2,workM));
            J.set_sub_mat(0,num_feet,N.get_sub_mat(0,3,0,num_feet,workM));


            static Vec dstep = foot_vec;
            dstep *= 1e-3;
            J.mult(dstep,dq);
            abrobot->get_generalized_coordinates(Moby::DynamicBody::eSpatial,q);
            abrobot->set_generalized_coordinates(q+=dq);

            for(int f = 0;f<num_feet;f++){
                Ravelin::Pose3d link_pose = *(eefs[f]->get_pose());
                link_pose.x[0] += 0.04;
                link_pose.update_relative_pose(Moby::GLOBAL);
                foot_pos[f] = link_pose.x[0];
                foot_pos[f+n] = link_pose.x[1];
                foot_pos[f+n*2] = link_pose.x[2];
            }
            foot_position_err = foot_pos;
            foot_position_err -= foot_vec;
            pos_error = foot_position_err.norm();
            outlog("feet",foot_pos);
            outlog("foot_position_err",foot_position_err);
            outlog("feet_goal",foot_vec);
            outlog("position",q);
            outlog("step",dq);
            std::cout << "step_size = " << dq.norm() <<std::endl<<std::endl;;
        }
    }
}

void determine_N_D(std::vector<ContactData>& contacts, Mat& N, Mat& D)
{
  int nc = contacts.size();
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  std::vector<RigidBodyPtr> eefs(nc);

  for(unsigned i=0;i<links.size();i++)
    for(unsigned j=0;j<nc;j++)
      if(contacts[j].name.compare(links[i]->id) == 0)
        eefs[j] = links[i];

  static Mat J, Jsub;
  static Vec col;

  // resize temporary N and ST
  N.resize(NJOINT+6,nc);
  D.resize(NJOINT+6,nc*nk);

  J.resize(NSPATIAL, NJOINT);

  // loop over all contacts
  for(int i = 0; i < nc; i++){
    // get the contact point and normal
    ContactData& c = contacts[i];

    // generate the contact tangents here...
    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);
    Vector3d torque;
    torque.set_zero();

    RigidBodyPtr sbfoot = eefs[i];

    Vec col(NSPATIAL);
    AAngled aa(0,0,1,0);
    Origin3d o(c.point);
    boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
    SForced sfn(c.normal,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
    N.set_column(i,col);
    for(int k=0;k<nk;k++){
      if(k%2 == 0) {
        SForced sfs(tan1,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
      } else {
        SForced sft(tan2,torque,pose);
        abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
      }
      if(k>=2) col.negate();
      D.set_column(i*nk + k,col);
    }
  }
}

void determine_N_D2(std::vector<ContactData>& contacts, Mat& N, Mat& ST)
{
  int nc = contacts.size(),nk = 2;
  const std::vector<RigidBodyPtr>& links = abrobot->get_links();
  std::vector<RigidBodyPtr> eefs(nc);

  for(unsigned i=0;i<links.size();i++)
    for(unsigned j=0;j<nc;j++)
      if(contacts[j].name.compare(links[i]->id) == 0)
        eefs[j] = links[i];

  static Mat J, Jsub;
  static Vec col;

  // resize temporary N and ST
  N.resize(NJOINT+6,nc);
  ST.resize(NJOINT+6,nc*nk);

  J.resize(NSPATIAL, NJOINT);

  // loop over all contacts
  for(int i = 0; i < nc; i++){
    // get the contact point and normal
    ContactData& c = contacts[i];

    // generate the contact tangents here...
    Vector3d tan1, tan2;
    Vector3d::determine_orthonormal_basis(c.normal, tan1, tan2);
    Vector3d torque;
    torque.set_zero();

    RigidBodyPtr sbfoot = eefs[i];

    Vec col(NSPATIAL);
    AAngled aa(0,0,1,0);
    Origin3d o(c.point);
    boost::shared_ptr<const Ravelin::Pose3d> pose(new Pose3d(aa,o));
    SForced sfn(c.normal,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfn, c.point, col);
    N.set_column(i,col);

    SForced sfs(tan1,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sfs, c.point, col);
    ST.set_column(i,col);

    sft = SForced(tan2,torque,pose);
    abrobot->convert_to_generalized_force(sbfoot,sft, c.point, col);
    ST.set_column(i+nc,col);
  }
}

// kinematics solver
/*

  */
