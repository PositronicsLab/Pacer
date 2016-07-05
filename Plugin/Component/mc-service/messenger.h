#include <Ravelin/RCArticulatedBodyd.h>



//------------------------------------------------------------------------------
namespace Pacer {
  //----------------------------------------------------------------------------
  namespace Service {
    typedef std::pair<
    std::map<std::string,std::vector<double> > , // Joints
    std::map<std::string,std::vector<double> > // Links
    > TimeSeriesMap;
    
    typedef std::pair<
    std::map<std::string,std::vector<double> > , // Joints
    std::map<std::string,std::vector<double> > // Links
    > ModelState;
    
    //--------------------------------------------------------------------------
    class Translator {
      /// propietary functions
      void set_data_interval(double min_t,double max_t,double res_t){
        _min_t = min_t;
        _max_t = max_t;
        _res_t = res_t;
      }
      void get_data_interval(double& min_t, double& max_t, double& res_t){
        min_t = _min_t;
        max_t = _max_t;
        res_t = _res_t;
      }
      
      static void add_model_data(double t, boost::shared_ptr<Ravelin::RCArticulatedBodyd>& model, TimeSeriesMap& time_series){
        
      };
      
      void set_model_data(const TimeSeriesMap& time_series);
      void get_model_data(TimeSeriesMap& time_series);
      
      void client_connect_to_server();
      void server_open_connection();
      
      void setup_message_client();
      void send_message_client();

    private:
      double _min_t, _max_t, _res_t;
    };
    //--------------------------------------------------------------------------
  }// end Service
  //----------------------------------------------------------------------------
}// end Pacer
//------------------------------------------------------------------------------
