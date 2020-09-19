#ifndef LIMITINGCONTROLLER
#define LIMITINGCONTROLLER

class LimitingController{
  public:
    LimitingController(float ffw, float p):ffw_gain(ffw), p_gain(p) {}
//    LimitingController(float ffw, float p, float i, float d):ffw_gain(ffw), p_gain(p), i_gain(i), d_gain(d), last_controlerror_sign(0), upper_integratedcontroldeviation(0), lower_integratedcontroldeviation(0) {}
    float ffw_gain;
    float p_gain;
//    float i_gain;
//    float d_gain;
    float evaluate(float upperlimit, float lowerlimit, float measurement, float control) {
      if(measurement>upperlimit){
        return min(ffw_gain*upperlimit+p_gain*(upperlimit-measurement),control);
      }
      if(measurement<lowerlimit){
        return max(ffw_gain*lowerlimit+p_gain*(lowerlimit-measurement),control);
      }
      return control;


      
//      float output(control);
//      float outputpid(0);
//      float controlerror(0);
//      bool positivecontrolreduceserror(p_gain>0);
//      bool override_control;
//      
//      if(measurement<lowerlimit) {
//        controlerror = measurement-lowerlimit;
//        if(last_controlerror_sign>-1) {
//          lower_lastcontroldeviation = 0;
//        }
//        outputpid = ffw_gain*lowerlimit + p_gain*controlerror + d_gain*(controlerror-lower_lastcontroldeviation) + i_gain*lower_integratedcontroldeviation;
//        
//        override_control = (positivecontrolreduceserror&&(control<outputpid))||((!positivecontrolreduceserror)&&(control>outputpid));
//        if(override_control) {
//          output = outputpid;
//          last_controlerror_sign = -1;
//          lower_integratedcontroldeviation += controlerror;
//          lower_lastcontroldeviation = controlerror;
//        }
//      }
//      else 
//      {
//        if(measurement>upperlimit) {
//          controlerror = measurement-upperlimit;
//          if(last_controlerror_sign<1) {
//            upper_lastcontroldeviation = 0;
//          }
//          outputpid = ffw_gain*upperlimit + p_gain*controlerror + d_gain*(controlerror-lower_lastcontroldeviation) + i_gain*lower_integratedcontroldeviation;
//        
//          override_control = (positivecontrolreduceserror&&(control>outputpid))||((!positivecontrolreduceserror)&&(control<outputpid));
//          if(override_control) {
//            output = outputpid;
//            last_controlerror_sign = 1;
//            upper_integratedcontroldeviation += controlerror;
//            upper_lastcontroldeviation = controlerror;
//          }
//        }
//        else {
//          last_controlerror_sign = 0;
//          lower_integratedcontroldeviation *=0.99;
//          upper_integratedcontroldeviation *=0.99;
//        }
//      }
//      return output;
    }
//    bool limit_active() {
//      return last_controlerror_sign!=0;
//    }
//  protected:
//    int last_controlerror_sign;
//    float lower_lastcontroldeviation;
//    float lower_integratedcontroldeviation;
//    float upper_lastcontroldeviation;
//    float upper_integratedcontroldeviation;
};

#endif
