#ifndef PIDCONTROLLER
#define PIDCONTROLLER

class PIDController {
  public:
    float ffw_gain;
    float p_gain;
    float i_gain;
    float d_gain;
    float evaluate(float target, float measurement, bool freeze_integration) {
      float controlerror(target-measurement);
      float output(ffw_gain*target+p_gain*controlerror+d_gain*(controlerror-lastcontroldeviation)+i_gain*integratedcontroldeviation);
      lastcontroldeviation = controlerror;
      integratedcontroldeviation += controlerror;
    }
  protected:
    float lastcontroldeviation(0);
    float integratedcontroldeviation(0);
}

#endif
