#ifndef BUTTONSTATEMACHINE
#define BUTTONSTATEMACHINE

#define SHORT_DURATION_MS 500
#define LONG_DURATION_MS 2000

enum ButtonActions {
  invalid,
  short_press,
  long_press,
  short_pause,
  long_pause,
  incomplete
};

class ButtonStateMachine
{
  public:
    ButtonStateMachine(int attachTo) : pin(attachTo) {}//Pin is required to discern between rising and falling flanks.
    int update_status();
    int current_status;
  protected:
    void update_status_array(int state);
    int detect_button_action();
    int match_pattern();
    int   pin;                                //Watch this pin.
    bool last_button_state;
    unsigned long last_value_change_time;
    unsigned int last_button_actions[5];
};

int ButtonStateMachine::update_status() {
  update_status_array(detect_button_action());
  return match_pattern();
}

int ButtonStateMachine::match_pattern() {
  // This assumes button press patterns consist of a (very) long pause, then one or two key presses (with a not-too-long pause in between) and a (very) long pause afterwards.
  // Not all possible patterns implemented (yet)
  if((last_button_actions[4]==invalid)&&(last_button_actions[3]!=invalid)){ //if last action is not invalid, pattern may be incomplete; if second-to-last action is invalid, pattern is old.
    if(last_button_actions[2]==invalid){// single-press pattern
      if(last_button_actions[3]==short_press) {
        current_status = 1;
      }
      else if(last_button_actions[3]==long_press) {
        current_status = 2;
      }
    }
    else {
      if(last_button_actions[0]==invalid) {
        if((last_button_actions[1]==short_press)&&(last_button_actions[3]==short_press)) {// double short-press
          current_status = 3;
        }
        if((last_button_actions[1]==long_press)&&(last_button_actions[3]==long_press)) {// double long-press
          current_status = 4;
        }
      }
      else { // invalid pattern, since we detect only patterns consisting of 2 key presses and a pause in between
        current_status = 0;
      }
    }
  }
  else {
    current_status = 0;
  }
  return current_status;
}

int ButtonStateMachine::detect_button_action() {
  bool current_button_state(digitalRead(pin)==HIGH);
  unsigned long current_time = millis();
  int new_status = invalid;
  if((current_button_state!=last_button_state)&&(current_time-last_value_change_time>40)) {
    if(last_button_state){// button is released
      if(current_time-last_value_change_time<SHORT_DURATION_MS) {
        new_status = short_press;
      }
      else {
        if(current_time-last_value_change_time<LONG_DURATION_MS) {
          new_status = long_press;
        }
        else {
          new_status = invalid;
        }
      }
    }
    else { // button is pressed
      if(current_time-last_value_change_time<SHORT_DURATION_MS) {
        new_status = short_pause;
      }
      else {
        if(current_time-last_value_change_time<LONG_DURATION_MS) {
          new_status = long_pause;
        }
        else {
          new_status = invalid;
        }
      }
    }
    last_value_change_time = current_time;
    last_button_state = current_button_state;
    return new_status;
  }
  else {
    if(current_time-last_value_change_time>LONG_DURATION_MS) {
      return invalid;
    }
    else {
      return incomplete;
    }
  }
}

void ButtonStateMachine::update_status_array(int state) {
  if((state!=incomplete)&&!(last_button_actions[4]==invalid&&state==invalid)){
    for (int i = 0;i<4;i++){
      last_button_actions[i] = last_button_actions[i+1];
    }
    last_button_actions[4] = state;
  }
}
#endif
