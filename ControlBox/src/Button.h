//
// Created by ub1404 on 15. 9. 18.
//

#ifndef PLAYGROUND_BUTTON_H
#define PLAYGROUND_BUTTON_H
class Button {
public:
    enum ButtonStates {
        BUTTON_ON = 1,
        BUTTON_OFF = 0,
    };

    enum EdesState {
        NO_EDGE = 0,
        ON_TO_OFF = -2,
        OFF_TO_ON = 2,
    };

    Button() {
        last_state = current_state = BUTTON_OFF;
        tick_count_buffer = tick_count = 0;
        sensitivity = 0.25;    //tuned
        pressed_for = 0.0;
    }

    /*
     * call this method in haptic or control loop.
     * state : BUTTON_ON or BUTTON_OFF
     * t	 : current time in seconds unit.
     */
    void update(ButtonStates state, double t) {
        current_state = state;
        diff = 2 * (current_state - last_state);

        if (t - last_tick_time > sensitivity) {
            tick_count_buffer = tick_count;
            tick_count = 0;
        }

        if (diff == OFF_TO_ON) {
            pressed_start = t;
        }

        if (diff == ON_TO_OFF) {
            tick_count++;
            last_tick_time = t;
            pressed_for = 0.0;
        }

        if (current_state == BUTTON_ON) {
            pressed_for = t - pressed_start;
        }

        last_state = current_state;
    }

    /*
     * Return elapsed time from button started being pressed.
     */
    double pressedFor() {
        return pressed_for;
    }

    /*
     * return BUTTON_ON or BUTTON_OFF
     */
    int isPressed() {
        return current_state;
    }

    /*
     * return NO_EDGE or ON_TO_OFF or OFF_TO_ON
     */
    int getEdge() {
        return diff;
    }

    /*
     * set acceptable time interval in seconds between each ticks.
     */
    void setTickSensitivity(double dt) {
        this->sensitivity = dt;
    }

    /*
     * get ticks.
     * As tick count is volatile quantity, it is cleared after being read.
     */
    int getTicks() {
        int ret = tick_count_buffer;
        tick_count_buffer = 0;
        return ret;
    }

private:
    int current_state;
    int last_state;
    int diff;

    double pressed_start;
    double pressed_for;

    int tick_count;
    int tick_count_buffer;
    double sensitivity;
    double last_tick_time;
};
#endif //PLAYGROUND_BUTTON_H
