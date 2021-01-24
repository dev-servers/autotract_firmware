#include <main.h>

App app;

int main(void) {
    app.init();
    for (;;) {
        app.run();
    }
    return 0;
}

void App::rcv_steering_cmd(const std_msgs::Int64 &new_steering_angle) {
    StepperDirection dir = StepperDirection::ClockWise;
    if (new_steering_angle.data < 0) {
        dir = StepperDirection::CounterClockWise;
        app.steering.stepper.pulse_n_tim(-1 * new_steering_angle.data, dir);
    } else {
        app.steering.stepper.pulse_n_tim(new_steering_angle.data, dir);
    }
}
void App::rcv_zero_cmd(const std_msgs::Empty &zero) {
    app.steering.encoder.set_zero();
}
void App::rcv_manual_cmd(const std_msgs::Bool &manual) {
    if (manual.data)
        app.steering.stepper.disable();
    else
        app.steering.stepper.enable();
}