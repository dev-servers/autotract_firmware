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
    double angle = new_steering_angle.data / 100;
    app.steering.set_angle(angle);
}
void App::rcv_zero_cmd(const std_msgs::Empty &zero) {
    app.steering.zero_angle();
}
void App::rcv_manual_cmd(const std_msgs::Bool &manual) {
    app.steering.set_manual(manual.data);
}