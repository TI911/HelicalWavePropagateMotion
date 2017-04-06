
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <joy_handler_hori/JoySelectedData.h>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1
// @ref http://wiki.ros.org/HORIjoy

#define HORI_BUTTON_ACTION_SQUARE     0
#define HORI_BUTTON_ACTION_CROSS      1
#define HORI_BUTTON_ACTION_CIRCLE     2
#define HORI_BUTTON_ACTION_TRIANGLE   3
#define HORI_BUTTON_REAR_LEFT_1       4
#define HORI_BUTTON_REAR_RIGHT_1      5
#define HORI_BUTTON_REAR_LEFT_2       6
#define HORI_BUTTON_REAR_RIGHT_2      7
#define HORI_BUTTON_SELECT            8
#define HORI_BUTTON_START             9
#define HORI_BUTTON_STICK_LEFT        10
#define HORI_BUTTON_STICK_RIGHT       11
#define HORI_BUTTON_PS                12

#define HORI_AXIS_STICK_LEFT_LEFTWARDS    0
#define HORI_AXIS_STICK_LEFT_UPWARDS      1
#define HORI_AXIS_STICK_RIGHT_LEFTWARDS   2
#define HORI_AXIS_STICK_RIGHT_UPWARDS     3
#define HORI_AXIS_CROSS_UP_DOWN           5
#define HORI_AXIS_CROSS_RIGHT_LEFT        4


class JoyHandlerHORI {
 public:
  static void Initialize() {
    ros::NodeHandle node;
    sub_joy_ = node.subscribe("joy", 1, CallBackOfJoy);
    pub_joy_selected_data_ = node.advertise<joy_handler_hori::JoySelectedData>("joy_selected_data", 1);
  }
  static void CallBackOfJoy(sensor_msgs::Joy joy) {
    joy_handler_hori::JoySelectedData joy_selected_data;

    joy_selected_data.joy_stick_r_x_rightwards = -joy.axes[HORI_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_selected_data.joy_stick_r_y_upwards    =  joy.axes[HORI_AXIS_STICK_RIGHT_UPWARDS];
    joy_selected_data.joy_stick_l_x_rightwards = -joy.axes[HORI_AXIS_STICK_LEFT_LEFTWARDS];
    joy_selected_data.joy_stick_l_y_upwards    =  joy.axes[HORI_AXIS_STICK_LEFT_UPWARDS];

    if (joy.buttons[HORI_BUTTON_ACTION_CIRCLE]) joy_selected_data.button_circle     = true;
    if (joy.buttons[HORI_BUTTON_ACTION_TRIANGLE]) joy_selected_data.button_triangle = true;
    if (joy.buttons[HORI_BUTTON_ACTION_SQUARE]) joy_selected_data.button_square     = true;
    if (joy.buttons[HORI_BUTTON_ACTION_CROSS]) joy_selected_data.button_cross       = true;

    if (joy.buttons[HORI_BUTTON_REAR_RIGHT_1]) joy_selected_data.button_r1 = true;
    if (joy.buttons[HORI_BUTTON_REAR_RIGHT_2]) joy_selected_data.button_r2 = true;
    if (joy.buttons[HORI_BUTTON_REAR_LEFT_1]) joy_selected_data.button_l1  = true;
    if (joy.buttons[HORI_BUTTON_REAR_LEFT_2]) joy_selected_data.button_l2  = true;

    if (joy.buttons[HORI_BUTTON_STICK_RIGHT]) joy_selected_data.button_r3 = true;
    if (joy.buttons[HORI_BUTTON_STICK_LEFT]) joy_selected_data.button_l3 = true;

    /*    if (joy.buttons[HORI_BUTTON_CROSS_UP]) joy_selected_data.cross_key_up = true;
    if (joy.buttons[HORI_BUTTON_CROSS_DOWN]) joy_selected_data.cross_key_down = true;
    if (joy.buttons[HORI_BUTTON_CROSS_RIGHT]) joy_selected_data.cross_key_right = true;*/

    if (joy.buttons[HORI_BUTTON_SELECT]) joy_selected_data.button_select = true;
    if (joy.buttons[HORI_BUTTON_START]) joy_selected_data.button_start   = true;
    if (joy.buttons[HORI_BUTTON_PS]) joy_selected_data.button_ps    = true;

    pub_joy_selected_data_.publish(joy_selected_data);
  }

 private:
  static ros::Subscriber sub_joy_;
  static ros::Publisher pub_joy_selected_data_;

};

ros::Subscriber JoyHandlerHORI::sub_joy_;
ros::Publisher JoyHandlerHORI::pub_joy_selected_data_;


int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_handler_HORI");
  JoyHandlerHORI::Initialize();
  ROS_INFO("Joy handler HORI was initialized!");
  ros::spin();
}



