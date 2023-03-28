#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/color_rgba.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <pololu_3pi_2040_robot.h>

const uint LED_PIN = 25;

// battery publisher
rcl_publisher_t battery_publisher;
std_msgs__msg__UInt16 battery_msg;

// motor subscriber
bool em_stop = false;
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// rgb led subscriberr
rcl_subscription_t rgb_led_sub;
std_msgs__msg__ColorRGBA rgb_led_msg;

void threepi_set_speed_command(float linear_m_s, float angle_rad_s) {
    const float WHEEL_SEPERATION_DIST_M = 0.02;

    float right_vel = (angle_rad_s * WHEEL_SEPERATION_DIST_M) / 2.0 + linear_m_s;
    float left_vel = (linear_m_s * 2.0) - right_vel;

    motors_set_speeds(left_vel, right_vel); 
}

void cmd_vel_callback(const void * msgin)
{
    // TODO the speed range is not yet calibrated at all
    // so to test use a linear.x of 6000 max and -6000 min.
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    if (!em_stop){
        threepi_set_speed_command(msg->linear.x, msg->angular.z);
    }

}

uint8_t rgb_convertor(float value) {
    // float(0-127) -> uint8(0-31), with max and floor
    if( value > 127 ) value = 127.0;
    if( value < 0 ) value = 0.0;
    value = value / 4.0; 
    return (uint8_t) value;
}

void rgb_led_callback(const void * msgin)
{
    // display the RGBA message value on all 6 3pi+ RGB LEDs.
    rgb_color colors[6];
    std_msgs__msg__ColorRGBA * msg = (std_msgs__msg__ColorRGBA *) msgin;
    
    for(int i=0; i<6; i++) {
        colors[i].red = rgb_convertor(msg->r);
        colors[i].green = rgb_convertor(msg->g);
        colors[i].blue = rgb_convertor(msg->b);
    }
    rgb_leds_write(colors, 6, rgb_convertor(msg->a));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret;
    // update battery
    uint16_t level_mv = battery_get_level_millivolts();
    battery_msg.data = level_mv;
    ret = rcl_publish(&battery_publisher, &battery_msg, NULL);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    motors_init();

    rgb_leds_init();
    rgb_leds_off();

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    // battery level publisher
    rclc_publisher_init_default(
        &battery_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
        "battery");

    // motor twist subscriber
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // rgb light subscriber
    rclc_subscription_init_default(
        &rgb_led_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
        "rgb_led");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, 
                                    &cmd_vel_msg, &cmd_vel_callback, 
                                    ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &rgb_led_sub, 
                                    &rgb_led_msg, &rgb_led_callback, 
                                    ON_NEW_DATA);


    gpio_put(LED_PIN, 1);

    // initialize messages
    battery_msg.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
