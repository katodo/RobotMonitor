#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico/binary_info.h"
#include "ws2812.pio.h"

#include <rcl/arguments.h>
#include <rcl/time.h>
#include <rcl/init.h>
#include <rcl/logging.h>
#include <rcl/node.h>
#include <rcl/subscription.h>
#include <rcl/wait.h>
#include <rcutils/error_handling.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/color_rgba.h>
#include <sensor_msgs/msg/battery_state.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

const uint LED_PIN = 25;
const int WS2812_PIN = 23;

static inline void put_pixel(uint32_t pixel_grb) {
  pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)(r) << 8) |
         ((uint32_t)(g) << 16) |
         (uint32_t)(b);
}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t publisher_batt;
sensor_msgs__msg__BatteryState msgBattery;
rosidl_runtime_c__float__Sequence battVoltage;
const uint NUMBEROFFCELL = 6;

//rcl_subscription_t sub_rgb = rcl_get_zero_initialized_subscription();

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    ret += rcl_publish(&publisher_batt, &msgBattery, NULL);
    msg.data++;
    msgBattery.capacity++;
    msgBattery.cell_voltage.data[0] += 0.05;
}

int main()
{
    //WS2812
    PIO pio = pio0;
    int sm = 0;
    char str[12];
    
    // Micro_ROS
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;


    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, true);
    put_pixel(urgb_u32(0x80, 0, 0));  // Red




    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);





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

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");
        
    msgBattery.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    msgBattery.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
	msgBattery.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    msgBattery.charge = 2;
    msgBattery.current = 10;
    battVoltage.size = NUMBEROFFCELL;
    msgBattery.cell_voltage.capacity = NUMBEROFFCELL;
    msgBattery.cell_voltage.size = NUMBEROFFCELL;
    msgBattery.cell_voltage.data = ( float*) malloc( msgBattery.cell_voltage.capacity * sizeof(float));

    msgBattery.cell_voltage.data[0] = 0;
    msgBattery.cell_voltage.data[1] = 1;
    msgBattery.cell_voltage.data[3] = 3;


    /* ************************************** */
    // https://github.com/tiiuae/rclgo/blob/530f86adbc7be63d03f690770d4944ca252d843a/cmd/ros2ctest/roslogging.c
    rcl_ret_t rc;
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    rc = rcl_subscription_init(&subscription, &node, rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__ColorRGBA(), "/rossina/rusina", &subscription_options);
    if (rc != RCL_RET_OK) {
        printf("rcl_subscription_init error '%d' '%s'\n", rc, rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    rmw_message_info_t rmw_message_info = rmw_get_zero_initialized_message_info();

    //  ros2 topic pub /rossina/rusina std_msgs/msg/ColorRGBA "{r: 128, g: 0.0, b: 0.0, a: 0.0"}
    /* ************************************** */



    rclc_publisher_init_default( 
        &publisher_batt, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), 
        "pBatt");


    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);
    put_pixel(urgb_u32(0xff, 0, 0));  // Red
    sleep_ms(500);
    put_pixel(urgb_u32(0, 0xff, 0));  // Green
    sleep_ms(500);
    put_pixel(urgb_u32(0, 0, 0xff));  // Blue
    sleep_ms(500);
    put_pixel(urgb_u32(0xff, 0xff, 0));  // Purple
    sleep_ms(500);
    put_pixel(urgb_u32(0, 0xff, 0xff));  // Cyan
    sleep_ms(500);
    put_pixel(urgb_u32(0xff, 0xff, 0xff));  // White
    sleep_ms(500);
    put_pixel(urgb_u32(0, 0, 0));  // Black or off
    sleep_ms(500);

    msg.data = 0;

    std_msgs__msg__ColorRGBA* ros2_msg_receive_buffer = std_msgs__msg__ColorRGBA__create();
    
    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));


        //sleep(2);

        rc = rcl_take(&subscription, ros2_msg_receive_buffer, &rmw_message_info, NULL);
        if (rc != RCL_RET_OK) {
            printf("rcl_take error '%d' '%s'\n", rc, rcutils_get_error_string().str);
            rcutils_reset_error();
        }

        //printf("ColorRGBA: %f %f %f %f\n", ros2_msg_receive_buffer->r, ros2_msg_receive_buffer->b, ros2_msg_receive_buffer->g, ros2_msg_receive_buffer->a);

        //std_msgs__msg__ColorRGBA__fini(ros2_msg_receive_buffer);

        //  ros2 topic pub /rossina/rusina std_msgs/msg/ColorRGBA "{r: 255, g: 0, b: 0, a: 0.0"}
        //  ros2 topic pub /rossina/rusina std_msgs/msg/ColorRGBA "{r: 0, g: 255, b: 0, a: 0.0"}
        //  ros2 topic pub /rossina/rusina std_msgs/msg/ColorRGBA "{r: 0, g: 0, b: 255, a: 0.0"}
        put_pixel(urgb_u32(ros2_msg_receive_buffer->r, ros2_msg_receive_buffer->g, ros2_msg_receive_buffer->b)); 
        //sleep_ms(100);
    }
    return 0;
}
