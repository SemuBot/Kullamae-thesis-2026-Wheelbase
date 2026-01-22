/*
 * cmd_vel.c
 *
 *  Created on: Nov 22, 2024
 *      Author: medved
 */

#include "cmd_vel.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h" // For CDC_Transmit_FS
#include "motor.h"       // For motor structures and functions

#define BUFFER_SIZE 256

// Static buffer to accumulate incoming characters from USB CDC
static char cdc_buffer[BUFFER_SIZE];
static uint16_t buffer_index = 0;



void Process_CmdVel(CmdVel *cmd_vel) {
    extern motor_st motor1, motor2, motor3;

    /*
    char debug_msg[128];
    snprintf(
        debug_msg,
        sizeof(debug_msg),
        "Rx OK: lin_x=%.2f, lin_y=%.2f, ang_z=%.2f\n",
        cmd_vel->linear_x,
        cmd_vel->linear_y,
        cmd_vel->angular_z
    );
    CDC_Transmit_FS((uint8_t *)debug_msg, strlen(debug_msg));
    */

    // Calculate the required duty cycles for each motor based on the command
    calculate_motor_duty_cycles(cmd_vel->linear_x, cmd_vel->linear_y, cmd_vel->angular_z,
                               &motor1, &motor2, &motor3);
}



void Receive_CmdVel(uint8_t* buf, uint32_t len, CmdVel *cmd_vel) {
    for (uint32_t i = 0; i < len; i++) {
        char received_char = buf[i];


        if (received_char == '\n' || received_char == '\r' || buffer_index >= BUFFER_SIZE - 1) {
            cdc_buffer[buffer_index] = '\0';

            if (buffer_index > 0) {
                if (strstr(cdc_buffer, "linear_x") && strstr(cdc_buffer, "angular_z")) {
                    int items_parsed = sscanf(
                        cdc_buffer,
                        "{linear_x:%f,linear_y:%f,linear_z:%f,angular_x:%f,angular_y:%f,angular_z:%f}",
                        &cmd_vel->linear_x,
                        &cmd_vel->linear_y,
                        &cmd_vel->linear_z,
                        &cmd_vel->angular_x,
                        &cmd_vel->angular_y,
                        &cmd_vel->angular_z
                    );


                    if (items_parsed == 6) {
                        Process_CmdVel(cmd_vel);

                    } else {
                        const char *error_msg = "Error: Invalid cmd_vel format.\n";
                        CDC_Transmit_FS((uint8_t *)error_msg, strlen(error_msg));
                    }
                }
            }
            buffer_index = 0;
        } else {
            cdc_buffer[buffer_index++] = received_char;
        }
    }
}
