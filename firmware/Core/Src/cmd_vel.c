/*
 * cmd_vel.c - Command Velocity Interface
 *
 * Purpose: Parse incoming velocity commands and pass to robot control
 */

#include "cmd_vel.h"
#include "robot_control.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include <stdbool.h>

#define BUFFER_SIZE 256

/* Static buffer for USB CDC */
static char cdc_buffer[BUFFER_SIZE];
static uint16_t buffer_index = 0;

/* Last command timestamp */
static uint32_t last_cmd_time = 0;

/* Safety limits */
#define MAX_LINEAR_VEL  1.0f   // m/s
#define MAX_ANGULAR_VEL 2.0f   // rad/s

/**
 * @brief Process a parsed cmd_vel command
 * @param cmd_vel: Pointer to CmdVel structure
 */
void Process_CmdVel(CmdVel *cmd_vel)
{

    // Update timestamp
    last_cmd_time = HAL_GetTick();

    // Safety check and limit velocities
    float vx = cmd_vel->linear_x;
    float vy = cmd_vel->linear_y;
    float w = cmd_vel->angular_z;

    // Limit linear velocity
    float linear_mag = sqrtf(vx * vx + vy * vy);
    if (linear_mag > MAX_LINEAR_VEL) {
        float scale = MAX_LINEAR_VEL / linear_mag;
        vx *= scale;
        vy *= scale;
    }

    // Limit angular velocity
    if (w > MAX_ANGULAR_VEL) w = MAX_ANGULAR_VEL;
    if (w < -MAX_ANGULAR_VEL) w = -MAX_ANGULAR_VEL;

    // Apply to robot
    Robot_ProcessCmdVel(vx, vy, w);

#ifdef DEBUG_CMD_VEL
    char msg[128];
    snprintf(msg, sizeof(msg), "cmd_vel: vx=%.2f vy=%.2f w=%.2f\r\n", vx, vy, w);
    CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
#endif
}

/**
 * @brief Receive and parse USB CDC data
 * @param buf: Received data buffer
 * @param len: Length of data
 * @param cmd_vel: Pointer to CmdVel structure
 */
void Receive_CmdVel(uint8_t* buf, uint32_t len, CmdVel *cmd_vel)
{
    for (uint32_t i = 0; i < len; i++) {
        char c = buf[i];

        if (c == '\n' || c == '\r' || buffer_index >= BUFFER_SIZE - 1) {
            cdc_buffer[buffer_index] = '\0';

            if (buffer_index > 0) {
                bool parsed = false;
                int items;

                // Format 1: Full ROS twist
                if (strstr(cdc_buffer, "linear_x") && strstr(cdc_buffer, "angular_z")) {
                    items = sscanf(cdc_buffer,
                        "{linear_x:%f,linear_y:%f,linear_z:%f,angular_x:%f,angular_y:%f,angular_z:%f}",
                        &cmd_vel->linear_x, &cmd_vel->linear_y, &cmd_vel->linear_z,
                        &cmd_vel->angular_x, &cmd_vel->angular_y, &cmd_vel->angular_z);
                    parsed = (items == 6);
                }
                // Format 2: Simple
                else if (strstr(cdc_buffer, "vx") && strstr(cdc_buffer, "w")) {
                    items = sscanf(cdc_buffer, "{vx:%f,vy:%f,w:%f}",
                        &cmd_vel->linear_x, &cmd_vel->linear_y, &cmd_vel->angular_z);
                    parsed = (items == 3);
                    cmd_vel->linear_z = 0.0f;
                    cmd_vel->angular_x = 0.0f;
                    cmd_vel->angular_y = 0.0f;
                }
                // Format 3: CSV
                else {
                    items = sscanf(cdc_buffer, "%f,%f,%f",
                        &cmd_vel->linear_x, &cmd_vel->linear_y, &cmd_vel->angular_z);
                    parsed = (items == 3);
                    cmd_vel->linear_z = 0.0f;
                    cmd_vel->angular_x = 0.0f;
                    cmd_vel->angular_y = 0.0f;
                }

                if (parsed) {
                    Process_CmdVel(cmd_vel);
                } else {
                    const char *err = "Error: Invalid cmd_vel format\r\n";
                    CDC_Transmit_FS((uint8_t *)err, strlen(err));
                }
            }
            buffer_index = 0;
        } else {
            cdc_buffer[buffer_index++] = c;
        }
    }
}

/**
 * @brief Check for command timeout
 */
void Check_CmdVel_Timeout(void)
{
    const uint32_t TIMEOUT_MS = 500;
    uint32_t current = HAL_GetTick();

    if (last_cmd_time > 0 && (current - last_cmd_time) > TIMEOUT_MS) {
        Motor_StopAll();

        static bool warned = false;
        if (!warned) {
            const char *msg = "cmd_vel timeout - motors stopped\r\n";
            CDC_Transmit_FS((uint8_t *)msg, strlen(msg));
            warned = true;
        }
    }
}