/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *    __  ___    ________
 *   / / / / |  / / ____/
 *  / / / /| | / / / __
 * / /_/ / | |/ / /_/ /
 * \____/  |___/\____/
 *
 * Robotat control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/** Handles incoming CPX messages and routing messages inside
 *  the Crazyflie firmware to specific Kalman and CRTP functions.
 */

#define DEBUG_MODULE "ROBOTAT"

#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "cpx_internal_router.h"
#include "cpx.h"
#include "system.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "robotat.h"

static CPXPacket_t cpxRx;
static bool isInit = false;

uint8_t* lastCpxData;
char* str;
double pose[7];


static float extPosStdDev = 0.01;
static float extQuatStdDev = 4.5e-3;

static poseMeasurement_t ext_pose;
static positionMeasurement_t ext_pos;
static setpoint_t setpoint;

// ============================================================================
// PROTOTIPOS DE FUNCIONES
// ============================================================================
static void robotatTask(void*);
STATIC_MEM_TASK_ALLOC(robotatTask, ROBOTAT_TASK_STACKSIZE);
static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw);

// ============================================================================
// INIT
// ============================================================================
void robotatInit() {
  STATIC_MEM_TASK_CREATE(robotatTask, robotatTask, ROBOTAT_TASK_NAME, NULL, ROBOTAT_TASK_PRI);
  isInit = true;
}

// ============================================================================
// TASK
// ============================================================================
static void robotatTask(void* parameters) {
  systemWaitStart();

  // Entrar en modo manual
  setHoverSetpoint(&setpoint, 0.0, 0.0, 0.0, 0.0);
  commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);

  ext_pos.x = 0;
  ext_pos.y = 0;
  ext_pos.z = 0;
  ext_pos.source = 0;
  ext_pos.stdDev = extPosStdDev;


  ext_pose.x = 0.0;
  ext_pose.y = 0.0;
  ext_pose.z = 0.0;
  ext_pose.quat.x = 0.0;
  ext_pose.quat.y = 0.0;
  ext_pose.quat.z = 0.0;
  ext_pose.quat.w = 0.0;
  ext_pose.stdDevPos = extPosStdDev;
  ext_pose.stdDevQuat = extQuatStdDev;

  while (true) {

    cpxInternalRouterReceiveRobotat(&cpxRx);
    memcpy((uint8_t*)pose, cpxRx.data, 7*sizeof(double));

    ext_pos.x = pose[0];
    ext_pos.y = pose[2];
    ext_pos.z = pose[1];
    estimatorEnqueuePosition(&ext_pos);

    setHoverSetpoint(&setpoint, 1.0, 0.5, 1.0, 0.0);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
  }
}

// ============================================================================
// FUNCIONES
// ============================================================================
static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x= x;
  setpoint->position.y = y;
  setpoint->velocity.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;

  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
}

// ============================================================================
// LOG
// ============================================================================
LOG_GROUP_START(robotat)
/**
 * @brief Position in X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &ext_pos.x)

/**
 * @brief Position in Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &ext_pos.y)

/**
 * @brief Position in Z [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &ext_pos.z)

LOG_GROUP_STOP(robotat)