/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#ifndef _HF_SENSOR_TYPE_H_
#define _HF_SENSOR_TYPE_H_

enum {
	SENSOR_TYPE_INVALID = 0,
	/* follow google default sensor type */
	SENSOR_TYPE_ACCELEROMETER = 1,
	SENSOR_TYPE_MAGNETIC_FIELD,
	SENSOR_TYPE_ORIENTATION,
	SENSOR_TYPE_GYROSCOPE,
	SENSOR_TYPE_LIGHT,
	SENSOR_TYPE_PRESSURE,
	SENSOR_TYPE_TEMPERATURE,
	SENSOR_TYPE_PROXIMITY,
	SENSOR_TYPE_GRAVITY,
	SENSOR_TYPE_LINEAR_ACCELERATION,
	SENSOR_TYPE_ROTATION_VECTOR,
	SENSOR_TYPE_RELATIVE_HUMIDITY,
	SENSOR_TYPE_AMBIENT_TEMPERATURE,
	SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
	SENSOR_TYPE_GAME_ROTATION_VECTOR,
	SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
	SENSOR_TYPE_SIGNIFICANT_MOTION,
	SENSOR_TYPE_STEP_DETECTOR,
	SENSOR_TYPE_STEP_COUNTER,
	SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR,
	SENSOR_TYPE_HEART_RATE,
	SENSOR_TYPE_TILT_DETECTOR,
	SENSOR_TYPE_WAKE_GESTURE,
	SENSOR_TYPE_GLANCE_GESTURE,
	SENSOR_TYPE_PICK_UP_GESTURE,
	SENSOR_TYPE_WRIST_TILT_GESTURE,
	SENSOR_TYPE_DEVICE_ORIENTATION,
	SENSOR_TYPE_POSE_6DOF,
	SENSOR_TYPE_STATIONARY_DETECT,
	SENSOR_TYPE_MOTION_DETECT,
	SENSOR_TYPE_HEART_BEAT,
	SENSOR_TYPE_DYNAMIC_SENSOR_META,
	SENSOR_TYPE_ADDITIONAL_INFO,
	SENSOR_TYPE_LOW_LATENCY_OFFBODY_DETECT,
	SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED,

	/* follow mtk add sensor type */
	SENSOR_TYPE_PEDOMETER					= 55,
	SENSOR_TYPE_IN_POCKET					= 56,
	SENSOR_TYPE_ACTIVITY					= 57,
	SENSOR_TYPE_PDR							= 58,
	SENSOR_TYPE_FREEFALL					= 59,
	SENSOR_TYPE_FLAT						= 60,
	SENSOR_TYPE_FACE_DOWN					= 61,
	SENSOR_TYPE_SHAKE						= 62,
	SENSOR_TYPE_BRINGTOSEE					= 63,
	SENSOR_TYPE_ANSWER_CALL					= 64,
	SENSOR_TYPE_GEOFENCE					= 65,
	SENSOR_TYPE_FLOOR_COUNTER				= 66,
	SENSOR_TYPE_EKG							= 67,
	SENSOR_TYPE_PPG1						= 68,
	SENSOR_TYPE_PPG2						= 69,
	SENSOR_TYPE_RGBW						= 70,
	SENSOR_TYPE_GYRO_TEMPERATURE			= 71,
	SENSOR_TYPE_SAR							= 72,
	SENSOR_TYPE_OIS							= 73,
	SENSOR_TYPE_GYRO_SECONDARY				= 74,
	SENSOR_TYPE_SAR_SECONDARY				= 75,
	SENSOR_TYPE_ALS_CCT						= 76,

	/* virtual sensor type */
	COMM_SENSOR_TYPE_RAISEUP_DETECT			= 90,
	COMM_SENSOR_TYPE_PUTDOWN_DETECT			= 91,
	COMM_SENSOR_TYPE_VIVOMOTION_DETECT		= 92,
	COMM_SENSOR_TYPE_ANGLE_JUDGE			= 93,
	COMM_SENSOR_TYPE_ANGLE_DIRECTION 		= 94,
	COMM_SENSOR_TYPE_AMD					= 95,
	COMM_SENSOR_TYPE_SMARTPROX_DETECT		= 96,
	COMM_SENSOR_TYPE_WINDOW_ORIENTATION		= 97,
	COMM_SENSOR_TYPE_AMBIENT_LIGHT_SCENE	= 98,
	COMM_SENSOR_TYPE_PROXIMITY_UNDERDISPLAY	= 99,
	COMM_SENSOR_TYPE_DROPDOWN_DETECT		= 100,
	COMM_SENSOR_TYPE_PROXIMITY_C			= 101,
	COMM_SENSOR_TYPE_EDGEREJECTION_DETECT	= 102,
	COMM_SENSOR_TYPE_CAR_NAVI_DETECT		= 103,
	COMM_SENSOR_TYPE_DROP_DEPTH				= 104,
	COMM_SENSOR_TYPE_LCM_ESDCHECK			= 105,
	COMM_SENSOR_TYPE_VLOGGER				= 106,
	COMM_SENSOR_TYPE_AIRPLANE_IDENTIFY		= 107,
	COMM_SENSOR_TYPE_GAME_GESTURE			= 108,
	COMM_SENSOR_TYPE_PROX_GESTURE			= 109,
	SENSOR_TYPE_SENSOR_MAX					= 110,
};

enum {
	ID_OFFSET = 1,

	/* follow google default sensor type */
	ID_ACCELEROMETER = 0,
	ID_MAGNETIC_FIELD,
	ID_ORIENTATION,
	ID_GYROSCOPE,
	ID_LIGHT,
	ID_PRESSURE,
	ID_TEMPERATURE,
	ID_PROXIMITY,
	ID_GRAVITY,
	ID_LINEAR_ACCELERATION,
	ID_ROTATION_VECTOR,
	ID_RELATIVE_HUMIDITY,
	ID_AMBIENT_TEMPERATURE,
	ID_MAGNETIC_FIELD_UNCALIBRATED,
	ID_GAME_ROTATION_VECTOR,
	ID_GYROSCOPE_UNCALIBRATED,
	ID_SIGNIFICANT_MOTION,
	ID_STEP_DETECTOR,
	ID_STEP_COUNTER,
	ID_GEOMAGNETIC_ROTATION_VECTOR,
	ID_HEART_RATE,
	ID_TILT_DETECTOR,
	ID_WAKE_GESTURE,
	ID_GLANCE_GESTURE,
	ID_PICK_UP_GESTURE,
	ID_WRIST_TILT_GESTURE,
	ID_DEVICE_ORIENTATION,
	ID_POSE_6DOF,
	ID_STATIONARY_DETECT,
	ID_MOTION_DETECT,
	ID_HEART_BEAT,
	ID_DYNAMIC_SENSOR_META,
	ID_ADDITIONAL_INFO,
	ID_LOW_LATENCY_OFFBODY_DETECT,
	ID_ACCELEROMETER_UNCALIBRATED,

	/* follow mtk add sensor type */
	ID_PEDOMETER			= SENSOR_TYPE_PEDOMETER - ID_OFFSET,
	ID_IN_POCKET			= SENSOR_TYPE_IN_POCKET - ID_OFFSET,
	ID_ACTIVITY				= SENSOR_TYPE_ACTIVITY - ID_OFFSET,
	ID_PDR					= SENSOR_TYPE_PDR - ID_OFFSET,
	ID_FREEFALL				= SENSOR_TYPE_FREEFALL - ID_OFFSET,
	ID_FLAT					= SENSOR_TYPE_FLAT - ID_OFFSET,
	ID_FACE_DOWN			= SENSOR_TYPE_FACE_DOWN - ID_OFFSET,
	ID_SHAKE				= SENSOR_TYPE_SHAKE - ID_OFFSET,
	ID_BRINGTOSEE			= SENSOR_TYPE_BRINGTOSEE - ID_OFFSET,
	ID_ANSWER_CALL			= SENSOR_TYPE_ANSWER_CALL - ID_OFFSET,
	ID_GEOFENCE				= SENSOR_TYPE_GEOFENCE - ID_OFFSET,
	ID_FLOOR_COUNTER		= SENSOR_TYPE_FLOOR_COUNTER - ID_OFFSET,
	ID_EKG					= SENSOR_TYPE_EKG - ID_OFFSET,
	ID_PPG1					= SENSOR_TYPE_PPG1 - ID_OFFSET,
	ID_PPG2					= SENSOR_TYPE_PPG2 - ID_OFFSET,
	ID_RGBW					= SENSOR_TYPE_RGBW - ID_OFFSET,
	ID_GYRO_TEMPERATURE		= SENSOR_TYPE_GYRO_TEMPERATURE - ID_OFFSET,
	ID_SAR					= SENSOR_TYPE_SAR - ID_OFFSET,
	ID_OIS					= SENSOR_TYPE_OIS - ID_OFFSET,
	ID_GYRO_SECONDARY		= SENSOR_TYPE_GYRO_SECONDARY - ID_OFFSET,
	ID_SAR_SECONDARY		= SENSOR_TYPE_SAR_SECONDARY - ID_OFFSET,
	ID_ALS_CCT				= SENSOR_TYPE_ALS_CCT - ID_OFFSET,

	/* virtual sensor type */
	ID_RAISEUP_DETECT			= COMM_SENSOR_TYPE_RAISEUP_DETECT - ID_OFFSET,
	ID_PUTDOWN_DETECT			= COMM_SENSOR_TYPE_PUTDOWN_DETECT - ID_OFFSET,
	ID_VIVOMOTION_DETECT		= COMM_SENSOR_TYPE_VIVOMOTION_DETECT - ID_OFFSET,
	ID_ANGLE_JUDGE				= COMM_SENSOR_TYPE_ANGLE_JUDGE - ID_OFFSET,
	ID_ANGLE_DIRECTION			= COMM_SENSOR_TYPE_ANGLE_DIRECTION - ID_OFFSET,
	ID_AMD						= COMM_SENSOR_TYPE_AMD - ID_OFFSET,
	ID_SMARTPROX_DETECT			= COMM_SENSOR_TYPE_SMARTPROX_DETECT - ID_OFFSET,
	ID_WINDOW_ORIENTATION		= COMM_SENSOR_TYPE_WINDOW_ORIENTATION - ID_OFFSET,
	ID_AMBIENT_LIGHT_SCENE		= COMM_SENSOR_TYPE_AMBIENT_LIGHT_SCENE - ID_OFFSET,
	ID_PROXIMITY_UNDERDISPLAY	= COMM_SENSOR_TYPE_PROXIMITY_UNDERDISPLAY - ID_OFFSET,
	ID_DROPDOWN_DETECT			= COMM_SENSOR_TYPE_DROPDOWN_DETECT - ID_OFFSET,
	ID_PROXIMITY_C				= COMM_SENSOR_TYPE_PROXIMITY_C - ID_OFFSET,
	ID_EDGEREJECTION_DETECT		= COMM_SENSOR_TYPE_EDGEREJECTION_DETECT - ID_OFFSET,
	ID_CAR_NAVI_DETECT			= COMM_SENSOR_TYPE_CAR_NAVI_DETECT - ID_OFFSET,
	ID_DROP_DEPTH				= COMM_SENSOR_TYPE_DROP_DEPTH - ID_OFFSET,
	ID_LCM_ESDCHECK				= COMM_SENSOR_TYPE_LCM_ESDCHECK - ID_OFFSET,
	ID_VLOGGER					= COMM_SENSOR_TYPE_VLOGGER - ID_OFFSET,
	ID_GAME_GESTURE				= COMM_SENSOR_TYPE_GAME_GESTURE - ID_OFFSET,
	ID_PROX_GESTURE				= COMM_SENSOR_TYPE_PROX_GESTURE - ID_OFFSET,
	ID_SENSOR_MAX				= SENSOR_TYPE_SENSOR_MAX - ID_OFFSET,
};

enum {
	SENSOR_ACCURANCY_UNRELIALE,
	SENSOR_ACCURANCY_LOW,
	SENSOR_ACCURANCY_MEDIUM,
	SENSOR_ACCURANCY_HIGH,
};

#endif
