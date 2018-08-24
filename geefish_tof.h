#pragma once

#include <stdint.h>


typedef struct tof_point
{
	float x, y, z;
} tof_point_t;

typedef struct tof_cloud
{
	unsigned short width, height;
	tof_point_t *points;
	float *distance;
	float *amplitude;
	float *gray;
	float temperature;
} tof_cloud_t;

typedef void (*frame_cb)(const tof_cloud_t *);

int tof_set_cb(frame_cb handler);

int tof_open();

int tof_close();

int tof_set_expose(uint16_t value);

int tof_get_expose(uint16_t &value);

int tof_save_calib(const char *name);

int tof_calibrate(float ref, const char *name);

const char *tof_version();

int tof_video_to_mass_storage();
