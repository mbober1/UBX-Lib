#pragma once
#include <stdint.h>
#include <memory.h>
#include <stdbool.h>
#include <assert.h>

struct posllh
{
  uint32_t iTOW;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t hMSL;
  uint32_t hAcc;
  uint32_t vAcc;
};
static_assert(sizeof(struct posllh) == 28, "Invalid frame size");


bool gps_parse(uint8_t *buffer, size_t buffer_size, struct posllh* posllh_data);