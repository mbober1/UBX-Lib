#include "ubx_lib.hpp"

const uint8_t UBX_HEADER[] = {0xB5, 0x62};
#define UBX_GPS_NAV_POSLLH_CLASS  (0x01)
#define UBX_GPS_NAV_POSLLH_ID     (0x02)
#define NO_FRAME_FOUND            (-1)

enum ubx_class_id
{
  UBX_CLASS_NAV = 0x01,
  UBX_CLASS_RXM = 0x02,
  UBX_CLASS_INF = 0x04,
  UBX_CLASS_ACK = 0x05,
  UBX_CLASS_CFG = 0x06,
  UBX_CLASS_UPD = 0x09,
  UBX_CLASS_MON = 0x0A,
  UBX_CLASS_AID = 0x0B,
  UBX_CLASS_TIM = 0x0D,
  UBX_CLASS_ESF = 0x10,
  UBX_CLASS_MGA = 0x13,
  UBX_CLASS_LOG = 0x21,
  UBX_CLASS_SEC = 0x27,
  UBX_CLASS_HNR = 0x28,
};

union ubx_frame_header
{
  struct
  {
    uint8_t sync[2];
    uint8_t frame_class;
    uint8_t frame_id;
    uint8_t length[2];
  };

  uint8_t buffer[6];
};

union ubx_frame
{
  struct
  {
    union ubx_frame_header header;
    uint8_t payload[sizeof(struct posllh)];
    uint8_t checksum[2];
  };

  uint8_t buffer[36];
};
static_assert(sizeof(union ubx_frame) == 36, "Invalid frame size");



/*
* @brief Find frame in buffer
* @param buffer - buffer with GPS data
* @param buffer_size - size of buffer
* @param offset - offset from which to start searching
* @return offset of frame in buffer if frame was found, NO_FRAME_FOUND otherwise
*/
size_t find_frame(uint8_t *buffer, size_t buffer_size, size_t offset)
{
  for (size_t i = offset; i < buffer_size - 1; i++)
  {
    if (buffer[i] == UBX_HEADER[0] && buffer[i + 1] == UBX_HEADER[1])
    {
      return i;
    }
  }

  return NO_FRAME_FOUND;
}

bool checksum(uint8_t *buffer, size_t buffer_size)
{
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  for (size_t i = 2; i < buffer_size - 2; i++)
  {
    ck_a += buffer[i];
    ck_b += ck_a;
  }

  return ck_a == buffer[buffer_size - 2] && ck_b == buffer[buffer_size - 1];
}


/*
* @brief Parse GPS data from buffer
* @param buffer - buffer with GPS data
* @param buffer_size - size of buffer
* @param posllh_data - pointer to struct where GPS data will be stored
* @return true if GPS data was parsed successfully, false otherwise
*/
bool gps_parse(uint8_t *buffer, size_t buffer_size, struct posllh* posllh_data)
{
  size_t frame_offset = 0;
  frame_offset = find_frame(buffer, buffer_size, frame_offset);

  while (NO_FRAME_FOUND != frame_offset)
  {
    static union ubx_frame_header frame_header;
    
    if (buffer_size - frame_offset < sizeof(frame_header.buffer))
    {
      return false;
    }

    memcpy(frame_header.buffer, buffer + frame_offset, sizeof(frame_header));

    if (frame_header.frame_class == UBX_GPS_NAV_POSLLH_CLASS && frame_header.frame_id == UBX_GPS_NAV_POSLLH_ID)
    {
      static union ubx_frame frame;
      memcpy(frame.buffer, buffer + frame_offset, sizeof(frame));

      if (true == checksum(frame.buffer, sizeof(frame.buffer)))
      {
        memcpy(posllh_data, frame.payload, sizeof(struct posllh));
        return true;
      }
    }

    frame_offset = find_frame(buffer, buffer_size, frame_offset + 1);
  }

  return false;
}