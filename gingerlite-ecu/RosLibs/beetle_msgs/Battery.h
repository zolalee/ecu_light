#ifndef _ROS_beetle_msgs_Battery_h
#define _ROS_beetle_msgs_Battery_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace beetle_msgs
{

  class Battery : public ros::Msg
  {
    public:
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _current_type;
      _current_type current;
      typedef float _soc_type;
      _soc_type soc;

    Battery():
      voltage(0),
      current(0),
      soc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.real = this->current;
      *(outbuffer + offset + 0) = (u_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_soc;
      u_soc.real = this->soc;
      *(outbuffer + offset + 0) = (u_soc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_soc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_soc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_soc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->soc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_current;
      u_current.base = 0;
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current = u_current.real;
      offset += sizeof(this->current);
      union {
        float real;
        uint32_t base;
      } u_soc;
      u_soc.base = 0;
      u_soc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_soc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_soc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_soc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->soc = u_soc.real;
      offset += sizeof(this->soc);
     return offset;
    }

    const char * getType(){ return "beetle_msgs/Battery"; };
    const char * getMD5(){ return "5c19f9aebb3c9ac9860c145a1ed1ea3f"; };

  };

}
#endif
