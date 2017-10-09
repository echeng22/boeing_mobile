#ifndef _ROS_SERVICE_ME495Srv_h
#define _ROS_SERVICE_ME495Srv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace me495_hw1
{

static const char ME495SRV[] = "me495_hw1/ME495Srv";

  class ME495SrvRequest : public ros::Msg
  {
    public:
      uint32_t input;

    ME495SrvRequest():
      input(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->input >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->input =  ((uint32_t) (*(inbuffer + offset)));
      this->input |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->input |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->input |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->input);
     return offset;
    }

    const char * getType(){ return ME495SRV; };
    const char * getMD5(){ return "50c8b86da7a26ace6f8ea1f64b0752e7"; };

  };

  class ME495SrvResponse : public ros::Msg
  {
    public:
      uint8_t output;

    ME495SrvResponse():
      output(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->output >> (8 * 0)) & 0xFF;
      offset += sizeof(this->output);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->output =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->output);
     return offset;
    }

    const char * getType(){ return ME495SRV; };
    const char * getMD5(){ return "a8b26e0bb5c5fc4aee37137f79a26d19"; };

  };

  class ME495Srv {
    public:
    typedef ME495SrvRequest Request;
    typedef ME495SrvResponse Response;
  };

}
#endif
