#ifndef BASE64MIXIN_H
#define BASE64MIXIN_H

#include "FujiDeviceMixin.h"

class Base64Mixin : public FujiDeviceMixin
{
protected:
    void encode_input(uint16_t len);
    void encode_compute();
    void encode_length();
    void encode_output(uint16_t len);
    void decode_input(uint16_t len);
    void decode_compute();
    void decode_length();
    void decode_output(uint16_t len);

 public:
    bool processCommand(PROCESS_COMMAND_TYPE command) override;
};

#endif /* BASE64MIXIN_H */
