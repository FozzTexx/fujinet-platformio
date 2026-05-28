#ifndef HASHMIXIN_H
#define HASHMIXIN_H

#include "FujiDeviceMixin.h"
#include "hash.h"

class HashMixin : public FujiDeviceMixin
{
protected:
    Hash::Algorithm _algorithm = Hash::Algorithm::UNKNOWN;

    void hash_input(uint16_t len);
    void hash_compute(bool clear_data, Hash::Algorithm algo);
    void hash_length(bool as_hex);
    void hash_output(bool as_hex);
    void hash_clear();

public:
    bool processCommand(PROCESS_COMMAND_TYPE command) override;
};

#endif /* HASHMIXIN_H */
