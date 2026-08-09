#ifndef __DCSBIOS_I2CPROTOCOL_H
#define __DCSBIOS_I2CPROTOCOL_H

#include <stdint.h>
#include "I2cCommand.h"

namespace DcsBios {

void initI2cProtocolDefaults();
void setI2cGaugeCount(uint8_t count);

} // namespace DcsBios

#endif
