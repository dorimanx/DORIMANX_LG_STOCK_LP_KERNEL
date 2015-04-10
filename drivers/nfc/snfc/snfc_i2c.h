/*
* snfc_i2c.h
*
*/

#ifndef __SNFC_I2C_H__
#define __SNFC_I2C_H__

/*
 *  INCLUDE FILES FOR MODULE
 */
#include <linux/list.h>
#include <linux/i2c.h>

#include "snfc_common.h"
/*
 *  DEFINE
 */
#define I2C_SNFC_SLAVE_ADDRESS     0x56 >> 1

typedef enum _e_snfc_i2c_status {
    I2C_STATUS_NO_USE = 0,
    I2C_STATUS_READY,
    I2C_STATUS_FOR_FELICA,
    I2C_STATUS_FOR_NFC,
    I2C_STATUS_NONE,
} _e_snfc_i2c_status;

struct snfc_i2c_dev {
    struct i2c_client *client;
};

/*
 *  FUNCTION PROTOTYPE
 */
int snfc_i2c_write(unsigned char reg, unsigned char *buf, size_t count,  struct i2c_client *client);
int snfc_i2c_read(unsigned char reg, unsigned char *buf, size_t count,  struct i2c_client *client);
#endif
