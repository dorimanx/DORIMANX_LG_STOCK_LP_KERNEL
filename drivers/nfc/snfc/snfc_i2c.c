/*
* snfc_i2c.c
*
*/

/*
 *  INCLUDE FILES FOR MODULE
 */
#include <linux/syscalls.h>
#include <linux/i2c-dev.h>

#include "snfc_i2c.h"

/*
 *   INTERNAL DEFINITION
 */

//#define I2C_SNFC_SLAVE_ADDRESS      0x56 << 1
#define I2C_STATUS_LOOP_MAX_CNT     0xFFFFFF

/*
 *   INTERNAL VARIABLE
 */

//static int fd = -1;

_e_snfc_i2c_status g_i2c_status = I2C_STATUS_NO_USE;

void __snfc_i2c_control_set_status(_e_snfc_i2c_status i2c_status)
{
#ifdef FEATURE_DEBUG_MIDDLE
    _e_snfc_i2c_status current_status = g_i2c_status;
#endif
    g_i2c_status = i2c_status;

#ifdef FEATURE_DEBUG_MIDDLE
    if(current_status > I2C_STATUS_READY && g_i2c_status > I2C_STATUS_READY)
        SNFC_DEBUG_MSG("[snfc_i2c_control] i2c status %d, but other device tries to use i2c \n", current_status);
#endif

    SNFC_DEBUG_MSG_MIDDLE("[snfc_i2c_control] i2c status %d -> %d\n", current_status, g_i2c_status );

    return;
}

EXPORT_SYMBOL(__snfc_i2c_control_set_status);
/*
* Description :
* Input :
* Output :
*/
_e_snfc_i2c_status __snfc_i2c_control_get_status(void)
{
    return g_i2c_status;
}
EXPORT_SYMBOL(__snfc_i2c_control_get_status);

/*
 *   FUNCTION DEFINITION
 */

/*
* Description :
* Input :
* Output :
*/
int snfc_i2c_read(unsigned char reg, unsigned char *buf, size_t count,  struct i2c_client *client)
{
//    struct i2c_client *client;
//    struct snfc_i2c_dev snfc_i2c_dev;

    ssize_t rc = 0;
//    int retry;

 //   snfc_i2c_dev = (snfc_i2c_dev*) private_data->client;

    __snfc_i2c_control_set_status(I2C_STATUS_FOR_NFC);

    SNFC_DEBUG_MSG_LOW("[snfc_i2c] snfc_i2c_read\n");

//    client = (struct snfc_i2c_dev*) private_data->client;
    //client->addr = I2C_SNFC_SLAVE_ADDRESS;
    //client->flags &= ~I2C_CLIENT_TEN;

    rc = i2c_master_send(client, &reg, 1);
    if (rc < 0)
    {
        SNFC_DEBUG_MSG("[snfc_i2c_read] ERROR - send address : %d \n",rc);
        __snfc_i2c_control_set_status(I2C_STATUS_READY);
        return rc;
    }

    rc = i2c_master_recv(client, buf, count);
    SNFC_DEBUG_MSG_LOW("[snfc_i2c_read] read data : 0x%02x \n",*buf);
    if (rc < 0)
    {
        SNFC_DEBUG_MSG("[snfc_i2c_read] ERROR - i2c_master_recv : %d \n",rc);
        __snfc_i2c_control_set_status(I2C_STATUS_READY);
        return rc;
    }

    __snfc_i2c_control_set_status(I2C_STATUS_READY);

    return 0;
}



/*
* Description :
* Input :
* Output :
*/
int snfc_i2c_write(unsigned char reg, unsigned char *buf, size_t count,  struct i2c_client *client)
{
//    struct i2c_client *client;
    //struct snfc_i2c_dev snfc_i2c_dev;

    ssize_t rc = 0;
    ssize_t err_ret = 0;
    unsigned char write_buf[2];

    //snfc_i2c_dev = (snfc_i2c_dev*) private_data;

    SNFC_DEBUG_MSG_LOW("[snfc_i2c] snfc_i2c_write\n");

//    client = (struct snfc_i2c_dev*) private_data->client;

    /* set register  */
    memset(write_buf,0x00,2*sizeof(unsigned char));
    write_buf[0] = reg;
    write_buf[1] = *buf;

    SNFC_DEBUG_MSG_LOW("[snfc_i2c] write_buf[0][1] : 0x%02x 0x%02x \n",write_buf[0],write_buf[1]);

    /* write data */
    rc = i2c_master_send(client, write_buf, count+1);
    if (rc < 0)
    {
        SNFC_DEBUG_MSG("[snfc_i2c] ERROR - i2c_master_send : %d \n",rc);
        __snfc_i2c_control_set_status(I2C_STATUS_READY);
        err_ret = 1;
        goto write_exit;
    }

write_exit:
    if(err_ret)
        return rc;
    else
        return 0;
}

