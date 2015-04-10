/*
 *  felica_cal.c
 *
 */

/*
 *    INCLUDE FILES FOR MODULE
 */
#include <linux/module.h>
#include <linux/kernel.h>


#include "felica_cal.h"


/*
 *    INTERNAL DEFINITION
 */

#define FELICA_I2C_SLAVE_ADDRESS  0x56
#define FELICA_I2C_REG_ADDRSS_01  0x01
#define FELICA_I2C_REG_ADDRSS_02  0x02

/*
 *    FUNCTION DEFINITION
 */

/*
* Description :
* Input :
* Output :
*/
static int felica_cal_open (struct inode *inode, struct file *fp)
{
  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_open\n");

  return 0;
}

/*
* Description :
* Input :
* Output :
*/
static int felica_cal_release (struct inode *inode, struct file *fp)
{
  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_release \n");

  return 0;
}

/*
* Description :
* Input :
* Output :
*/
static ssize_t felica_cal_read(struct file *fp, char *buf, size_t count, loff_t *pos)
{
  unsigned char read_buf = 0x00;
  int rc = -1;

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_read - start \n");

/* Check error */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR fp \n");
    return -1;
  }

  if(NULL == buf)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR buf \n");
    return -1;
  }

  if(1 != count)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR count \n");
    return -1;
  }

  if(NULL == pos)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR file \n");
    return -1;
  }



  rc = felica_i2c_read(FELICA_I2C_REG_ADDRSS_01, &read_buf, 1);
  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] felica_i2c_read : %d \n",rc);
    return -1;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal : 0x%02x \n",read_buf);

  rc = copy_to_user(buf, &read_buf, count);
  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR - copy_from_user \n");
    return -1;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_read - end \n");

  return 1;
}

/*
* Description :
* Input :
* Output :
*/
static ssize_t felica_cal_write(struct file *fp, const char *buf, size_t count, loff_t *pos)
{
  unsigned char write_buf = 0x00, read_buf = 0x00;
  int rc = -1;

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_write - start \n");

/* Check error */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR file \n");
    return -1;
  }

  if(NULL == buf)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR buf \n");
    return -1;
  }

  if(1 != count)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL]ERROR count \n");
    return -1;
  }

  if(NULL == pos)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR file \n");
    return -1;
  }

  /* copy from user data */
  rc = copy_from_user(&write_buf, buf, count);
  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR - copy_from_user \n");
    return -1;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] write_buf : 0x%02x \n",write_buf);

  /* read register value before writing new value */
  rc = felica_i2c_read(FELICA_I2C_REG_ADDRSS_01, &read_buf, 1);
  udelay(50);

  /* write new value */
  write_buf = write_buf | 0x80;
  rc = felica_i2c_write(FELICA_I2C_REG_ADDRSS_01, &write_buf, 1);
  mdelay(2);

  /* read register value after writing new value */
  rc = felica_i2c_read(FELICA_I2C_REG_ADDRSS_01, &read_buf, 1);
  udelay(50);

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_write - end \n");

  return 1;
}

/*
 *    STRUCT DEFINITION
 */


static struct file_operations felica_cal_fops =
{
  .owner    = THIS_MODULE,
  .open    = felica_cal_open,
  .read    = felica_cal_read,
  .write    = felica_cal_write,
  .release  = felica_cal_release,
};

static struct miscdevice felica_cal_device =
{
  .minor = 242,
  .name = FELICA_CAL_NAME,
  .fops = &felica_cal_fops
};

/*
* Description :
* Input :
* Output :
*/
static int felica_cal_init(void)
{
  int rc = -1;

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_init - start \n");

  /* register the device file */
  rc = misc_register(&felica_cal_device);
  if (rc < 0)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA_CAL] ERROR - can not register felica_cal \n");
    return rc;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_init - end \n");

  return 0;
}
/*
* Description :
* Input :
* Output :
*/
static void felica_cal_exit(void)
{
  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_exit - start \n");

  /* deregister the device file */
  misc_deregister(&felica_cal_device);

  FELICA_DEBUG_MSG_LOW("[FELICA_CAL] felica_cal_exit - end \n");
}

module_init(felica_cal_init);
module_exit(felica_cal_exit);

MODULE_LICENSE("Dual BSD/GPL");

