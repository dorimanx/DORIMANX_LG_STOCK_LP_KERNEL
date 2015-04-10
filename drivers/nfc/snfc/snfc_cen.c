/*
 *  felica_cen.c
 *
 */

/*
 *    Include header files
 */
//#include <linux/module.h>
#include <linux/kernel.h>


#include "snfc_cen.h"

/*
 *    Internal definition
 */

 //#define FEATURE_DEBUG_LOW
#define SNFC_I2C_SLAVE_ADDRESS  0x56
#define SNFC_I2C_REG_ADDRSS_01  0x01
#define SNFC_I2C_REG_ADDRSS_02  0x02

struct snfc_i2c_dev snfc_i2c_dev;

static DEFINE_MUTEX(nfc_cen_mutex);

static int isopen = 0; // 0 : No open 1 : Open

extern void snfc_avali_poll_cen_status(int cen_status);
/*
 *    Function definition
 */
/*
int snfc_get_cen_status(int *cen_status)
{
    unsigned char read_buf = 0x00;
    int rc;

    rc = snfc_i2c_read(0x02, &read_buf, 1, snfc_i2c_dev.client);

}
*/
/*
* Description:
* Input:
* Output:
*/
static int snfc_cen_open (struct inode *inode, struct file *fp)
{
    if(1 == isopen)
    {
        SNFC_DEBUG_MSG_LOW("[snfc_cen] snfc_cen_open - already open \n");
        return 0;
    }
    else
    {
        SNFC_DEBUG_MSG_LOW("[snfc_cen] snfc_cen_open - start \n");
        isopen = 1;
    }
    return 0;
}


/*
 * Description:
 * Input:
 * Output:
 */
static int snfc_cen_release (struct inode *inode, struct file *fp)
{


    if(0 == isopen)
    {
        SNFC_DEBUG_MSG_LOW("[snfc_cen] snfc_cen_release - not open \n");

        return -1;
    }
    else
    {
        SNFC_DEBUG_MSG_LOW("[snfc_cen] snfc_cen_release - start \n");

        isopen = 0;
    }

    return 0;
}


/*
 * Description:
 * Input:
 * Output:
 */
static ssize_t snfc_cen_read(struct file *fp, char *buf, size_t count, loff_t *pos)
{
    //struct snfc_i2c_dev *snfc_i2c_dev = fp->private_data;

    unsigned char read_buf = 0x00;
    char snfc_cen = -1, rc = -1;

  SNFC_DEBUG_MSG_LOW("[snfc_cen] snfc_cen_read - start \n");

/* Check error */
  if(NULL == fp || NULL == buf || 1 != count || NULL == pos)
  {
    SNFC_DEBUG_MSG("[snfc_cen][read] parameter ERROR \n");
    return -1;
  }

  mutex_lock(&nfc_cen_mutex);
  rc = snfc_i2c_read(0x02, &read_buf, 1, snfc_i2c_dev.client);
  mutex_unlock(&nfc_cen_mutex);

  if(rc)
  {
    SNFC_DEBUG_MSG("[snfc_cen][read] snfc_i2c_read : %d \n",rc);
    return -1;
  }

  // check bit 7(locken)
  if(read_buf&0x01)  // unlock
  {
    SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][read] CEN = High (UNLOCK) \n");
    snfc_cen = (char)GPIO_HIGH_VALUE;
  }
  else  // lock
  {
    SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][read] CEN = Low (LOCK) \n");
    snfc_cen = (char)GPIO_LOW_VALUE;
  }

  rc = copy_to_user(buf, &snfc_cen, count);
  if(rc)
  {
    SNFC_DEBUG_MSG("[snfc_cen][read] ERROR - copy_from_user \n");
    return -1;
  }

  SNFC_DEBUG_MSG_LOW("[snfc_cen][read] snfc_cen_read - end \n");

  return 1;
}

/*
 * Description:
 * Input:
 * Output:
 */
static ssize_t snfc_cen_write(struct file *fp, const char *buf, size_t count, loff_t *pos)
{

    //struct snfc_i2c_dev *snfc_i2c_dev = fp->private_data;

    unsigned char write_buf = 0x00/*, read_buf = 0x00*/;
    int rc = -1;

    SNFC_DEBUG_MSG_LOW("[snfc_cen][write] snfc_cen_write - start \n");

    /* Check error */
    if(NULL == fp || NULL == buf ||  1 != count || NULL == pos)
    {
        SNFC_DEBUG_MSG("[snfc_cen][write] ERROR \n");
        return -1;
    }

    /* copy from user data */
    rc = copy_from_user(&write_buf, buf, count);
    if(rc)
    {
        SNFC_DEBUG_MSG("[snfc_cen][write] ERROR - copy_from_user \n");
        return -1;
    }

    SNFC_DEBUG_MSG_LOW("[snfc_cen][write] copy_from_user(%d) \n",*buf);

    /* check user data */
    if(*buf == 1)
    {
        SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][write] CEN = High (UNLOCK) \n");

        write_buf = 0x81; // set unlock
        mutex_lock(&nfc_cen_mutex);
        rc = snfc_i2c_write(0x02, &write_buf, 1, snfc_i2c_dev.client);
        mutex_unlock(&nfc_cen_mutex);
        mdelay(2);
        snfc_avali_poll_cen_status(GPIO_HIGH_VALUE);
    }
    else if(*buf == 0)
    {
        SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][write] CEN = Low (LOCK) \n");

        write_buf = 0x80; // set lock
        mutex_lock(&nfc_cen_mutex);
        rc = snfc_i2c_write(0x02, &write_buf, 1, snfc_i2c_dev.client);
        mutex_unlock(&nfc_cen_mutex);
        mdelay(2);
        snfc_avali_poll_cen_status(GPIO_LOW_VALUE);
    }
    else if(*buf == 2)
    {
        write_buf = 0x80; // set lock
        mutex_lock(&nfc_cen_mutex);
        rc = snfc_i2c_write(0x02, &write_buf, 1, snfc_i2c_dev.client);
        mutex_unlock(&nfc_cen_mutex);
        SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][write] CEN = Low & Hgh(LOCK) \n");

        mdelay(1);

        write_buf = 0x81; // set unlock
        mutex_lock(&nfc_cen_mutex);
        rc = snfc_i2c_write(0x02, &write_buf, 1, snfc_i2c_dev.client);
        mutex_unlock(&nfc_cen_mutex);
        SNFC_DEBUG_MSG_MIDDLE("[snfc_cen][write] CEN = Low & Hgh(UNLOCK) \n");
    }

    SNFC_DEBUG_MSG_LOW("[snfc_cen][write] snfc_cen_write - end \n");

    return 1;
}

static struct file_operations snfc_cen_fops =
{
  .owner    = THIS_MODULE,
  .open    = snfc_cen_open,
  .read    = snfc_cen_read,
  .write    = snfc_cen_write,
  .release  = snfc_cen_release,
};

static struct miscdevice snfc_cen_device =
{
  .minor = 122,
  .name = "snfc_cen",
  .fops = &snfc_cen_fops
};

static int snfc_cen_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int rc = -1;

    SNFC_DEBUG_MSG("[snfc_cen] snfc_cen_probe start\n");

    snfc_i2c_dev.client = client;

    /* register the device file */
    rc = misc_register(&snfc_cen_device);
    if (rc < 0)
    {
      SNFC_DEBUG_MSG("[snfc_cen] FAIL!! can not register snfc_cen \n");
      return rc;
    }

    snfc_i2c_dev.client->addr = I2C_SNFC_SLAVE_ADDRESS;
    snfc_i2c_dev.client->flags &= ~I2C_CLIENT_TEN;
    i2c_set_clientdata(client, &snfc_i2c_dev);

    SNFC_DEBUG_MSG("[snfc_cen] snfc_cen_probe end\n");

    return 0;
}

static int snfc_cen_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id snfc_i2c_id[] = {
    {"snfc_cen", 0},
    {}
};

static struct of_device_id snfc_match_table[] = {
    { .compatible = "sony,snfc_cen",},
    { },
};

static struct i2c_driver snfc_i2c_driver = {
    .id_table = snfc_i2c_id,
    .probe = snfc_cen_probe,
    .remove = snfc_cen_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "snfc_cen",
        .of_match_table = snfc_match_table,
    },
};
static int __init snfc_cen_dev_init(void)
{   int rc;

    SNFC_DEBUG_MSG("[snfc_cen] snfc_cen_dev_init - start \n");

    rc = i2c_add_driver(&snfc_i2c_driver);
    if (rc < 0)
    {
        SNFC_DEBUG_MSG("[snfc_cen] FAIL!! can not register snfc_cen \n");
        return rc;
    }

    SNFC_DEBUG_MSG("[snfc_cen] snfc_cen_dev_init - end \n");
    return 0;
}
module_init(snfc_cen_dev_init);

static void __exit snfc_cen_dev_exit(void)
{
    i2c_del_driver(&snfc_i2c_driver);
}
module_exit(snfc_cen_dev_exit);

MODULE_DEVICE_TABLE(i2c, snfc_i2c_id);
MODULE_DESCRIPTION("NFC sony nfc i2c driver");
MODULE_LICENSE("GPL");

