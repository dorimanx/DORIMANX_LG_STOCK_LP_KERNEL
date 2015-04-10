/*
 *  felica.c
 *
 */

/*
 *    INCLUDE FILES FOR MODULE
 */
#include <linux/mutex.h>
#include <linux/syscalls.h>
#include "felica.h"
#include "felica_uart.h"

#include "felica_test.h"

/*
 *  DEFINE
 */

enum{
UART_STATUS_NOT_OPEN= 0,
UART_STATUS_RX,
UART_STATUS_RX_TX,
};

/* Definition for transmit and receive buffer */
#define RECEIVE_BUFFER_MAX_SIZE 4096 /* FN requirement for felica device driver 1.7 */
#define TRANSMIT_BUFFER_MAX_SIZE 4096

char receive_buf[RECEIVE_BUFFER_MAX_SIZE + 4];
char transmit_buf[TRANSMIT_BUFFER_MAX_SIZE + 4];

/*
 *   INTERNAL VARIABLE
 */
static int uart_status = UART_STATUS_NOT_OPEN;

static DEFINE_MUTEX(felica_mutex);

/*
 *    FUNCTION PROTOTYPE
 */

/*
 *   FUNCTION DEFINITION
 */

/*
* Description :
* Input :
* Output :
*/
static int felica_open (struct inode *inode, struct file *fp)
{
  int rc = 0;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_open() is called \n");

  /* Check input parameters */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - fp is NULL \n");
    return -1;
  }

  /* FileInputStream and FileOutPutStream open felica
     Only one case has to be excuted */

  if(UART_STATUS_NOT_OPEN == uart_status)
  {
    uart_status = UART_STATUS_RX;
  }
  else if(UART_STATUS_RX == uart_status)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] /dev/felica RX is already openned. \n");

    uart_status = UART_STATUS_RX_TX;

    return 0;
  }
  else
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] /dev/felica RX/TX are already openned. \n");

    return 0;
  }

  rc = felica_uart_open();

  //mdelay(100);

  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - open_hs_uart \n");
    uart_status = UART_STATUS_NOT_OPEN;
    unlock_felica_wake_lock();
    return rc;
  }

  lock_felica_wake_lock();

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_open - end \n");

#ifdef FELICA_FN_DEVICE_TEST
  FELICA_DEBUG_MSG_LOW("[FELICA] felica_open - result_open(%d) \n",result_open_uart);
  return result_open_uart;
#else
  return 0;
#endif

}

/*
* Description :
* Input :
* Output :
*/

static ssize_t felica_read(struct file *fp, char *buf, size_t count, loff_t *pos)
{
  int rc = 0;
  int readcount = 0;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_read - start \n");

  /* Check input parameters */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - fp is NULL \n");
    return -1;
  }

  if(NULL == buf)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - buf is NULL \n");

    return -1;
  }

  if(count > RECEIVE_BUFFER_MAX_SIZE)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - count(%d) \n",count);
    return -1;
  }

  if(NULL == pos)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - pos is NULL \n");

    return -1;
  }

  memset(receive_buf, 0, sizeof(receive_buf));

  /* Test log
  {
    int remainingcount = 0;
    int rcTest = 0;

    rcTest = felica_uart_ioctrl(&remainingcount);
    FELICA_DEBUG_MSG_LOW("[FELICA] before remainedcount : %d \n",remainingcount);
    if (rcTest) {
      FELICA_DEBUG_MSG_LOW("[FELICA] ERROR - felica_uart_ioctrl \n");
      return rcTest;
    }

    if(4095 == remainedcount)
    {
      int i = 0;
      int testreadcount = 0;

      char *ptr = NULL;

      testreadcount = felica_uart_read(receive_buf,264);

      ptr = receive_buf;

      if(NULL != ptr)
      {
        FELICA_DEBUG_MSG_LOW("===== TEST READ DATA =====\n");
        for(i=0; i<testreadcount; i++)
        {
          FELICA_DEBUG_MSG_LOW(" %02x", *ptr++);
          if(0 == (i+1)%10)
          {
            FELICA_DEBUG_MSG_LOW("\n");
          }
        }
        FELICA_DEBUG_MSG_LOW("\n");
      }
    }

  }*/


  /* Copy UART receive data to receive buffer */
  mutex_lock(&felica_mutex);
  readcount = felica_uart_read(receive_buf,count);
  mutex_unlock(&felica_mutex);

  if(0 >= readcount)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - No data in data buffer \n");

    return 0;
  }

  //mdelay(5);

  /* Test log
  {
    int remainingcount = 0;
    int rcTest = 0;

    rcTest = felica_uart_ioctrl(&remainingcount);
    FELICA_DEBUG_MSG_LOW("[FELICA] remaining count : %d \n",remainingcount);
    if (rcTest) {
      FELICA_DEBUG_MSG_LOW("[FELICA] ERROR - felica_uart_ioctrl \n");
      return rcTest;
    }
  }
  */

/* Display low data for debugging */
#ifdef RXTX_LOG_ENABLE
  {
    int i = 0;
    char *ptr = NULL;

    ptr = receive_buf;
    if(NULL != ptr)
    {
      FELICA_DEBUG_MSG_LOW("===== READ FELICA RESPONSE ===== \n");
	  FELICA_DEBUG_MSG_LOW("FELICA - ");
      for(i=0; i<count; i++)
      {
        FELICA_DEBUG_MSG_LOW(" %02x", *ptr++);
        if(0 == (i+1)%10)
        {
          FELICA_DEBUG_MSG_LOW("\n");
		  FELICA_DEBUG_MSG_LOW("FELICA - ");
        }
      }
      FELICA_DEBUG_MSG_LOW("\n");
    }
  }
#endif
  /* Copy receive buffer to user memory */
  rc = copy_to_user(buf, receive_buf, count);

  if (rc) {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - copy_to_user \n");

    return rc;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_read - end \n");

#ifdef FELICA_FN_DEVICE_TEST
  if(result_read_uart != -1)
    result_read_uart = readcount;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_read - result_read_uart(%d) \n",result_read_uart);

  return result_read_uart;
#else
  return readcount;
#endif
}

/*
* Description :
* Input :
* Output :
*/
static ssize_t felica_write(struct file *fp, const char *buf, size_t count, loff_t *f_pos)
{
  int rc = 0;
  int writecount = 0;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_write - start \n");


  /* Check input parameters */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - fp is NULL \n");

    return -1;
  }

  if(NULL == buf)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - buf is NULL \n");

    return -1;
  }

  if(count > TRANSMIT_BUFFER_MAX_SIZE)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - count(%d) \n",count);

    return -1;
  }

  /* Clear transmit buffer before using */
  memset(transmit_buf, 0, sizeof(transmit_buf));

  /* Copy user memory to kernel memory */
  rc = copy_from_user(transmit_buf, buf, count);
  if (rc) {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - copy_to_user \n");

    return rc;
  }

/* Display low data for debugging */
 #ifdef RXTX_LOG_ENABLE
 {
    int i = 0;
    char *ptr = NULL;

    ptr = transmit_buf;

    if(NULL != ptr)
    {
      FELICA_DEBUG_MSG_LOW("===== WRITE FELICA COMMAND =====\n");
      for(i=0; i<count; i++)
      {
        FELICA_DEBUG_MSG_LOW(" %02x", *ptr++);
        if(0 == (i+1)%10)
        {
          FELICA_DEBUG_MSG_LOW("\n");
        }
      }
      FELICA_DEBUG_MSG_LOW("\n");
    }
  }
#endif

  /* Test log
  {
    int remainingcount = 0;
    int rcTest = 0;

    rcTest = felica_uart_ioctrl(&remainingcount);
    FELICA_DEBUG_MSG_LOW("[FELICA] remaining count : %d \n",remainingcount);
    if (rcTest) {
      FELICA_DEBUG_MSG_LOW("[FELICA] ERROR - felica_uart_ioctrl \n");
      return rcTest;
    }
  }
    */

  /* Send transmit data to UART transmit buffer */
  mutex_lock(&felica_mutex);
  writecount = felica_uart_write(transmit_buf,count);
  mutex_unlock(&felica_mutex);

  //mdelay(50);

  FELICA_DEBUG_MSG_MED("[FELICA] writecount : %d \n",writecount);


#ifdef FELICA_FN_DEVICE_TEST
  FELICA_DEBUG_MSG_LOW("[FELICA] felica_write - result_write_uart(%d) \n",result_write_uart);

  if(result_write_uart != -1) {
    if (writecount == 0)
      result_write_uart = -1;
    else
      result_write_uart = writecount;
	}
    return result_write_uart;
#else
  return writecount;
#endif
}

/*
* Description :
* Input :
* Output :
*/
static int felica_release (struct inode *inode, struct file *fp)
{
  int rc = 0;

  /* Check input parameters */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - fp is NULL \n");

    return -1;
  }
  /* FileInputStream and FileOutPutStream close felica
     Only one case has to be excuted */

  if(UART_STATUS_RX_TX == uart_status)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] /dev/felica RX is closed. \n");

    uart_status = UART_STATUS_RX;

    return 0;
  }
  else if(UART_STATUS_RX == uart_status)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] /dev/felica TX is closed. \n");

    uart_status = UART_STATUS_NOT_OPEN;
  }
  else
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] /dev/felica RX/TX are already closed. \n");

    return 0;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_release - start \n");

  rc = felica_uart_close();

  unlock_felica_wake_lock();

  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - open_hs_uart \n");

    return rc;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_release - end \n");

#ifdef FELICA_FN_DEVICE_TEST
  FELICA_DEBUG_MSG_LOW("[FELICA] felica_close - result_close(%d) \n",result_close_uart);
  return result_close_uart;
#else
  return 0;
#endif
}

/*
* Description : MFC calls this function using available method(int available()) of FileOutputStream class
* Input : None
* Output : Return the number of byte that can be read.
*/
static long felica_ioctl (struct file *fp, unsigned int cmd, unsigned long arg)
{
  int numofreceiveddata = 0;
  int rc = 0;
  int *uarg = (int *)arg;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_ioctl - start \n");

  /* Check input parameters */
  if(NULL == fp)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - fp is NULL \n");

    return -1;
  }

  if(IOCTL_FELICA_MAGIC != _IOC_TYPE(cmd))
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - IO cmd type(%d) \n",_IOC_TYPE(cmd));

    return -1;
  }

  if(IOCTL_FELICA_CMD_AVAILABLE != _IOC_NR(cmd))
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - IO cmd number(%d) \n",_IOC_NR(cmd));

    return -1;
  }

  if(0 != _IOC_SIZE(cmd))
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - IO cmd size(%d) \n",_IOC_SIZE(cmd));

    return -1;
  }

  mutex_lock(&felica_mutex);
  rc = felica_uart_ioctrl(&numofreceiveddata);
  mutex_unlock(&felica_mutex);

  if (rc) {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - felica_uart_ioctrl \n");

    return rc;
  }

   //mdelay(20);

  rc = copy_to_user(uarg, &numofreceiveddata, sizeof(int));
  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - open_hs_uart \n");

    return rc;
  }

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_ioctl - end \n");

#ifdef FELICA_FN_DEVICE_TEST
  return result_available_uart;
#else
  return rc;
#endif
}

/*
* Description :
* Input :
* Output :
*/
static int felica_fsync(struct file *fp, loff_t param1, loff_t param2, int datasync)
{
  FELICA_DEBUG_MSG_LOW("[FELICA] felica_fsync \n");


  // TODO: TRANSMIT DATA TO FELICA CHIP

  return 0;
}

static struct file_operations felica_fops =
{
  .owner    = THIS_MODULE,
  .open      = felica_open,
  .read      = felica_read,
  .write    = felica_write,
  .unlocked_ioctl    = felica_ioctl,
  .fsync    = felica_fsync,
  .release  = felica_release,
};

static struct miscdevice felica_device = {
  .minor = MINOR_NUM_FELICA,
  .name = FELICA_NAME,
  .fops = &felica_fops,
};

/*
* Description :
* Input :
* Output :
*/
static int felica_init(void)
{
  int rc = 0;

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_init - start \n");

  /* register the device file */
  rc = misc_register(&felica_device);
  if(rc)
  {
    FELICA_DEBUG_MSG_HIGH("[FELICA] ERROR - felica_init can not register(%d)\n", rc);

    return rc;
  }

  init_felica_wake_lock();

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_init - end \n");

  return 0;
}

/*
* Description :
* Input :
* Output :
*/
static void felica_exit(void)
{
  FELICA_DEBUG_MSG_LOW("[FELICA] felica_exit - start \n");

  destroy_felica_wake_lock();

  /* deregister the device file */
  misc_deregister(&felica_device);

  FELICA_DEBUG_MSG_LOW("[FELICA] felica_exit - end \n");
}

module_init(felica_init);
module_exit(felica_exit);

MODULE_LICENSE("Dual BSD/GPL");
