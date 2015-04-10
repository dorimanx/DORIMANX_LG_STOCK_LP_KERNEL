/*
 *  snfc_available_poll.c
 *
 */

/*
 *  Inclued header files
 */

#include "snfc_available_poll.h"
#include <linux/delay.h>
#include <mach/board_lge.h>
//#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
//#include <linux/irq.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/errno.h>

extern struct snfc_gp snfc_gpios;
extern int koto_state;
extern struct snfc_i2c_dev snfc_i2c_dev;

wait_queue_head_t snfcpollavailwq;
int snfcpollavail_sig;

void snfc_avali_poll_rfs_status(void)
{
    snfcpollavail_sig = 1;
    wake_up(&snfcpollavailwq);
}
EXPORT_SYMBOL(snfc_avali_poll_rfs_status);
void snfc_avali_poll_cen_status(int cen_status)
{
    snfcpollavail_sig = 1;
    wake_up(&snfcpollavailwq);
}
void snfc_avali_poll_felica_status(void)
{
    snfcpollavail_sig = 1;
    wake_up(&snfcpollavailwq);
}
int snfc_hvdd_wait_rfs_low(void)
{
    int rc = 0;

    do{
        snfcpollavail_sig = 0;
        if( snfc_gpio_read(snfc_gpios.gpio_rfs) == GPIO_HIGH_VALUE ){
            snfc_gpio_write(snfc_gpios.gpio_uicc_con ,GPIO_HIGH_VALUE );
        } else {
            snfc_gpio_write(snfc_gpios.gpio_uicc_con ,GPIO_LOW_VALUE );
            break;
        }
        rc = wait_event_interruptible( snfcpollavailwq, snfcpollavail_sig);
    }while(rc != -ERESTARTSYS);

    return rc;
}
/*
 * Description:
 * Input:
 * Output:
 */
static int __snfc_avail_poll_get_rfs_status(void)
{
    int return_val;

    return_val = snfc_gpio_read(snfc_gpios.gpio_rfs);               //Rev.B

    return return_val;
}

/*
 * Description:
 * Input:
 * Output:
 */
static int __snfc_avail_poll_get_cen_status(void)
{
    int rc = 0;
    unsigned char read_buf = 0x00;
    int cen_status;

    rc = snfc_i2c_read(0x02, &read_buf, 1, snfc_i2c_dev.client);
    if(rc)
    {
        SNFC_DEBUG_MSG("[__snfc_avail_poll_get_cen_status] snfc_i2c_read : %d \n",rc);
        return -1;
    }
    // check bit 7(locken)
    if(read_buf&0x01)
    {
        SNFC_DEBUG_MSG_LOW("[__snfc_avail_poll_get_cen_status] CEN = High (UNLOCK) \n");
        cen_status = GPIO_HIGH_VALUE;
    }
    else
    {
        SNFC_DEBUG_MSG_LOW("[__snfc_avail_poll_get_cen_status] CEN = Low (LOCK) \n");
        cen_status = GPIO_LOW_VALUE;
    }

    return cen_status;
}

/*
 * Description: it may need to check rfs, cen gpio is initialized
 * Input:
 * Output:
 */
static int snfc_avail_poll_open (struct inode *inode, struct file *fp)
{
    int rc = 0;
    int rfs_status = -1, cen_status = -1, uart_status = -1;

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_open - start \n");

    rfs_status = __snfc_avail_poll_get_rfs_status();
    cen_status = __snfc_avail_poll_get_cen_status();
    uart_status = __snfc_uart_control_get_uart_status();

    if(rfs_status == GPIO_HIGH_VALUE && cen_status == GPIO_HIGH_VALUE && uart_status != UART_STATUS_FOR_FELICA){
        snfcpollavail_sig = 1;
    } else {
        snfcpollavail_sig = 0;
    }

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_open - end \n");

    return rc;
}

/*
 * Description:
 * Input:
 * Output:
 */
static int snfc_avail_poll_release (struct inode *inode, struct file *fp)
{
    int rc = 0;

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_release - start \n");

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_release - end \n");

    return rc;
}

/*
 * Description:
 * Input:
 * Output:
 */
static ssize_t snfc_avail_poll_read(struct file *pf, char *pbuf, size_t size, loff_t *pos)
{
    //unsigned char read_buf = 0x00;
    int loop = 1;
    int available_poll = -1;
    int rc = -1;
    int rfs_status = -1, cen_status = -1, uart_status = -1;

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_read - start \n");

    /* Check parameters */
    if( NULL == pf || NULL == pbuf /*|| size == NULL*/ /*|| pos == NULL*/)
    {
        SNFC_DEBUG_MSG("[snfc_avail_poll] file error pf = %p, pbuf = %p, size = %d, pos = %d\n", pf, pbuf, (int)size,(int)pos);
        return -1;
    }

    do{
               rc = 0;
         cen_status = __snfc_avail_poll_get_cen_status();
               uart_status = __snfc_uart_control_get_uart_status();
         rfs_status = __snfc_avail_poll_get_rfs_status();

         SNFC_DEBUG_MSG_MIDDLE("[snfc_avail_poll] current rfs_status : %d, cen_status : %d, uart_status : %d \n",rfs_status, cen_status, uart_status);
         if(rfs_status == GPIO_HIGH_VALUE && cen_status == GPIO_HIGH_VALUE && uart_status != UART_STATUS_FOR_FELICA)
            {
                available_poll = 1;
                rc = copy_to_user(pbuf, &available_poll, size);

             SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_read stop, polling available!! \n");
                return 1;
            } else {
                snfcpollavail_sig = 0;
            }
            if(koto_state == 10)
                break;

         if( rfs_status != GPIO_HIGH_VALUE /*&& cen_status == GPIO_HIGH_VALUE && uart_status != UART_STATUS_FOR_FELICA*/){
                mdelay(1);
            } else {
                rc = wait_event_interruptible( snfcpollavailwq, snfcpollavail_sig );
            }

            if ( rc == -ERESTARTSYS)
                return -1;
     }while(loop == 1 && rc >= 0);


    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_read - end \n");

    return 1;
}

static struct file_operations snfc_avail_poll_fops = {
    .owner    = THIS_MODULE,
    .open     = snfc_avail_poll_open,
    .read     = snfc_avail_poll_read,
    .release  = snfc_avail_poll_release,
};

static struct miscdevice snfc_avail_poll_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "snfc_available_poll",
    .fops   = &snfc_avail_poll_fops,
};

int snfc_avail_poll_probe(struct device_node *np)
{
    int rc = 0;

    /* Register the device file */
    rc = misc_register(&snfc_avail_poll_device);
    if (rc < 0)
    {
        SNFC_DEBUG_MSG("[snfc_avail_poll] FAIL!! can not register snfc_avail_poll \n");
        return rc;
    }

    init_waitqueue_head(&snfcpollavailwq);
//    init_waitqueue_head(&snfcpollavailcenwq);
//    init_waitqueue_head(&snfcpollavailfelicawq);
    return rc;
}
void snfc_avail_poll_remove(void)
{
    /* deregister the device file */
    misc_deregister(&snfc_avail_poll_device);

}

#if 0
static int snfc_avail_poll_init(void)
{
    int rc;

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_init - start \n");

    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_init - end \n");

    return 0;
}

static void snfc_avail_poll_exit(void)
{
    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_exit - start \n");


    SNFC_DEBUG_MSG_LOW("[snfc_avail_poll] snfc_avail_poll_exit - end \n");
}

module_init(snfc_avail_poll_init);
module_exit(snfc_avail_poll_exit);
#endif

MODULE_LICENSE("Dual BSD/GPL");

