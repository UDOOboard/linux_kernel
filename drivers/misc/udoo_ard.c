#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#define DRIVER_NAME     "udoo_ard"

#define AUTH_TOKEN 0x5A5A
// #define MAX_MSEC_SINCE_LAST_IRQ 1000*1000*1000
#define MAX_MSEC_SINCE_LAST_IRQ 400 // 
#define GRAY_TIME_BETWEEN_RESET 10000 // In this time we can't accept new erase/reset code 

/* pinctrl state */
#define PINCTRL_DEFAULT      "default"
#define PINCTRL_ALTERNATE    "udoo_ard_alt"

static struct platform_device_id udoo_ard_devtype[] = {
        {
                /* keep it for coldfire */
                .name = DRIVER_NAME,
                .driver_data = 0,
        }, {
                /* sentinel */
        }
};
MODULE_DEVICE_TABLE(platform, fec_devtype);

static const struct of_device_id udoo_ard_dt_ids[] = {
        { .compatible = "udoo,imx6q-udoo-ard", .data = &udoo_ard_devtype[0], },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fec_dt_ids);

static struct workqueue_struct *erase_reset_wq;

typedef struct {
    struct work_struct erase_reset_work;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *pins_alternate;
    int    step;
    int    cmdcode;
    int    erase_reset_lock;
    int    gpio_bossac_clk;
    int    gpio_bossac_dat;
    int    gpio_ard_erase;
    int    gpio_ard_reset;
    int    gpio_4_6;
    int    gpio_4_7;
    unsigned long    last_int_time_in_ns;
    unsigned long    last_int_time_in_sec;
} erase_reset_work_t;

erase_reset_work_t *work;

static void erase_reset_wq_function( struct work_struct *work2)
{
    int ret;

    pinctrl_select_state(work->pinctrl, work->pins_alternate);
    printk("Disable serial on UDOO Quad. \n");

    ret = gpio_request(work->gpio_4_6, "SERIAL INPUT");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR MX6QSDL_SECO_KEYCOL0__GPIO_4_6 IRQ\n");
    } else {
            gpio_direction_input(work->gpio_4_6);
    }

    ret = gpio_request(work->gpio_4_7, "SERIAL INPUT");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR MX6QSDL_SECO_KEYROW0__GPIO_4_7 IRQ\n");
    } else {
            gpio_direction_input(work->gpio_4_7);
    }

    gpio_direction_input(work->gpio_ard_reset);
//    gpio_set_value(work->gpio_ard_reset, 1);
    msleep(1);

    gpio_direction_output(work->gpio_ard_erase, 1);
    msleep(300);
    gpio_direction_input(work->gpio_ard_erase);

    msleep(10);
    gpio_set_value(work->gpio_ard_reset, 0);

    msleep(80);
    gpio_set_value(work->gpio_ard_reset, 1);

    printk("UDOO ERASE and RESET on Sam3x EXECUTED. [%d]\n", work->erase_reset_lock);
    msleep(GRAY_TIME_BETWEEN_RESET);
    work->erase_reset_lock = 0;

    return;
}


static irqreturn_t udoo_bossac_req(int irq, void *dev_id)
{
    int     retval, auth_bit, expected_bit;
    int     cmdcode = 0x0;
    int     msec_since_last_irq;
    u64        nowsec;
    unsigned long  rem_nsec;

    auth_bit = 0;
    if (gpio_get_value(work->gpio_bossac_dat) != 0x0)
      auth_bit = 1;
// printk("ARDUINO IRQ RECEIVED %d !!\n\n", auth_bit);

    erase_reset_work_t *erase_reset_work = (erase_reset_work_t *)work;

    nowsec = local_clock();
    rem_nsec = do_div(nowsec, 1000000000) ;
    msec_since_last_irq = (((unsigned long)nowsec * 1000) + rem_nsec/1000000 ) - (((unsigned long)erase_reset_work->last_int_time_in_sec * 1000) + erase_reset_work->last_int_time_in_ns/1000000);

//printk("msec_since_last_irq = %li\n", msec_since_last_irq);
    if (msec_since_last_irq > MAX_MSEC_SINCE_LAST_IRQ) {
        erase_reset_work->step = 0;
  printk("Warn! UDOO KEY RESET AUTHENTICATION RESETTING TIMEOUT !\n");
    }

// printk("STEP %d -> 0x%d \n", erase_reset_work->step, auth_bit);
    erase_reset_work->last_int_time_in_ns = rem_nsec;
    erase_reset_work->last_int_time_in_sec = nowsec;

    if ( erase_reset_work->step < 16 ) {  // Authenticating received token bit.
        expected_bit = (( AUTH_TOKEN >> erase_reset_work->step ) & 0x01 );
        if ( auth_bit == expected_bit ) {
            erase_reset_work->step = erase_reset_work->step + 1;
        } else {
            erase_reset_work->step = 0;
        }
    } else { // Passed all authentication step. Receiving command code.
        erase_reset_work->cmdcode = erase_reset_work->cmdcode | (auth_bit << (erase_reset_work->step - 16));
        erase_reset_work->step = erase_reset_work->step + 1;
    }

//printk("erase_reset_work->erase_reset_lock = %d \n", erase_reset_work->erase_reset_lock);
    if ( erase_reset_work->step == 21 ) {  // Passed authentication and code acquiring step.

 printk("RECEIVED CODE = 0x%04x \n", erase_reset_work->cmdcode);
        if (erase_reset_work->cmdcode == 0xF) {
	    if (erase_reset_work->erase_reset_lock == 0) {
            	erase_reset_work->erase_reset_lock = 1;
            	retval = queue_work( erase_reset_wq, (struct work_struct *)work );
	    } else {
		printk("Erase and reset operation already in progress. Do nothing.\n");
	    } 
            // To do: check retval error code.
        } else {
              pinctrl_select_state(work->pinctrl, work->pins_default);
              printk("Re-enable serial on UDOO quad. \n");
        }
        erase_reset_work->step = 0;
        erase_reset_work->cmdcode = 0;
    }

    return IRQ_HANDLED;
}

static int gpio_setup(void)
{
    int ret;

    ret = gpio_request(work->gpio_bossac_clk, "BOSSA_CLK");
    if (ret) {
            printk(KERN_ERR "request BOSSA_CLK IRQ\n");
            return -1;
    } else {
            gpio_direction_input(work->gpio_bossac_clk);
    }

    ret = gpio_request(work->gpio_bossac_dat, "BOSSA_DAT");
    if (ret) {
            printk(KERN_ERR "request BOSSA_DAT IRQ\n");
            return -1;
    } else {
            gpio_direction_input(work->gpio_bossac_dat);
    }

    ret = gpio_request(work->gpio_ard_erase, "BOSSAC");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR ARDUINO ERASE\n");
            return -1;
    } else {
            gpio_direction_input(work->gpio_ard_erase);
    }

    ret = gpio_request(work->gpio_ard_reset, "BOSSAC");
    if (ret) {
            printk(KERN_ERR "request GPIO FOR ARDUINO RESET\n");
            return -1;
    } else {
            gpio_direction_output(work->gpio_ard_reset, 1);
    }

    return 0;
}


static int udoo_ard_probe(struct platform_device *pdev)
{
    int retval;

    struct platform_device *bdev;
    bdev = kzalloc(sizeof(*bdev), GFP_KERNEL);

    struct device_node *np = pdev->dev.of_node;

    if (!np)
            return -ENODEV;

    work = (erase_reset_work_t *)kmalloc(sizeof(erase_reset_work_t), GFP_KERNEL);
    if (work) {
	    work->gpio_ard_reset = of_get_named_gpio(np, "bossac-reset-gpio", 0);
	    work->gpio_ard_erase = of_get_named_gpio(np, "bossac-erase-gpio", 0);
	    work->gpio_bossac_clk = of_get_named_gpio(np, "bossac-clk-gpio", 0);
	    work->gpio_bossac_dat = of_get_named_gpio(np, "bossac-dat-gpio", 0);
	    work->gpio_4_6 = of_get_named_gpio(np, "bossac-4_6-gpio", 0);
	    work->gpio_4_7 = of_get_named_gpio(np, "bossac-4_7-gpio", 0);
	    work->pinctrl = devm_pinctrl_get(&pdev->dev);
            work->pins_default = pinctrl_lookup_state(work->pinctrl, PINCTRL_DEFAULT);
            work->pins_alternate = pinctrl_lookup_state(work->pinctrl, PINCTRL_ALTERNATE);
    } else {
	    printk("[bossac] Failed to allocate data structure.\n");
	    return -ENOMEM;
    }

    pinctrl_select_state(work->pinctrl, work->pins_alternate);
//    pinctrl_select_state(work->pinctrl, work->pins_default);
    gpio_setup();

    printk("[bossac] Registering IRQ %d for BOSSAC Arduino erase/reset operation\n", gpio_to_irq(work->gpio_bossac_clk));
    retval = request_irq(gpio_to_irq(work->gpio_bossac_clk), udoo_bossac_req, IRQF_TRIGGER_FALLING, "UDOO", bdev);

    erase_reset_wq = create_workqueue("erase_reset_queue");
    if (erase_reset_wq) {

        /* Queue some work (item 1) */
        if (work) {
                INIT_WORK( (struct work_struct *)work, erase_reset_wq_function );
                work->step = 1;
                work->cmdcode = 0;
                work->last_int_time_in_ns = 0;
                work->last_int_time_in_sec = 0;
                work->erase_reset_lock = 0;
            //  retval = queue_work( erase_reset_wq, (struct work_struct *)work );
        }
    }
    return  0;
}

static void udoo_ard_remove(struct platform_device *pdev)
{
    printk("Unloading UDOO ard driver.\n");
    free_irq(gpio_to_irq(work->gpio_bossac_clk), NULL);

    gpio_free(work->gpio_ard_reset);
    gpio_free(work->gpio_ard_erase);
    gpio_free(work->gpio_bossac_clk);
    gpio_free(work->gpio_bossac_dat);
    return;
}

static struct platform_driver udoo_ard_driver = {
        .driver = {
                .name   = DRIVER_NAME,
                .owner  = THIS_MODULE,
                .of_match_table = udoo_ard_dt_ids,
        },
        .id_table = udoo_ard_devtype,
        .probe  = udoo_ard_probe,
        .remove = udoo_ard_remove,
};

module_platform_driver(udoo_ard_driver);

MODULE_ALIAS("platform:"DRIVER_NAME);
MODULE_LICENSE("GPL");

