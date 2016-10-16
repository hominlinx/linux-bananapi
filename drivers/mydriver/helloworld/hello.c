
#include <linux/kernel.h>
#include <linux/module.h>

static int __init hello_init(void)
{
    printk(KERN_INFO "------------->>Hello World Enter!\n");
    return 0;
}

static void __exit hello_exit(void)
{
    printk(KERN_INFO "Hello World Exit!/n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("Yuanguang");
MODULE_DESCRIPTION("A Simple Hello World Module");
MODULE_LICENSE("GPL");

