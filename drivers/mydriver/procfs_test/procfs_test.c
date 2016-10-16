#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define LEN_MSG 64
#define NAME_NODE "procfstest"
#define NAME_DIR "procfs_test_dir"

static char *get_rw_buf( void )
{
    static char buf_msg[LEN_MSG+1] = ".........1.........2.........3.........4.........5\n" ;
    return buf_msg;
}


/*
 * include/linux/fs.h
 * ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
 */
static ssize_t procfs_test_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    char *buf_msg = get_rw_buf();
    int res;

    pr_info("read: %d bytes (ppos:%lld)", count, ppos);
    if (*ppos >= strlen(buf_msg)) {
        *ppos = 0;
        pr_info("EOF");
        return 0;
    }

    if (count > strlen(buf_msg) - *ppos) {
        count = strlen(buf_msg) - *ppos;
    }

    res = copy_to_user((void*)buf, buf_msg + *ppos, count);
    *ppos = count;
    pr_info("return %d bytes", count);
    return count;
}

/*
 * include/linux/fs.h
 * ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
 */
static ssize_t procfs_test_write( struct file *file, char *buf, size_t count, loff_t *ppos)
{
    char *msg_buf = get_rw_buf();
    int res;
    int len = count < LEN_MSG ? count : LEN_MSG; // 取小值
    pr_info("write : %d bytes", (int)count);
    res = copy_from_user( msg_buf, (void*)buf, len  );
    msg_buf[ len  ] = '\0';

    pr_info( "put %d bytes", len  );
    return len;
}

static const struct file_operations procfs_test_fops = {
    .owner = THIS_MODULE,
    .read = procfs_test_read,
    .write = procfs_test_write
};

static int __init procfs_test_init( void )
{
    struct proc_dir_entry *procfs_test_file;
    procfs_test_file = proc_create("procfstest", 0666, NULL, &procfs_test_fops);
    if (procfs_test_file == NULL) {
        pr_err("create /proc/%s failed!\n", NAME_NODE);
        return -ENOENT;
    }
    pr_info("create /proc/%s success!\n", NAME_NODE);
    return 0;
}

static void __exit procfs_test_exit( void )
{
    remove_proc_entry(NAME_NODE, NULL);
    pr_info("exit /proc/%s module.\n", NAME_NODE);
}

module_init(procfs_test_init);
module_exit(procfs_test_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("Yuanguang");
MODULE_DESCRIPTION("A Simple Procfs Module");
MODULE_LICENSE("GPL");


