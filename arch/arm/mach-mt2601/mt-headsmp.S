#include <linux/linkage.h>
#include <linux/init.h>


ENTRY(mt_secondary_startup)
    b secondary_startup
END(mt_secondary_startup)

