/*
  my_plugin.c - plugin init for simulator with ATCI keepout plugin
*/

#include <stdio.h>
#include "grbl/hal.h"

extern void mrbear_init(void);

void my_plugin_init(void)
{
    fprintf(stderr, "[my_plugin_init] calling mrbear_init()\n");
    mrbear_init();
}
