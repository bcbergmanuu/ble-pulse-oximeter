#include <stdio.h>
#include <zephyr/kernel.h>
#include "max30102.h"

typedef struct { 
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */    
	ppg_item_t * item;
} fifo_item_t;

enum powerevents {
	evt_powerup = 1,
	evt_powerdown = 2,
};