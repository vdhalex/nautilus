/* 
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the 
 * United States National  Science Foundation and the Department of Energy.  
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national 
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2019, Peter Dinda <pdinda@northwestern.edu>
 * Copyright (c) 2019, The V3VEE Project  <http://www.v3vee.org> 
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Authors: Peter Dinda <pdinda@northwestern.edu>
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */


/*

  This is stub code for the CS 343 Driver Lab at Northwestern.

  This driver provides access to the legacy first parallel port (LPT1)
  and abstracts it using NK's chardev interface.

  https://en.wikipedia.org/wiki/Parallel_port

*/


#include <nautilus/nautilus.h>
#include <nautilus/irq.h>
#include <nautilus/dev.h>
#include <nautilus/chardev.h>
#include <nautilus/shell.h>


///////////////////////////////////////////////////////////////////
// Wrappers for debug and other output so that
// they can be enabled/disabled at compile time using kernel
// build configuration (Kconfig)
//

#ifndef NAUT_CONFIG_DEBUG_PARPORT
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...) 
#endif

#define ERROR(fmt, args...) ERROR_PRINT("parport: ERROR: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("parport: DEBUG: " fmt, ##args)
#define INFO(fmt, args...)  INFO_PRINT("parport: " fmt, ##args)


///////////////////////////////////////////////////////////////////
// Wrappers for locking the software state of a device
//

#define STATE_LOCK_CONF uint8_t _state_lock_flags
#define STATE_LOCK(state) _state_lock_flags = spin_lock_irq_save(&((state)->lock))
#define STATE_UNLOCK(state) spin_unlock_irq_restore(&(((state)->lock)), _state_lock_flags)


///////////////////////////////////////////////////////////////////
// The software state of a device
//
struct parport_state {
    struct nk_char_dev *dev;     // we are a character device (chardev)

    uint16_t base_port;          // we are doing I/O port I/O starting at this port
    
    uint8_t  irq;                // irq we are on

    spinlock_t lock;             // we have a lock

    enum {READY=0, BUSY} state;     
};


///////////////////////////////////////////////////////////////////
// Mapping of registers of the legacy first parallel port
// into the I/O address space of an x64 (also interrupt)
//
#define PARPORT0_BASE     0x378
#define PARPORT0_IRQ      7
#define DATA_PORT(base)   (base+0)     // read/write
#define STAT_PORT(base)   (base+1)     // read-only
#define CTRL_PORT(base)   (base+2)     // 


///////////////////////////////////////////////////////////////////
// Register layouts
//

// data register layout
// data written to the data register shows up on pins 2-9 of the connector
// data read from the data register is what's on pins 2-9 of the connector 
typedef uint8_t data_reg_t;   

// status register layout - this is read-only
typedef union _stat_reg_t {
    uint8_t   val;
    struct {
	uint_t res     :2;  // reserved
	uint_t irq     :1;
	uint_t err     :1;  // 0 => error (active low)
	uint_t sel_in  :1;  // 1 => attached device online
	uint_t pap_out :1;  // 1 => attached device out of paper
	uint_t ack     :1;  // 0 => attached device acknowledges (active low)
	uint_t busy    :1;  // 0 => attached device is busy (active low)
    } __attribute__((packed));
} __attribute__((packed)) stat_reg_t;


// control register layout - this is read/write
typedef union _ctrl_reg_t {
    uint8_t   val;
    struct {
	uint_t strobe  :1;  // alert attached device to data
	uint_t auto_lf :1;  // automatically add linefeeds to carriage returns
	uint_t init    :1;  // 0 => reinit attached device (active low)
	uint_t select  :1;  // 1 => select attached device
	uint_t irq_en  :1;  // enable interrupt on completion
	uint_t bidir   :1;  // 0 => output, 1 => input
	uint_t res     :2;  // reserved
    } __attribute__((packed));
} __attribute__((packed)) ctrl_reg_t;



static int get_characteristics(void *state, struct nk_char_dev_characteristics *c)
{
    memset(c,0,sizeof(*c));
    return 0;
}



static int read_write(void *state, uint8_t *data, int write)
{
    struct parport_state *s = (struct parport_state *)state;
    int rc=-1;
    ctrl_reg_t ctrl;
    stat_reg_t stat;

    DEBUG("doing %s of %c\n",write ? "write" : "read", *data);

    outb(CTRL_PORT(s->base_port),0);
    outb(DATA_PORT(s->base_port),*data);

    return 1;
    
    STATE_LOCK_CONF;

    // get exclusive control
    STATE_LOCK(s);

    // if not ready, we are toast
    if (s->state!=READY) {
	DEBUG("not ready\n");
	rc = 0;
	goto out;
    }

#if 0
    // set the port to output
    ctrl.val=0;
    ctrl.irq_en = 1; // interrupt when done
    ctrl.bidir =  write ? 0 : 1; // data flowing to attached device

    DEBUG("writing config %02x\n",ctrl.val);
    
    outb(CTRL_PORT(s->base_port),ctrl.val);
#endif
    
    // this should not wait long...
    while ( !(inb(STAT_PORT(s->base_port)) & 0x80)) {
	io_delay();
    }

    DEBUG("ready to write data\n");
    
    if (write) {
	// actually output the data
	DEBUG("port=%x\n",s->base_port);
	outb(DATA_PORT(s->base_port),*data);
	DEBUG("data written\n");

	// now do the strobe
	ctrl.val=inb(CTRL_PORT(s->base_port));
	ctrl.strobe = 1;
	outb(CTRL_PORT(s->base_port),ctrl.val);
	io_delay();
	ctrl.strobe = 0;
	outb(CTRL_PORT(s->base_port),ctrl.val);
	//s->state = BUSY;
	DEBUG("strobe complete\n");
	// this should not wait long...
	while ( !(inb(STAT_PORT(s->base_port)) & 0x80)) {
	    io_delay();
	}
	DEBUG("write complete\n");
    } else {
	// actually input the data
	*data = inb(DATA_PORT(s->base_port));
	DEBUG("data read (%c)\n",*data);
    }

    rc = 1; //success

 out:
    
    STATE_UNLOCK(s);

    return rc; // success

}

static int read(void *state, uint8_t *dest)
{
    return read_write(state,dest,0);
}

static int write(void *state, uint8_t *src)
{
    return read_write(state,src,1);
}



static int status(void *state)
{
    struct parport_state *s = (struct parport_state *)state;
    int rc;
    STATE_LOCK_CONF;
    

    STATE_LOCK(s);
    rc = s->state==READY;
    STATE_UNLOCK(s);

    if (rc) {
	return NK_CHARDEV_READABLE | NK_CHARDEV_WRITEABLE;
    } else {
	return 0;
    }
}
    
    
static int interrupt_handler (excp_entry_t * excp, excp_vec_t vec, void *state)
{
    struct parport_state *s = (struct parport_state *)state;

    DEBUG("interrupt - device %s!\n",s->dev->dev.name);

    s->state=READY;
    
    IRQ_HANDLER_END();
    
    return 0;
}


static int init(struct parport_state *s)
{
    DEBUG("no initialization - will only do trivial mode\n");
    return 0;
}


struct nk_char_dev_int interface =  {
    .get_characteristics = get_characteristics,
    .read = read,
    .write = write,
    .status = status,
};


static int bringup(uint16_t port, uint8_t irq, char *name)
{
    struct parport_state *s = malloc(sizeof(*s));
    
    if (!s) {
	ERROR("Failed to allocate state\n");
	return -1;
    }

    memset(s,0,sizeof(*s));

    spinlock_init(&s->lock);

    s->base_port = port;
    s->irq = irq;

    // now register our interrupt handler, though we will probably
    // never use it in this code at all.
    // if the interrupt fires, s will be handed to it
    if (register_irq_handler(s->irq,interrupt_handler,s)) {
	ERROR("failed to register interrupt handler for IRQ %d\n",s->irq);
	return -1;
    }

    s->dev = nk_char_dev_register(name,0,&interface,s);

    if (!s->dev) {
	ERROR("failed to register new character device %s\n",name);
	return -1;
    }

    if (init(s)) {
	ERROR("failed to initialize %s\n",name);
	return -1;
    }

    nk_unmask_irq(s->irq);

    INFO("detected and initialized %s (base=%x,irq=%d)\n",s->dev->dev.name,s->base_port,s->irq);

    return 0;
    
}

static int discover_and_bringup_devices()
{
    // In a non-legacy driver, or a better version of this
    // driver, we would do device discovery here to find
    // all the instances of parallel ports that we can drive.
    // Instead, here, we are just assuming that the first legacy
    // parallel port exists in the time-honored place

    return bringup(PARPORT0_BASE,PARPORT0_IRQ,"parport0");
}


int nk_parport_init()
{
    if (discover_and_bringup_devices()) {
	ERROR("discovery or bringup failed\n");
	return -1;
    }

    INFO("inited\n");
    return 0;
}
    
    

