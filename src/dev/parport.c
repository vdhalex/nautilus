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

#define ERROR(fmt, args...) ERROR_PRINT("parport: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("parport: " fmt, ##args)
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
#define DATA_PORT(s)   (s->base_port+0)     // read/write
#define STAT_PORT(s)   (s->base_port+1)     // read-only
#define CTRL_PORT(s)   (s->base_port+2)     // 


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
	uint_t irq     :1;  // 0 => interrupt asserted (active low)
	uint_t err     :1;  // attached device error line (active low)
	uint_t sel     :1;  // attached device select line
	uint_t pout    :1;  // attached device out of paper line
	uint_t ack     :1;  // attached device ack line (active low)
	uint_t busy    :1;  // attached device busy line (active low)
    } __attribute__((packed));
} __attribute__((packed)) stat_reg_t;


// control register layout - this is read/write
typedef union _ctrl_reg_t {
    uint8_t   val;
    struct {
	uint_t strobe  :1;  // attached device strobe line - alert device to data (0->1->0)
	uint_t autolf  :1;  // attached device autolf line - auomatically add linefeeds to carriage returns (if 1)
	uint_t init    :1;  // attached device init line - init attached device (if 0)
	uint_t select  :1;  // attached device select print/in 
	uint_t irq_en  :1;  // enable interrupt when ack line is asserted by attached device
	uint_t bidir_en:1;  // select transfer direction 0 => write to attached device
	uint_t res     :2;  // reserved
    } __attribute__((packed));
} __attribute__((packed)) ctrl_reg_t;



///////////////////////////////////////////////////////////////////
// Interface functions needed by the chardev abstraction
//

static int get_characteristics(void *state, struct nk_char_dev_characteristics *c)
{
    memset(c,0,sizeof(*c));
    return 0;
}


// This function and the interrupt handler are where the action is
static int read_write(void *state, uint8_t *data, int write)
{
    struct parport_state *s = (struct parport_state *)state;
    int rc=-1;
    ctrl_reg_t ctrl;
    stat_reg_t stat;

    DEBUG("doing %s of data %c\n",write ? "write" : "read", *data);

    STATE_LOCK_CONF;

    // get exclusive control
    STATE_LOCK(s);
    
    // if not ready, we are toast
    if (s->state!=READY) {
	DEBUG("not ready\n");
	rc = 0;
	goto out;
    }

    // The busy state will be reset by the interrupt handler
    s->state = BUSY;
    
    DEBUG("waiting for attached device to become ready\n");
    
    // this should not wait long...
    do {
	io_delay();
	stat.val = inb(STAT_PORT(s));
    } while (!stat.busy); // busy is active low

    DEBUG("attached device ready\n");
    
    if (write) {
	DEBUG("enabling output buffers to allow output\n");
	// enable output drivers for writing
	ctrl.val = inb(CTRL_PORT(s));
	ctrl.bidir_en =  0; // active low to enable output
	outb(ctrl.val,CTRL_PORT(s));

	DEBUG("outputing actual data\n");
	
	// actually output the data
	outb(*data,DATA_PORT(s));
	
	DEBUG("strobing attached device\n");
	
	// now do the strobe (0->1->0) to clock the
	// data to the attached device
	ctrl.val=inb(CTRL_PORT(s));
	ctrl.strobe = 1;
	outb(ctrl.val,CTRL_PORT(s));
	io_delay();
	ctrl.strobe = 0;
	outb(ctrl.val,CTRL_PORT(s));
	
    } else {
	DEBUG("disabling output buffers to allow input\n");
	// disable output drivers for reading
	ctrl.val = inb(CTRL_PORT(s));
	ctrl.bidir_en =  1; // active low to enable output
	outb(ctrl.val,CTRL_PORT(s));

	DEBUG("reading data\n");
	// actually input the data
	*data = inb(DATA_PORT(s));
	
	DEBUG("data read was %c\n",*data);
    }

    DEBUG("operation complete\n");
    
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


// This tells the chardev abstraction whether we are
// currently in a state where we can read or write, etc.
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
    
// Note that this device only fires an interrupt if the attached device
// raises its ack signal and we have this configured to produce an interrupt
// it does *not* raise an interrupt after every character unless the attached
// device (e.g., printer) will do that.
static int interrupt_handler (excp_entry_t * excp, excp_vec_t vec, void *state)
{
    STATE_LOCK_CONF;
    struct parport_state *s = (struct parport_state *)state;

    DEBUG("interrupt received - byte complete\n");

    STATE_LOCK(s);
    s->state=READY;
    STATE_UNLOCK(s);
    
    IRQ_HANDLER_END();
    
    return 0;
}

// Put the device into a known state before we do anything else
static int init(struct parport_state *s)
{
    ctrl_reg_t ctrl;

    ctrl.val = inb(CTRL_PORT(s));

    DEBUG("initial control value 0x%02x\n",ctrl);

    // set the port to output with interrupts enabled and printer selected
    ctrl.val=0;        // bidir = 0, which means we are in output mode
    ctrl.select   = 1; // attached device selected
    ctrl.init     = 1; // active low => 1 means we are not initializing it
    ctrl.irq_en   = 1; // interrupt if we get an ack on the line
    
    DEBUG("writing config %02x\n",ctrl.val);
    
    outb(ctrl.val,CTRL_PORT(s));
    
    return 0;
}


///////////////////////////////////////////////////////////////////
// Interface definition which will be used to register the device
// It consists of function pointers to earlier functions.
static struct nk_char_dev_int interface =  {
    .get_characteristics = get_characteristics,
    .read = read,
    .write = write,
    .status = status,
};

// start up one device that is located at the given port and interrupt request line
// and that we want to give the given name
static int bringup(uint16_t port, uint8_t irq, char *name)
{
    struct parport_state *s = malloc(sizeof(*s));
    
    if (!s) {
	ERROR("Failed to allocate state\n");
	return -1;
    }

    memset(s,0,sizeof(*s));

    spinlock_init(&s->lock);

    // establish our state
    s->base_port = port;
    s->irq = irq;

    // now register our interrupt handler, though we will probably
    // never use it in this code at all.
    // if the interrupt fires, s will be handed to it

    if (register_irq_handler(s->irq,interrupt_handler,s)) {
	ERROR("failed to register interrupt handler for IRQ %d\n",s->irq);
	return -1;
    }

    // Now register ourselves with the chardev subsystem as a new
    // character device with the given interface and state
    s->dev = nk_char_dev_register(name,0,&interface,s);

    if (!s->dev) {
	ERROR("failed to register new character device %s\n",name);
	return -1;
    }

    // initialize the device to a known state
    if (init(s)) {
	ERROR("failed to initialize %s\n",name);
	return -1;
    }

    // begin listening to interrupts from the device
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

// Called by the kernel to find and setup all parallel port devices
int nk_parport_init()
{
    if (discover_and_bringup_devices()) {
	ERROR("discovery or bringup failed\n");
	return -1;
    }

    INFO("inited\n");
    return 0;
}
    
    

