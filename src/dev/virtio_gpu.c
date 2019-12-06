/*
 * This file is part of the Nautilus AeroKernel developed
 * by the Hobbes and V3VEE Projects with funding from the
 * United States National Science Foundation and the Department of Energy.
 *
 * The V3VEE Project is a joint project between Northwestern University
 * and the University of New Mexico.  The Hobbes Project is a collaboration
 * led by Sandia National Laboratories that includes several national
 * laboratories and universities. You can find out more at:
 * http://www.v3vee.org  and
 * http://xstack.sandia.gov/hobbes
 *
 * Copyright (c) 2019  Peter Dinda
 * Copyright (c) 2019, The Intereaving Project <http://www.interweaving.org>
 *                     The Hobbes Project <http://xstack.sandia.gov/hobbes>
 * All rights reserved.
 *
 * Authors: Peter Dinda <pdinda@northwestern.edu>
 *
 * This is free software.  You are permitted to use,
 * redistribute, and modify it as specified in the file "LICENSE.txt".
 */

#include <nautilus/nautilus.h>
// eventually gpudev.h - for now we are a generic device
#include <nautilus/dev.h>
//#include <nautilus/blkdev.h>



#include <nautilus/irq.h>
#include <dev/pci.h>
#include <dev/virtio_gpu.h>

#ifndef NAUT_CONFIG_DEBUG_VIRTIO_GPU
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
#endif

#define INFO(fmt, args...) INFO_PRINT("virtio_gpu: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("virtio_gpu: " fmt, ##args)
#define ERROR(fmt, args...) ERROR_PRINT("virtio_gpu: " fmt, ##args)

#define FBIT_ISSET(features, bit) ((features) & (0x01 << (bit)))
#define FBIT_SETIF(features_out, features_in, bit) if (FBIT_ISSET(features_in,bit)) { features_out |= (0x01 << (bit)) ; }
#define DEBUG_FBIT(features, bit) if (FBIT_ISSET(features, bit)) {	\
	DEBUG("feature bit set: %s\n", #bit);				\
    }



#define VIRTIO_GPU_OFF_CONFIG(v)     (virtio_pci_device_regs_start(v) + 0)

#define HEADER_DESC_LEN           16  // header descriptor length
#define STATUS_DESC_LEN           1   // status descriptor length


static uint64_t num_devs = 0;

struct virtio_gpu_dev {
    // Does not currently exist
    struct nk_dev               *gpu_dev;     // nautilus gpu device
    
    struct virtio_pci_dev       *virtio_dev;  // nautilus virtio device
};


#define u8   uint8_t
#define le8  uint8_t
#define le16 uint16_t
#define le32 uint32_t
#define le64 uint64_t

// Definitions from the spec

// device capabilities
#define VIRTIO_GPU_F_VIRGL 0x1
#define VIRTIO_GPU_F_EDID  0x2

#define VIRTIO_GPU_EVENT_DISPLAY (1 << 0)
struct virtio_gpu_config {
    le32 events_read;
    le32 events_clear;
    le32 num_scanouts;
    le32 reserved;
};

enum virtio_gpu_ctrl_type {
    /* 2d commands */
    VIRTIO_GPU_CMD_GET_DISPLAY_INFO = 0x0100,
    VIRTIO_GPU_CMD_RESOURCE_CREATE_2D,
    VIRTIO_GPU_CMD_RESOURCE_UNREF,

    VIRTIO_GPU_CMD_SET_SCANOUT,
    VIRTIO_GPU_CMD_RESOURCE_FLUSH,
    VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D,
    VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING,
    VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING,
    VIRTIO_GPU_CMD_GET_CAPSET_INFO,
    VIRTIO_GPU_CMD_GET_CAPSET,
    VIRTIO_GPU_CMD_GET_EDID,
    /* cursor commands */
    VIRTIO_GPU_CMD_UPDATE_CURSOR = 0x0300,
    VIRTIO_GPU_CMD_MOVE_CURSOR,
    /* success responses */
    VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
    VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET_INFO,
    VIRTIO_GPU_RESP_OK_CAPSET,
    VIRTIO_GPU_RESP_OK_EDID,
    /* error responses */
    VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
    VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
    VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
    VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER,
};

#define VIRTIO_GPU_FLAG_FENCE (1 << 0)

struct virtio_gpu_ctrl_hdr {
        le32 type;
        le32 flags;
        le64 fence_id;
        le32 ctx_id;
        le32 padding;
};


// for VIRTIO_GPU_CMD_GET_DISPLAY_INFO request

#define VIRTIO_GPU_MAX_SCANOUTS 16

struct virtio_gpu_rect {
    le32 x;
    le32 y;
    le32 width;
    le32 height;
};

struct virtio_gpu_resp_display_info {
    struct virtio_gpu_ctrl_hdr hdr;
    struct virtio_gpu_display_one {
	struct virtio_gpu_rect r;
	le32 enabled;
	le32 flags;
    } pmodes[VIRTIO_GPU_MAX_SCANOUTS];
};


// for VIRTIO_GPU_CMD_GET_EDID
struct virtio_gpu_get_edid {
    struct virtio_gpu_ctrl_hdr hdr;
    le32 scanout;
    le32 padding;
};

struct virtio_gpu_resp_edid {
    struct virtio_gpu_ctrl_hdr hdr;
    le32 size;
    le32 padding;
    u8 edid[1024];
};

// for VIRTIO_GPU_CMD_RESOURCE_CREATE
enum virtio_gpu_formats {
    VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM  = 1,
    VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM  = 2,
    VIRTIO_GPU_FORMAT_A8R8G8B8_UNORM  = 3,
    VIRTIO_GPU_FORMAT_X8R8G8B8_UNORM  = 4,
    VIRTIO_GPU_FORMAT_R8G8B8A8_UNORM  = 67,
    VIRTIO_GPU_FORMAT_X8B8G8R8_UNORM  = 68,
    VIRTIO_GPU_FORMAT_A8B8G8R8_UNORM  = 121,
    VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM  = 134,
};

struct virtio_gpu_resource_create_2d {
    struct virtio_gpu_ctrl_hdr hdr;
    le32 resource_id;
    le32 format;
    le32 width;
    le32 height;
};

struct virtio_gpu_resource_unref { 
    struct virtio_gpu_ctrl_hdr hdr; 
    le32 resource_id; 
    le32 padding; 
};

struct virtio_gpu_resource_attach_backing { 
    struct virtio_gpu_ctrl_hdr hdr; 
    le32 resource_id; 
    le32 nr_entries; 
}; 
 
struct virtio_gpu_mem_entry { 
    le64 addr; 
    le32 length; 
    le32 padding; 
};

struct virtio_gpu_resource_detach_backing { 
        struct virtio_gpu_ctrl_hdr hdr; 
        le32 resource_id; 
        le32 padding; 
};

struct virtio_gpu_set_scanout { 
    struct virtio_gpu_ctrl_hdr hdr; 
    struct virtio_gpu_rect r; 
    le32 scanout_id; 
    le32 resource_id; 
};

struct virtio_gpu_transfer_to_host_2d { 
        struct virtio_gpu_ctrl_hdr hdr; 
        struct virtio_gpu_rect r; 
        le64 offset; 
        le32 resource_id; 
        le32 padding; 
};

struct virtio_gpu_resource_flush { 
        struct virtio_gpu_ctrl_hdr hdr; 
        struct virtio_gpu_rect r; 
        le32 resource_id; 
        le32 padding; 
};

// CURSOR STRUCTS

struct virtio_gpu_cursor_pos { 
        le32 scanout_id; 
        le32 x; 
        le32 y; 
        le32 padding; 
};

struct virtio_gpu_update_cursor { 
        struct virtio_gpu_ctrl_hdr hdr; 
        struct virtio_gpu_cursor_pos pos; 
        le32 resource_id; 
        le32 hot_x; 
        le32 hot_y; 
        le32 padding; 
};

/************************************************************
 ****************** gpu ops for kernel ********************
 ************************************************************/
// NONE EXIST CURRENTLY

/*
static void fill_hdr_desc(struct virtq *vq, struct virtio_blk_req *hdr, uint16_t hdr_index, uint16_t buf_index)
{
    struct virtq_desc *hdr_desc = &vq->desc[hdr_index];

    hdr_desc->addr = (uint64_t) hdr;
    hdr_desc->len = HEADER_DESC_LEN;
    hdr_desc->flags = 0;
    hdr_desc->next = buf_index;
    hdr_desc->flags |= VIRTQ_DESC_F_NEXT;
}

static void fill_buf_desc(struct virtq *vq, uint32_t size, uint64_t count, uint8_t *dest, uint16_t buf_index, uint16_t stat_index, uint8_t write)
{
    struct virtq_desc *buf_desc = &vq->desc[buf_index];

    buf_desc->addr = (uint64_t) dest;
    buf_desc->len = size * count;
    buf_desc->flags |= write ? 0 : VIRTQ_DESC_F_WRITE;
    buf_desc->next = stat_index;
    buf_desc->flags |= VIRTQ_DESC_F_NEXT;
}

static void fill_stat_desc(struct virtq *vq, uint8_t *status, uint16_t stat_index)
{
    struct virtq_desc *stat_desc = &vq->desc[stat_index];

    stat_desc->addr = (uint64_t) status;  
    stat_desc->flags = 0;
    stat_desc->flags |= VIRTQ_DESC_F_WRITE;
    stat_desc->len = STATUS_DESC_LEN;
    stat_desc->next = 0;
}

*/
/*
static int read_write_blocks(struct virtio_blk_dev *dev, uint64_t blocknum, uint64_t count, uint8_t *src_dest, void (*callback)(nk_block_dev_status_t, void *), void *context, uint8_t write) 
{
    DEBUG("%s blocknum = %lu count = %lu buf = %p callback = %p context = %p\n", write ? "write" : "read", blocknum, count, src_dest, callback, context);
    
    if (blocknum + count > dev->blk_config->capacity) {
        ERROR("request goes beyond device capacity\n");
        return -1;
    }

    if (write && (FBIT_ISSET(dev->virtio_dev->feat_accepted,VIRTIO_BLK_F_RO))) {
	ERROR("attempt to write read-only device\n");
	return -1;
    }

    DEBUG("[build request header]\n");

    struct virtio_blk_req *hdr = malloc(sizeof(struct virtio_blk_req));

    if (!hdr) {
	ERROR("Failed to allocate request header\n");
	return -1;
    }
    
    memset(hdr, 0, sizeof(struct virtio_blk_req));

    hdr->type = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
    hdr->sector = blocknum;
    hdr->reserved = 0;
    hdr->status = 0;

    DEBUG("[allocate descriptors]\n");

    uint16_t desc[3];

    if (virtio_pci_desc_chain_alloc(dev->virtio_dev,VIRTIO_BLK_REQUEST_QUEUE,desc,3)) {
	ERROR("Failed to allocate descriptor chain\n");
	free(hdr);
	return -1;
    }
    
    uint16_t hdr_index = desc[0];
    uint16_t buf_index = desc[1];
    uint16_t stat_index = desc[2];

    struct virtq *vq = &dev->virtio_dev->virtq[VIRTIO_BLK_REQUEST_QUEUE].vq;

    DEBUG("[create descriptors]\n");

    DEBUG("[create header descriptor]\n");
    fill_hdr_desc(vq, hdr, hdr_index, buf_index);

    DEBUG("[create buffer descriptor]\n");
    fill_buf_desc(vq, dev->blk_config->blk_size, count, src_dest, buf_index, stat_index, write);

    DEBUG("[create status descriptor]\n");
    fill_stat_desc(vq, &hdr->status, stat_index);

    dev->blk_callb[hdr_index].callback = callback;
    dev->blk_callb[hdr_index].context = context;

    DEBUG("request in indexes: header = %d, buffer = %d, status = %d\n", hdr_index, buf_index, stat_index);

    // update avail ring
    vq->avail->ring[vq->avail->idx % vq->qsz] = hdr_index;
    mbarrier();
    vq->avail->idx++;
    mbarrier();
    
    DEBUG("available ring's hdr index = %d, at ring index %d\n", hdr_index, vq->avail->idx - 1);
    DEBUG("available ring's ring index for next hdr = %u\n", vq->avail->idx);

    DEBUG("[notify device]\n");
    virtio_pci_write_regw(dev->virtio_dev, QUEUE_NOTIFY, VIRTIO_BLK_REQUEST_QUEUE);
    
    return 0;
}

static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest, void (*callback)(nk_block_dev_status_t, void *), void *context)
{
    struct virtio_blk_dev *dev = (struct virtio_blk_dev *) state;

    return read_write_blocks(dev, blocknum, count, dest, callback, context, 0);
}

static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src, void (*callback)(nk_block_dev_status_t,void *), void *context)
{
    struct virtio_blk_dev *dev = (struct virtio_blk_dev *) state;

    return read_write_blocks(dev, blocknum, count, src, callback, context, 1);
}

static struct nk_block_dev_int ops = {
    .get_characteristics = get_characteristics,
    .read_blocks = read_blocks,
    .write_blocks = write_blocks,
};


/************************************************************
 *************** interrupt handler & callback ***************
 ************************************************************/
static void teardown(struct virtio_pci_dev *dev) 
{
    // actually do frees... and reset device here...
    virtio_pci_virtqueue_deinit(dev);
}

/* 
static int process_used_ring(struct virtio_blk_dev *dev) 
{
    uint16_t hdr_desc_idx; 
    //void (*callback)(nk_block_dev_status_t, void *);
    void *context;
    struct virtio_pci_virtq *virtq = &dev->virtio_dev->virtq[VIRTIO_BLK_REQUEST_QUEUE];
    struct virtq *vq = &dev->virtio_dev->virtq[VIRTIO_BLK_REQUEST_QUEUE].vq;
     
    DEBUG("[processing used ring]\n");
    DEBUG("current virtq used index = %d\n", virtq->vq.used->idx);
    DEBUG("last seen used index = %d\n", virtq->last_seen_used);
     
    for (; virtq->last_seen_used != virtq->vq.used->idx; virtq->last_seen_used++) {
	
	// grab the head of used descriptor chain
	hdr_desc_idx = vq->used->ring[virtq->last_seen_used % virtq->vq.qsz].id;
	 
	if (vq->desc[hdr_desc_idx].flags != VIRTQ_DESC_F_NEXT)  {
	    ERROR("Huh? head in the used ring is not a header descriptor\n");
	    return -1;
	}
	 
	struct virtq_desc *hdr_desc = &vq->desc[hdr_desc_idx];
	struct virtio_blk_req *hdr = (struct virtio_blk_req *)hdr_desc->addr;
	uint8_t status = hdr->status;
	 
	DEBUG("completion for descriptor at index %d with status: %d\n", hdr_desc_idx, status);

	// grab corresponding callback
	callback = dev->blk_callb[hdr_desc_idx].callback;
	context = dev->blk_callb[hdr_desc_idx].context;
	 
	memset(&dev->blk_callb[hdr_desc_idx],0,sizeof(dev->blk_callb[hdr_desc_idx]));
	 
	free(hdr);

	DEBUG("descriptor hdr index = %u, callback = %p, context = %p\n", hdr_desc_idx, callback, context);
	 
	DEBUG("free used descriptors\n");

	if (virtio_pci_desc_chain_free(dev->virtio_dev, VIRTIO_BLK_REQUEST_QUEUE, hdr_desc_idx)) {
	    ERROR("error freeing descriptors\n");
	    return -1;
	}
	 
	if (callback) {
	    DEBUG("[issuing callback]\n");
	    callback(status ? NK_BLOCK_DEV_STATUS_ERROR : NK_BLOCK_DEV_STATUS_SUCCESS,context);
	}
    }
     
    return 0;
}

static int handler(excp_entry_t *exp, excp_vec_t vec, void *priv_data)
{
    DEBUG("[received an interrupt!]\n");
    struct virtio_blk_dev *dev = (struct virtio_blk_dev *) priv_data;
    
    // only for legacy style interrupt
    if (dev->virtio_dev->itype == VIRTIO_PCI_LEGACY) {
        DEBUG("using legacy style interrupt\n");
        // read the interrupt status register, which will reset it to zero
        uint8_t isr = virtio_pci_read_regb(dev->virtio_dev, ISR_STATUS);
	
        // if the lower bit is not set, not my interrupt
        if (!(isr & 0x1))  {
	    DEBUG("not my interrupt\n");
	    IRQ_HANDLER_END();
	    return 0;
        }
    }
    
    if (process_used_ring(dev)) {
	ERROR("failed to process used ring\n");
	IRQ_HANDLER_END();
	return -1;
    } 

    // print used for test
    //DEBUG("free count after = %d\n", dev->virtio_dev->virtq[VIRTIO_BLK_REQUEST_QUEUE].nfree);
    //uint8_t used_index = dev->virtio_dev->virtq[VIRTIO_BLK_REQUEST_QUEUE].vq.used->idx;
    //DEBUG("used ring next index = %d\n", used_index);

    DEBUG("[interrupt handler finished]\n");
    IRQ_HANDLER_END();
    return 0;
}

*/

/*************************************************
 ******************** tests **********************
 *************************************************/

/************************************************************
 ****************** device initialization *******************
 ************************************************************/
static uint64_t select_features(uint64_t features) 
{
    DEBUG("device features: 0x%0lx\n",features);
    DEBUG_FBIT(features, VIRTIO_GPU_F_VIRGL);
    DEBUG_FBIT(features, VIRTIO_GPU_F_EDID);

    // choose accepted features
    uint64_t accepted = 0;

    // probably should not feature accept either of these...
    FBIT_SETIF(accepted,features,VIRTIO_GPU_F_VIRGL);
    FBIT_SETIF(accepted,features,VIRTIO_GPU_F_EDID);
    
    DEBUG("features accepted: 0x%0lx\n", accepted);
    return accepted;
}


static int handler(excp_entry_t *exp, excp_vec_t vec, void *priv_data)
{
    //DEBUG("Interrupt invoked\n");
    IRQ_HANDLER_END();
    return 0;
}

static int open(void *state)
{
    return 0;
}

static int close(void *state)
{
    return 0;
}

static struct nk_dev_int ops = {
    .open = open,
    .close = close,
};


#define ZERO(a) memset(a,0,sizeof(*a))

static void dump_descriptors(struct virtq *vq, int start, int count)
{
    int i;
    for (i=start;i<(start+count);i++) {
	DEBUG("vq[%d] = %p len=%u flags=0x%hx next=%hu\n",
	      i,
	      vq->desc[i].addr,
	      vq->desc[i].len,
	      vq->desc[i].flags,
	      vq->desc[i].next);
    }
}

static int transact_base(struct virtio_pci_dev *dev,
			 uint16_t qidx,
			 uint16_t didx)
{
    struct virtio_pci_virtq *virtq = &dev->virtq[qidx];
    struct virtq *vq = &virtq->vq;
    uint16_t waitidx;
    uint16_t usedidx;

    //DEBUG("transact base didx=%u\n",didx);
    
    vq->avail->ring[vq->avail->idx % vq->qsz] = didx;
    mbarrier();
    vq->avail->idx++;
    waitidx = vq->avail->idx;
    mbarrier(); 

    virtio_pci_atomic_store(&dev->common->queue_select, qidx);

    // should not do this every time...
    virtio_pci_atomic_store(&dev->common->queue_enable, 1);

    //DEBUG("queue notify offset: %d\n", dev->common->queue_notify_off);

    //dump_descriptors(vq,0,8);

    //DEBUG("starting transaction on index %hu\n",didx);
    // fix to user the relevant notify addr 
    virtio_pci_atomic_store(dev->notify_base_addr, 0xFFFFFFFFF);

    //DEBUG("request initiated\n");

    do {
      usedidx = virtio_pci_atomic_load(&virtq->vq.used->idx);
      //DEBUG("vq->avail->idx: %d\n", vq->avail->idx);
      //DEBUG("vq->used->idx: %d\n", vq->used->idx);
    } while(usedidx!=waitidx);

    if (virtio_pci_desc_chain_free(dev,qidx,didx)) {
	ERROR("Failed to free descriptor chain\n");
	return -1;
    }

    //DEBUG("transaction complete\n");
    
    return 0;
}
    

// 2 fragment transaction using polling
// Note that every request and response
// begins with a virtio_gpu_ctrl_hdr
static int transact_rw(struct virtio_pci_dev *dev,
		       uint16_t qidx,
		       void    *req,
		       uint32_t reqlen,
		       void    *resp,
		       uint32_t resplen)
{
    uint16_t desc_idx[2];

    if (virtio_pci_desc_chain_alloc(dev, qidx, desc_idx, 2)) {
	ERROR("Failed to allocate descriptor chain\n");
	return -1;
    }

    //DEBUG("allocated chain %hu -> %hu\n",desc_idx[0],desc_idx[1]);

    struct virtq_desc *desc[2] = {&dev->virtq[qidx].vq.desc[desc_idx[0]],
				  &dev->virtq[qidx].vq.desc[desc_idx[1]]};

    desc[0]->addr = (le64) req;
    desc[0]->len = reqlen;
    desc[0]->flags |= 0;
    desc[0]->next = desc_idx[1];

    desc[1]->addr = (le64) resp;
    desc[1]->len = resplen;
    desc[1]->flags |= VIRTQ_DESC_F_WRITE;
    desc[1]->next = 0;

    return transact_base(dev,qidx,desc_idx[0]);
}

// 3 fragment transaction using polling
// Note that every request and response
// begins with a virtio_gpu_ctrl_hdr
static int transact_rrw(struct virtio_pci_dev *dev,
			uint16_t qidx,
			void    *req,
			uint32_t reqlen,
			void    *more,
			uint32_t morelen,
			void    *resp,
			uint32_t resplen)
{
    uint16_t desc_idx[3];

    if (virtio_pci_desc_chain_alloc(dev, qidx, desc_idx, 3)) {
	ERROR("Failed to allocate descriptor chain\n");
	return -1;
    }

    //    DEBUG("allocated chain %hu -> %hu -> %hu\n",desc_idx[0],desc_idx[1],desc_idx[2]);

    struct virtq_desc *desc[3] = {&dev->virtq[qidx].vq.desc[desc_idx[0]],
				  &dev->virtq[qidx].vq.desc[desc_idx[1]],
				  &dev->virtq[qidx].vq.desc[desc_idx[2]] };

    desc[0]->addr = (le64) req;
    desc[0]->len = reqlen;
    desc[0]->flags |= 0;
    desc[0]->next = desc_idx[1];

    desc[1]->addr = (le64) more;
    desc[1]->len = morelen;
    desc[1]->flags |= 0;
    desc[1]->next = desc_idx[2];

    desc[2]->addr = (le64) resp;
    desc[2]->len = resplen;
    desc[2]->flags |= VIRTQ_DESC_F_WRITE;
    desc[2]->next = 0;

    return transact_base(dev,qidx,desc_idx[0]);
}

// resource id "0" means "disabled" or "none"
#define MY_RID 42 

static int test_gpu(struct virtio_pci_dev *dev) {

    struct virtio_gpu_ctrl_hdr disp_info_req;
    struct virtio_gpu_resp_display_info disp_info;  // not noted as resp since used elsewhere
    ZERO(&disp_info_req);
    ZERO(&disp_info);
    
    disp_info_req.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;
    
    if (transact_rw(dev,
		    0,
		    &disp_info_req,
		    sizeof(disp_info_req),
		    &disp_info,
		    sizeof(disp_info))) {
	ERROR("Failed to get display info\n");
	return -1;
    }
	
    DEBUG("display info complete: rc=%x\n",disp_info.hdr.type);

    DEBUG("Display Info code: %x\n", disp_info.hdr.type);
    
    for (int jj = 0; jj < 16; jj++) {
	if (disp_info.pmodes[jj].enabled) { 
	    DEBUG("scanout (monitor) %u has info: %u, %u, %ux%u, %u, %u\n", jj, disp_info.pmodes[jj].r.x, disp_info.pmodes[jj].r.y, disp_info.pmodes[jj].r.width, disp_info.pmodes[jj].r.height, disp_info.pmodes[jj].flags, disp_info.pmodes[jj].enabled);
	}
    }

    
    struct virtio_gpu_resource_create_2d create_2d_req;
    struct virtio_gpu_ctrl_hdr create_2d_resp;

    ZERO(&create_2d_req);
    ZERO(&create_2d_resp);
    
    create_2d_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    create_2d_req.width = disp_info.pmodes[0].r.width;
    create_2d_req.height = disp_info.pmodes[0].r.height;
    create_2d_req.format = VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM;
    create_2d_req.resource_id = MY_RID;

    DEBUG("create 2d %u x %u @ %d id=%u\n",create_2d_req.width,create_2d_req.height,create_2d_req.format,create_2d_req.resource_id);
    
    if (transact_rw(dev,
		    0,
		    &create_2d_req,
		    sizeof(create_2d_req),
		    &create_2d_resp,
		    sizeof(create_2d_resp))) {
	ERROR("Failed to create 2d resource\n");
	return -1;
    }

    DEBUG("create 2d resource complete - rc=%x\n",create_2d_resp.type);

    // create local framebuffer
    uint64_t fb_length = disp_info.pmodes[0].r.width * disp_info.pmodes[0].r.height * 4;
    uint32_t *framebuffer = malloc(fb_length);

    if (!framebuffer) {
	ERROR("failed to allocate framebuffer of length %lu\n",fb_length);
    } else {
	DEBUG("allocated framebuffer of length %lu\n",fb_length);
    }

    DEBUG("framebuffer allocated\n");

    // fill framebuffer with a picture of memory starting at 0
    uint32_t *start = 0;
    for (int k=0;k<fb_length/4;k++) {
	framebuffer[k]= start[k]; 
    }
    
    DEBUG("framebuffer filled\n");
     
    struct virtio_gpu_resource_attach_backing backing_req;
    struct virtio_gpu_mem_entry backing_entry;
    struct virtio_gpu_ctrl_hdr backing_resp;

    ZERO(&backing_req);
    ZERO(&backing_entry);
    ZERO(&backing_resp);
	
    backing_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;

    backing_req.resource_id = MY_RID;
    backing_req.nr_entries = 1;

    backing_entry.addr = (uint64_t) framebuffer;
    backing_entry.length = fb_length;

    if (transact_rrw(dev,
		     0,
		     &backing_req,
		     sizeof(backing_req),
		     &backing_entry,
		     sizeof(backing_entry),
		     &backing_resp,
		     sizeof(backing_resp))) {
	ERROR("Failed to attach backing\n");
	return -1;
    }

    DEBUG("attach backing complete - rc=%x\n",backing_resp.type);

    struct virtio_gpu_set_scanout setso_req;
    struct virtio_gpu_ctrl_hdr setso_resp;

    ZERO(&setso_req);
    ZERO(&setso_resp);
    
    setso_req.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    setso_req.r=disp_info.pmodes[0].r;
    setso_req.resource_id=MY_RID;
    setso_req.scanout_id=0;
    
    if (transact_rw(dev,
		    0,
		    &setso_req,
		    sizeof(setso_req),
		    &setso_resp,
		    sizeof(setso_resp))) {
	ERROR("Failed to set scanout\n");
	return -1;
    }

    DEBUG("set scanout complete - rc=%x\n",setso_resp.type);
    
    struct virtio_gpu_transfer_to_host_2d xfer_req;
    struct virtio_gpu_ctrl_hdr xfer_resp;

    ZERO(&xfer_req);
    ZERO(&xfer_resp);
    
    xfer_req.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;

    xfer_req.r=disp_info.pmodes[0].r;
    xfer_req.offset = 0;
    xfer_req.resource_id=MY_RID;

    if (transact_rw(dev,
		    0,
		    &xfer_req,
		    sizeof(xfer_req),
		    &xfer_resp,
		    sizeof(xfer_resp))) {
	ERROR("Failed to transfer to host\n");
	return -1;
    }

    DEBUG("xfer complete - rc=%x\n",xfer_resp.type);

    struct virtio_gpu_resource_flush flush_req;
    struct virtio_gpu_ctrl_hdr flush_resp;

    ZERO(&flush_req);
    ZERO(&flush_resp);
    
    flush_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
	
    flush_req.r=disp_info.pmodes[0].r;
    flush_req.resource_id=MY_RID;
    
    if (transact_rw(dev,
		    0,
		    &flush_req,
		    sizeof(flush_req),
		    &flush_resp,
		    sizeof(flush_resp))) {
	ERROR("Failed to flush\n");
	return -1;
    }
    
    DEBUG("flush complete - rc=%x\n",flush_resp.type);

    // we'll walk through first 32 MB
    while(start<(uint32_t*)0x2000000) {
	// Make window slide through memory
	for (int k=0;k<fb_length/4;k++) {
	    framebuffer[k] = start[k];
	}
	start += disp_info.pmodes[0].r.width*8;

	ZERO(&xfer_req);
	ZERO(&xfer_resp);
	
	xfer_req.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
	
	xfer_req.r=disp_info.pmodes[0].r;
	xfer_req.offset = 0;
	xfer_req.resource_id=MY_RID;
	
	if (transact_rw(dev,
			0,
			&xfer_req,
			sizeof(xfer_req),
			&xfer_resp,
			sizeof(xfer_resp))) {
	    ERROR("Failed to transfer to host\n");
	    return -1;
	}
	
	ZERO(&flush_req);
	ZERO(&flush_resp);
	
	flush_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
	
	flush_req.r=disp_info.pmodes[0].r;
	flush_req.resource_id=MY_RID;
	
	if (transact_rw(dev,
			0,
			&flush_req,
			sizeof(flush_req),
			&flush_resp,
			sizeof(flush_resp))) {
	    ERROR("Failed to flush\n");
	    return -1;
	}

    }

    DEBUG("Now attempting to switch back to VGA mode\n");
 
    struct virtio_gpu_resource_detach_backing detach_req;
    struct virtio_gpu_ctrl_hdr detach_resp;

    ZERO(&detach_req);
    ZERO(&detach_resp);
	
    detach_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING;
    detach_req.resource_id=MY_RID;


    if (transact_rw(dev,
		    0,
		    &detach_req,
		    sizeof(detach_req),
		    &detach_resp,
		    sizeof(detach_resp))) {
	ERROR("Failed to detach backing\n");
	return -1;
    }

    DEBUG("detach complete - rc=%x\n",detach_resp.type);


    ZERO(&setso_req);
    ZERO(&setso_resp);
    
    setso_req.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    setso_req.r=disp_info.pmodes[0].r;
    setso_req.resource_id=0;  // disable
    setso_req.scanout_id=0;
    
    if (transact_rw(dev,
		    0,
		    &setso_req,
		    sizeof(setso_req),
		    &setso_resp,
		    sizeof(setso_resp))) {
	ERROR("Failed to reset scanout\n");
	return -1;
    }

    DEBUG("reset scanout complete - rc=%x\n",setso_resp.type);

    
    struct virtio_gpu_resource_unref unref_req;
    struct virtio_gpu_ctrl_hdr unref_resp;

    ZERO(&unref_req);
    ZERO(&unref_resp);
	
    unref_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_UNREF;
    unref_req.resource_id=MY_RID;

    if (transact_rw(dev,
		    0,
		    &unref_req,
		    sizeof(unref_req),
		    &unref_resp,
		    sizeof(unref_resp))) {
	ERROR("Failed to unref our resource\n");
	return -1;
    }

    DEBUG("unref complete - rc=%x\n",unref_resp.type);

    DEBUG("Freeing framebuffer\n");

    free(framebuffer);
}

static int run_graphics_mode(struct virtio_pci_dev *dev) {
    struct virtio_gpu_ctrl_hdr disp_info_req;
    struct virtio_gpu_resp_display_info disp_info;  // not noted as resp since used elsewhere
    ZERO(&disp_info_req);
    ZERO(&disp_info);
    
    disp_info_req.type = VIRTIO_GPU_CMD_GET_DISPLAY_INFO;
    
    if (transact_rw(dev,
		    0,
		    &disp_info_req,
		    sizeof(disp_info_req),
		    &disp_info,
		    sizeof(disp_info))) {
	ERROR("Failed to get display info\n");
	return -1;
    }
	
    DEBUG("display info complete: rc=%x\n",disp_info.hdr.type);

    DEBUG("Display Info code: %x\n", disp_info.hdr.type);

    struct virtio_gpu_resource_create_2d create_2d_req;
    struct virtio_gpu_ctrl_hdr create_2d_resp;

    ZERO(&create_2d_req);
    ZERO(&create_2d_resp);
    
    create_2d_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_CREATE_2D;
    create_2d_req.width = disp_info.pmodes[0].r.width;
    create_2d_req.height = disp_info.pmodes[0].r.height;
    create_2d_req.format = VIRTIO_GPU_FORMAT_R8G8B8X8_UNORM;
    create_2d_req.resource_id = MY_RID;

    DEBUG("create 2d %u x %u @ %d id=%u\n",create_2d_req.width,create_2d_req.height,create_2d_req.format,create_2d_req.resource_id);
    
    if (transact_rw(dev,
		    0,
		    &create_2d_req,
		    sizeof(create_2d_req),
		    &create_2d_resp,
		    sizeof(create_2d_resp))) {
	ERROR("Failed to create 2d resource\n");
	return -1;
    }

    DEBUG("create 2d resource complete - rc=%x\n",create_2d_resp.type);

    // create local framebuffer
    uint64_t fb_length = disp_info.pmodes[0].r.width * disp_info.pmodes[0].r.height * 4;
    uint32_t *framebuffer = malloc(fb_length);

    if (!framebuffer) {
	ERROR("failed to allocate framebuffer of length %lu\n",fb_length);
    } else {
	DEBUG("allocated framebuffer of length %lu\n",fb_length);
    }

    DEBUG("framebuffer allocated\n");

    // fill framebuffer with a picture of memory starting at 0
    // uint32_t *start = 0;
    // for (int k=0;k<fb_length/4;k++) {
	// framebuffer[k]= start[k]; 
    // }
    
    DEBUG("framebuffer filled\n");
     
    struct virtio_gpu_resource_attach_backing backing_req;
    struct virtio_gpu_mem_entry backing_entry;
    struct virtio_gpu_ctrl_hdr backing_resp;

    ZERO(&backing_req);
    ZERO(&backing_entry);
    ZERO(&backing_resp);
	
    backing_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING;

    backing_req.resource_id = MY_RID;
    backing_req.nr_entries = 1;

    backing_entry.addr = (uint64_t) framebuffer;
    backing_entry.length = fb_length;

    if (transact_rrw(dev,
		     0,
		     &backing_req,
		     sizeof(backing_req),
		     &backing_entry,
		     sizeof(backing_entry),
		     &backing_resp,
		     sizeof(backing_resp))) {
	ERROR("Failed to attach backing\n");
	return -1;
    }

    DEBUG("attach backing complete - rc=%x\n",backing_resp.type);

    struct virtio_gpu_set_scanout setso_req;
    struct virtio_gpu_ctrl_hdr setso_resp;

    ZERO(&setso_req);
    ZERO(&setso_resp);
    
    setso_req.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    setso_req.r=disp_info.pmodes[0].r;
    setso_req.resource_id=MY_RID;
    setso_req.scanout_id=0;
    
    if (transact_rw(dev,
		    0,
		    &setso_req,
		    sizeof(setso_req),
		    &setso_resp,
		    sizeof(setso_resp))) {
	ERROR("Failed to set scanout\n");
	return -1;
    }

    DEBUG("set scanout complete - rc=%x\n",setso_resp.type);
    
    struct virtio_gpu_transfer_to_host_2d xfer_req;
    struct virtio_gpu_ctrl_hdr xfer_resp;

    ZERO(&xfer_req);
    ZERO(&xfer_resp);
    
    xfer_req.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;

    xfer_req.r=disp_info.pmodes[0].r;
    xfer_req.offset = 0;
    xfer_req.resource_id=MY_RID;

    if (transact_rw(dev,
		    0,
		    &xfer_req,
		    sizeof(xfer_req),
		    &xfer_resp,
		    sizeof(xfer_resp))) {
	ERROR("Failed to transfer to host\n");
	return -1;
    }

    DEBUG("xfer complete - rc=%x\n",xfer_resp.type);

    struct virtio_gpu_resource_flush flush_req;
    struct virtio_gpu_ctrl_hdr flush_resp;

    ZERO(&flush_req);
    ZERO(&flush_resp);
    
    flush_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
	
    flush_req.r=disp_info.pmodes[0].r;
    flush_req.resource_id=MY_RID;
    
    if (transact_rw(dev,
		    0,
		    &flush_req,
		    sizeof(flush_req),
		    &flush_resp,
		    sizeof(flush_resp))) {
	ERROR("Failed to flush\n");
	return -1;
    }
    
    DEBUG("flush complete - rc=%x\n",flush_resp.type);

    // NOW ATTEMPTING TO KEEP TRACK OF MOUSE
    struct virtio_gpu_cursor_pos curr_pos;
    struct virtio_gpu_ctrl_hdr update_cursor_req;
    struct virtio_gpu_update_cursor cursor_resp;
    ZERO(&curr_pos);
    ZERO(&update_cursor_req);
    ZERO(&cursor_resp);

    int y = 384; // setting y to a default value for this test
    
    for (int x_i = 0; x_i < 1024; x_i++) {
        // fill in new cursor information
        update_cursor_req.type = VIRTIO_GPU_CMD_UPDATE_CURSOR;
        curr_pos.x = x_i;
        curr_pos.y = y;
        curr_pos.scanout_id = 0;

        cursor_resp.pos = curr_pos;
        cursor_resp.resource_id = MY_RID;

        if (transact_rw(dev, 1, &update_cursor_req, sizeof(update_cursor_req), &cursor_resp, sizeof(cursor_resp))) {
            ERROR("Failed to update cursor at %u\n", x_i);
            return -1;
        }

        ZERO(&xfer_req);
        ZERO(&xfer_resp);
        
        xfer_req.hdr.type = VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D;
        
        xfer_req.r=disp_info.pmodes[0].r;
        xfer_req.offset = 0;
        xfer_req.resource_id=MY_RID;
        
        if (transact_rw(dev,
                0,
                &xfer_req,
                sizeof(xfer_req),
                &xfer_resp,
                sizeof(xfer_resp))) {
            ERROR("Failed to transfer to host\n");
            return -1;
        }
        
        ZERO(&flush_req);
        ZERO(&flush_resp);
        
        flush_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_FLUSH;
        
        flush_req.r=disp_info.pmodes[0].r;
        flush_req.resource_id=MY_RID;
        
        if (transact_rw(dev,
                0,
                &flush_req,
                sizeof(flush_req),
                &flush_resp,
                sizeof(flush_resp))) {
            ERROR("Failed to flush\n");
            return -1;
        }
    }

    DEBUG("Now attempting to switch back to VGA mode\n");
 
    struct virtio_gpu_resource_detach_backing detach_req;
    struct virtio_gpu_ctrl_hdr detach_resp;

    ZERO(&detach_req);
    ZERO(&detach_resp);
	
    detach_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING;
    detach_req.resource_id=MY_RID;


    if (transact_rw(dev,
		    0,
		    &detach_req,
		    sizeof(detach_req),
		    &detach_resp,
		    sizeof(detach_resp))) {
	ERROR("Failed to detach backing\n");
	return -1;
    }

    DEBUG("detach complete - rc=%x\n",detach_resp.type);


    ZERO(&setso_req);
    ZERO(&setso_resp);
    
    setso_req.hdr.type = VIRTIO_GPU_CMD_SET_SCANOUT;
    setso_req.r=disp_info.pmodes[0].r;
    setso_req.resource_id=0;  // disable
    setso_req.scanout_id=0;
    
    if (transact_rw(dev,
		    0,
		    &setso_req,
		    sizeof(setso_req),
		    &setso_resp,
		    sizeof(setso_resp))) {
	ERROR("Failed to reset scanout\n");
	return -1;
    }

    DEBUG("reset scanout complete - rc=%x\n",setso_resp.type);

    
    struct virtio_gpu_resource_unref unref_req;
    struct virtio_gpu_ctrl_hdr unref_resp;

    ZERO(&unref_req);
    ZERO(&unref_resp);
	
    unref_req.hdr.type = VIRTIO_GPU_CMD_RESOURCE_UNREF;
    unref_req.resource_id=MY_RID;

    if (transact_rw(dev,
		    0,
		    &unref_req,
		    sizeof(unref_req),
		    &unref_resp,
		    sizeof(unref_resp))) {
	ERROR("Failed to unref our resource\n");
	return -1;
    }

    DEBUG("unref complete - rc=%x\n",unref_resp.type);

    DEBUG("Freeing framebuffer\n");

    free(framebuffer);

    return 0;
}

int virtio_gpu_init(struct virtio_pci_dev *dev)
{
    char buf[DEV_NAME_LEN];
    
    DEBUG("initialize device\n");
    
    // initialize block device
    struct virtio_gpu_dev *d = malloc(sizeof(*d));
    if (!d) {
	ERROR("cannot allocate state\n");
	return -1;
    }
    memset(d,0,sizeof(*d));
    
    // acknowledge device
    if (virtio_pci_ack_device(dev)) {
        ERROR("Could not acknowledge device\n");
        free(d);
        return -1;
    }

    // read device feature bits
    if (virtio_pci_read_features(dev)) {
        ERROR("Unable to read device features\n");
        free(d);
        return -1;
    }

    // accept feature bits
    if (virtio_pci_write_features(dev, select_features(dev->feat_offered))) {
        ERROR("Unable to write device features\n");
        free(d);
        return -1;
    }
    
    // initilize device virtqueue
    if (virtio_pci_virtqueue_init(dev)) {
	ERROR("failed to initialize virtqueues\n");
	free(d);
	return -1;
    }
    
    dev->state = d;
    dev->teardown = teardown;
    d->virtio_dev = dev;
    
    //parse_config(d);
    
    // register virtio gpu device, currently just as a generic device
    snprintf(buf,DEV_NAME_LEN,"virtio-gpu%u",__sync_fetch_and_add(&num_devs,1));
    d->gpu_dev = nk_dev_register(buf,NK_DEV_GENERIC,0,&ops,d);			       
    
    if (!d->gpu_dev) {
	ERROR("failed to register block device\n");
	virtio_pci_virtqueue_deinit(dev);
	free(d);
	return -1;
    }
    
    // We assume that interrupt allocations will not fail...
    // if we do fail, the rest of this code will leak
    
    struct pci_dev *p = dev->pci_dev;
    uint8_t i;
    ulong_t vec;
    
    if (dev->itype==VIRTIO_PCI_MSI_X_INTERRUPT) {
	// we assume MSI-X has been enabled on the device
	// already, that virtqueue setup is done, and
	// that queue i has been mapped to MSI-X table entry i
	// MSI-X is on but whole function is masked

	DEBUG("setting up interrupts via MSI-X\n");
	
	if (dev->num_virtqs != p->msix.size) {
	    ERROR("weird mismatch: numqueues=%u msixsize=%u\n", dev->num_virtqs, p->msix.size);
	    // continue for now...
	    // return -1;
	}
	
	// this should really go by virtqueue, not entry
	// and ideally pulled into a per-queue setup routine
	// in virtio_pci...
	uint16_t num_vec = p->msix.size;
        
	// now fill out the device's MSI-X table
	for (i=0;i<num_vec;i++) {
	    // find a free vector
	    // note that prioritization here is your problem
	    if (idt_find_and_reserve_range(1,0,&vec)) {
		ERROR("cannot get vector...\n");
		return -1;
	    }
	    // register your handler for that vector
	    if (register_int_handler(vec, handler, d)) {
		ERROR("failed to register int handler\n");
		return -1;
		// failed....
	    }
	    // set the table entry to point to your handler
	    if (pci_dev_set_msi_x_entry(p,i,vec,0)) {
		ERROR("failed to set MSI-X entry\n");
		return -1;
	    }
	    // and unmask it (device is still masked)
	    if (pci_dev_unmask_msi_x_entry(p,i)) {
		ERROR("failed to unmask entry\n");
		return -1;
	    }
	    DEBUG("finished setting up entry %d for vector %u on cpu 0\n",i,vec);
	}
	
	// unmask entire function
	if (pci_dev_unmask_msi_x_all(p)) {
	    ERROR("failed to unmask device\n");
	    return -1;
	}
	
    } else {

	ERROR("This device must operate with MSI-X\n");
	return -1;
    }
    
    DEBUG("device inited\n");
    
    /*************************************************
     ********************* TEST **********************
     *************************************************/
    // DEBUG("******* Running read_blocks or write_blocks *******\n");
    // test_read(d);
    // test_write(d);
    DEBUG("****** Running graphics mode ******\n");
    if (run_graphics_mode(dev)) {
        DEBUG("test gpu failed");
    }
    
    // DEBUG("******* Running descriptor chain test *******\n");
    // struct virtio_pci_virtq *virtq = &dev->virtq[0];
    // struct virtq *vq = &virtq->vq;
    // uint64_t qsz = vq->num;
    // for (i = 0; i < qsz; i++) {
    //   DEBUG("Next pointer of descriptor %d is %d\n", i, vq->desc[i].next);
    // }

    // disable this on a functional qemu
#if 1
    DEBUG("Skipping reset to VGA mode, which will only work on a non-buggy QEMU (4.1.1)\n");
    return 0;
#endif
    
    DEBUG("Reseting device\n");
    
    // now reset the device to get us back into VGA compatibility
    virtio_pci_atomic_store(&dev->common->device_status,0);


    DEBUG("Done\n");

    return 0;
}
