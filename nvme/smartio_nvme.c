/*********************************************************************************
 *                                                                               *
 * Copyright (C) 1993 - 2019                                                     *
 *         Dolphin Interconnect Solutions AS                                     *
 *                                                                               *
 * This program is free software; you can redistribute it and/or modify          *
 * it under the terms of the GNU General Public License as published by          *
 * the Free Software Foundation; either version 2 of the License,                *
 * or (at your option) any later version.                                        *
 *                                                                               *
 * This program is distributed in the hope that it will be useful,               *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of                *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 *
 * GNU General Public License for more details.                                  *
 *                                                                               *
 * You should have received a copy of the GNU General Public License             *
 * along with this program; if not, write to the Free Software                   *
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.   *
 *                                                                               *
 *                                                                               *
 *********************************************************************************/ 

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "sisci_types.h"
#include "sisci_api.h"
#include "sisci_error.h"

#ifdef OS_IS_VXWORKS
#include <vxWorks.h>
#include <sysLib.h>
#include <taskLib.h>
#include <sys/times.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <process.h>
#else
#include <unistd.h>
#endif


/*
 * Some convenience macros.
 */
#define _MIN(a, b)                  ((a) <= (b) ? (a) : (b))
#define _MASK(nbits)                ((1ULL << (nbits)) - 1)

/* 
 * The get_bits and set_bits macros follow the bit XX:YY notation in
 * the NVMe 1.3 specification.
 */
#define _MASK_PART(hi, lo)          (_MASK((hi) + 1) - _MASK((lo)))
#define get_bits(value, hi, lo)     ( ( (value) & _MASK_PART((hi), (lo)) ) >> (lo) )
#define bitsw(value, hi, lo)        ((((uint64_t) (value)) << (lo)) & _MASK_PART((hi), (lo)))


#define NVME_OPCODE(generic, function, dtrans) \
    (bitsw((generic), 7, 7) | bitsw((function), 6, 2) | bitsw((dtrans), 1, 0))


/* NVMe command opcodes */
enum nvme_opcodes
{
    NVME_IDENTIFY   =   NVME_OPCODE(0, 1, 2)
};


/*
 * Mapping to device memory segments.
 */
struct device_mem_map
{
    sci_remote_segment_t    segment;    /* Device memory segment */
    sci_map_t               map_handle; /* Memory mapping handle */
    volatile void*          map_ptr;    /* Memory mapped pointer */
};


/*
 * Controller register definition.
 * See section 3.1 in the NVMe 1.3 specification.
 */
struct registers
{
    uint64_t cap;       /* Controller capabilities */
    uint32_t vs;        /* Version */
    uint32_t intms;     /* Interrupt mask set */
    uint32_t intmc;     /* Interrupt mask clear */
    uint32_t cc;        /* Controller configuration */
    uint32_t reserved;
    uint32_t csts;      /* Controller status */
    uint32_t nssr;      /* NVM subsystem reset */
    uint32_t aqa;       /* Admin queue attributes */
    uint64_t asq;       /* Admin submission queue */
    uint64_t acq;       /* Admin completion queue */
    uint32_t cmbloc;    /* Controller memory buffer location */
    uint32_t cmbsz;     /* Controller memory buffer size */
    uint32_t bpinfo;    /* Boot partition information */
    uint32_t bprsel;    /* Boot partition read select */
    uint64_t bpmbl;     /* Boot partition memory buffer location */
};


/*
 * Queue descriptor.
 * Describes a NVM queue and holds necessary handles and descriptors.
 */
struct nvme_queue 
{
    struct device_mem_map   mlbar;      /* Mapping to MLBAR */
    struct device_mem_map   queue_mem;  /* Queue memory */
    sci_ioaddr_t            ioaddr;     /* IO address (seen from the device */
    uint16_t                no;         /* Queue number */
    uint16_t                qs;         /* Queue size (in number of entries) */
    uint16_t                es;         /* Entry size (in number of bytes) */
    int                     phase;      /* Current phase tag */
    int                     empty;       /* Indicates that the queue is empty */
    uint32_t                head;       /* Queue's head pointer */
    uint32_t                tail;       /* Queue's tail pointer */
};


/*
 * NVMe controller device handle.
 */
struct nvme_device
{
    sci_smartio_device_t    device;     /* SmartIO device handle */
    struct device_mem_map   mlbar;      /* Mapping to MLBAR (BAR0) */
    struct nvme_queue       acq;        /* Admin completion queue */
    struct nvme_queue       asq;        /* Admin submission queue */
};




/*
 * Command queue entry.
 */
struct command
{
    uint32_t    dword[16];
};


/*
 * Completion queue entry
 */
struct completion
{
    uint32_t    dword[4];
};



/*
 * Helper function for setting up a command
 */
static void prepare_command(struct command* cmd, uint16_t cid, uint8_t opcode, uint32_t ns_id, uint64_t dptr)
{
    memset(cmd, 0, sizeof(*cmd));
    cmd->dword[0] = (((uint32_t) cid) << 16) | (opcode & 0x7f);
    cmd->dword[1] = ns_id;
    cmd->dword[6] = (uint32_t) dptr;
    cmd->dword[7] = (uint32_t) (dptr >> 32UL);
    cmd->dword[8] = 0;
    cmd->dword[9] = 0;
}



/*
 * Helper function to get doorbell register of a queue.
 */
static volatile uint32_t* get_doorbell(const struct nvme_queue* queue, int sq)
{
    uint64_t cap;
    size_t dstrd;
    volatile void* ptr;

    cap = ((const volatile struct registers*) queue->mlbar.map_ptr)->cap;
    dstrd = get_bits(cap, 35, 32);

    ptr = ((volatile unsigned char*) queue->mlbar.map_ptr) + 0x1000 + ((2 * queue->no + !sq) * (4 << dstrd));
    return (volatile uint32_t*) ptr;
}



static int sq_enqueue(struct nvme_queue* sq, const struct command* cmd)
{
    volatile unsigned char* queue_ptr;
    volatile struct command* slot;

    /* Check if queue is full */
    if ((sq->tail - sq->head) % sq->qs == 0 && !sq->empty) {
        return EBUSY;
    }

    sq->empty = 0;

    queue_ptr = sq->queue_mem.map_ptr;
    slot = (volatile struct command*) (queue_ptr + sq->es * sq->tail);

    /* 
     * Write command to queue memory.
     * Since queue memory may be remote, we want to exploit
     * the write combining buffer by writing all 64 bytes
     * at once.
     */
    *slot = *cmd;

    /* Update tail pointer and wrap if necessary */
    if (++sq->tail == sq->qs) {
        sq->phase = !sq->phase;
        sq->tail = 0;
    }

    return 0;
}



/*
 * Submit all commands enqueued
 */
static void sq_submit(struct nvme_queue* sq)
{
    volatile uint32_t* db;
    sci_error_t err;

    SCICacheSync(NULL, (void*) sq->queue_mem.map_ptr, sq->es * sq->qs, SCI_FLAG_CACHE_FLUSH, &err);
    SCIFlush(NULL, 0);

    db = get_doorbell(sq, 1);
    *db = sq->tail;
}


/*
 * Finish processing a single completion by updating SQ head
 */
static void sq_update(struct nvme_queue* sq)
{
    if (++sq->head == sq->qs) {
        sq->head = 0;
    }

    sq->empty = sq->head == sq->tail;
}



static int cq_dequeue(struct nvme_queue* cq, struct completion* cpl)
{
    sci_error_t err;
    volatile unsigned char* queue_ptr;
    volatile struct completion* slot;
    
    queue_ptr = cq->queue_mem.map_ptr;
    slot = (volatile struct completion*) (queue_ptr + cq->es * cq->head);
    
    SCICacheSync(NULL, (void*) slot, cq->es, SCI_FLAG_CACHE_FLUSH | SCI_FLAG_CACHE_INVALIDATE, &err);

    /* Check if new completion is ready by checking the phase tag */
    if (!!get_bits(slot->dword[3], 16, 16) != cq->phase) {
        return EBUSY;
    }

    *cpl = *slot;

    if (++cq->head == cq->qs) {
        cq->head = 0;
        cq->phase = !cq->phase;
    }

    return 0;
}



static void cq_update(struct nvme_queue* cq)
{
    volatile uint32_t* db = get_doorbell(cq, 0);
    *db = cq->head;
    SCIFlush(NULL, 0);
    cq->tail = cq->head;
}



/* 
 * Helper function to calculate the 
 * base-2 logarithm of a number.
 */
static uint8_t b2log(uint64_t n)
{
    uint8_t count = 0;

    while (n > 0) {
        ++count;
        n >>= 1;
    }

    return count - 1;
}



/*
 * Helper function to retrieve controller supported page size (MPSMIN)
 * from controller capabilities.
 */
static size_t get_page_size(const struct device_mem_map* mlbar)
{
    uint64_t cap = ((const volatile struct registers*) mlbar->map_ptr)->cap;
    return 1ULL << (12 + get_bits(cap, 51, 48));
}



/*
 * Helper function to retrieve controller timeout (TO) from
 * controller capabilities.
 */
static uint32_t get_timeout(const struct device_mem_map* mlbar)
{
    uint64_t cap = ((const volatile struct registers*) mlbar->map_ptr)->cap;
    return get_bits(cap, 31, 24) * 500;
}



/*
 * Helper function to sleep a number of milliseconds.
 * Note: ms must be less than 1000.
 */
static void sleep_ms(unsigned int ms)
{
#if defined(WIN32)
    Sleep(ms);
#elif defined (OS_IS_VXWORKS)
    taskDelay((CLOCKS_PER_SEC * ms) / 1000);
#else
    usleep(1000 * ms);
#endif
}



/* 
 * Enqueue a command and submit it straight away
 */
static int nvme_submit_command(struct nvme_queue* sq, const struct command* cmd)
{
    int err;

    err = sq_enqueue(sq, cmd);
    if (err != 0) {
        return err;
    }

    sq_submit(sq);
    return 0;
}



/* 
 * Wait for command completion 
 */
static int nvme_wait_completion(struct nvme_queue* cq, struct completion* cpl)
{
    int err;
    uint32_t timeout = get_timeout(&cq->mlbar);

    err = cq_dequeue(cq, cpl);
    while (err == EBUSY) {
        if (timeout == 0) {
            return ETIME;
        }

        sleep_ms(100);
        timeout -= _MIN(timeout, 100);
        err = cq_dequeue(cq, cpl);
    } 

    cq_update(cq);

    return 0;
}



/*
 * Map a device memory segment for the local application.
 */
static int map_device_mem(struct device_mem_map* mem, sci_smartio_device_t device, uint32_t type, uint32_t id)
{
    uint32_t flags = 0;
    sci_error_t err;
    size_t size;

    SCIConnectDeviceSegment(device, &mem->segment, id, type, NULL, NULL, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to connect to device memory: %s\n",
                SCIGetErrorString(err));
        return EIO;
    }

    size = SCIGetRemoteSegmentSize(mem->segment);
    flags = (type & SCI_MEMTYPE_BAR) ? SCI_FLAG_IO_MAP_IOSPACE : 0;

    mem->map_ptr = SCIMapRemoteSegment(mem->segment, &mem->map_handle, 0, size, NULL, flags, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to map device memory: %s\n",
                SCIGetErrorString(err));
        SCIDisconnectSegment(mem->segment, 0, &err);
        return ENOSPC;
    }

    return 0;
}



/*
 * Unmap a device memory segment for the local application.
 */
static void unmap_device_mem(struct device_mem_map* mem)
{
    sci_error_t err;

    SCIUnmapSegment(mem->map_handle, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to unmap segment: %s\n",
                SCIGetErrorString(err));
    }

    mem->map_ptr = NULL;

    do {
        SCIDisconnectSegment(mem->segment, 0, &err);
    } while (err == SCI_ERR_BUSY);

    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to disconnect from segment: %s\n",
                SCIGetErrorString(err));
    }
}



/*
 * Set queue handle to its initial state.
 */
static void reset_queue(struct nvme_queue* queue)
{
    memset((void*) queue->queue_mem.map_ptr, 0, queue->es * queue->qs);
    queue->phase = 1;
    queue->head = 0;
    queue->tail = 0;
    queue->empty = 1;
}



/*
 * Allocate queue memory, map it for device and initialize queue descriptor.
 */
static int create_queue(struct nvme_queue* queue, sci_smartio_device_t device, 
        size_t page_size, uint16_t no, int sq)
{
    int err;
    sci_error_t scierr;
    uint32_t hints = 0;

    err = map_device_mem(&queue->mlbar, device, SCI_MEMTYPE_BAR, 0);
    if (err != 0) {
        return err;
    }

    /* Figure out where to place queue memory */
    if (sq) {
        hints = SCI_MEMACCESS_DEVICE_READ | SCI_MEMACCESS_HOST_WRITE;
    } else {
        hints = SCI_MEMACCESS_DEVICE_WRITE | SCI_MEMACCESS_HOST_READ;
    }

    /* Allocate queue memory */
    SCICreateDeviceSegment(device, 2*no + sq, page_size, 
            SCI_MEMTYPE_PRIVATE, hints, 0, &scierr);
    if (scierr != SCI_ERR_OK) {
        fprintf(stderr, "Failed to create private device segment: %s\n",
                SCIGetErrorString(scierr));
        return ENOSPC;
    }

    /* Map controller registers for ourselves */
    err = map_device_mem(&queue->queue_mem, device, SCI_MEMTYPE_PRIVATE, 2*no + sq);
    if (err != 0) {
        unmap_device_mem(&queue->mlbar);
        return err;
    }

    /* Map queue memory for device */
    SCIMapRemoteSegmentForDevice(queue->queue_mem.segment, device, 
            &queue->ioaddr, 0, page_size, 0, &scierr);
    if (scierr != SCI_ERR_OK) {
        unmap_device_mem(&queue->queue_mem);
        unmap_device_mem(&queue->mlbar);
        fprintf(stderr, "Failed to map segment for device: %s\n",
                SCIGetErrorString(scierr));
        return EIO;
    }

    queue->no = no;
    if (sq) {
        queue->es = sizeof(struct command);
    } else {
        queue->es = sizeof(struct completion);
    }
    queue->qs = page_size / queue->es;

    reset_queue(queue);
    return 0;
}



/*
 * Unmap queue memory for device and release queue memory.
 */
static void remove_queue(struct nvme_queue* queue, sci_smartio_device_t device)
{
    sci_error_t err;

    if (!queue->empty || queue->head != queue->tail) {
        fprintf(stderr, "Warning: Queue is not empty on removal, device may still be using it!\n");
    }

    SCIUnmapRemoteSegmentForDevice(queue->queue_mem.segment, device, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to unmap device memory segment for device: %s\n",
                SCIGetErrorString(err));
    }

    unmap_device_mem(&queue->queue_mem);
    unmap_device_mem(&queue->mlbar);
}



/*
 * Query device and get an adapter we can reach the device on.
 */
static sci_error_t query_device(sci_smartio_device_t device, sci_smartio_device_info_t* info)
{
    sci_error_t err;
    sci_smartio_query_device_t query = {0};

    memset(info, 0, sizeof(*info));

    query.fdid = SCIGetFabricDeviceId(device);
    query.subcommand = SCI_Q_DEVICE_INFO;
    query.data = info;

    SCIQuery(SCI_Q_DEVICE, &query, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to query device: %s\n", SCIGetErrorString(err));
        return err;
    }

    return SCI_ERR_OK;
}



/*
 * Loop through a list of discovered NVMe devices and try to borrow one of them.
 */
static int get_device(struct nvme_device* device, sci_desc_t sd)
{
    int err;
    sci_error_t scierr;
    size_t num_devices;
    uint32_t device_ids[64] = {0};
    size_t device_it;
    size_t page_size;
    sci_smartio_device_info_t filter = {0};

    filter.pci_class = 0x010802; /* Filter on NVMe drives, see Table 2.1.5 */

    /* Get a list of discovered SmartIO devices matching filter */
    num_devices = SCIGetDeviceList(sd, device_ids, 64, &filter, 0, &scierr);

    /* Loop through discovered devices and try to borrow one of them */
    fprintf(stdout, "Discovered %lu NVMe devices\n", 
            (unsigned long) num_devices);
    for (device_it = 0; device_it < num_devices; ++device_it) {
        fprintf(stdout, "Trying to borrow device %x...", device_ids[device_it]);

        SCIBorrowDevice(sd, &device->device, device_ids[device_it], 
                SCI_FLAG_EXCLUSIVE, &scierr);
        if (scierr == SCI_ERR_OK) {
            fprintf(stdout, "OK\n");
            break;

        } else {
            fprintf(stdout, "FAIL\n");
        }
    }

    /* Could not find any suitable devices */
    if (device_it == num_devices) {
        return ENODEV;
    }

    /* Map MLBAR (BAR0) of the drive */
    err = map_device_mem(&device->mlbar, device->device, SCI_MEMTYPE_BAR, 0);
    if (err != 0) {
        SCIReturnDevice(device->device, 0, &scierr);
        return err;
    }

    page_size = get_page_size(&device->mlbar);

    /* Allocate admin queues */
    err = create_queue(&device->acq, device->device, page_size, 0, 0);
    if (err != 0) {
        unmap_device_mem(&device->mlbar);
        SCIReturnDevice(device->device, 0, &scierr);
        return err;
    }

    err = create_queue(&device->asq, device->device, page_size, 0, 1);
    if (err != 0) {
        remove_queue(&device->acq, device->device);
        unmap_device_mem(&device->mlbar);
        SCIReturnDevice(device->device, 0, &scierr);
        return err;
    }

    return 0;
}



/*
 * Unmap device memory mappings and return device.
 */
static void put_device(struct nvme_device* device)
{
    sci_error_t err;

    remove_queue(&device->asq, device->device);
    remove_queue(&device->acq, device->device);
    unmap_device_mem(&device->mlbar);

    SCIReturnDevice(device->device, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to return device: %s\n",
                SCIGetErrorString(err));
    }
}



static sci_error_t create_local_segment(sci_local_segment_t* segment, 
        sci_desc_t sd, unsigned int adapter, size_t size)
{
    sci_error_t err;

    SCICreateSegment(sd, segment, 123, size, NULL, NULL, SCI_FLAG_AUTO_ID, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to create local segment: %s\n",
                SCIGetErrorString(err));
        return err;
    }

    SCIPrepareSegment(*segment, adapter, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to prepare local segment on adapter: %s\n",
                SCIGetErrorString(err));
        return err;
    }

    return SCI_ERR_OK;
}



static void remove_local_segment(sci_local_segment_t segment, unsigned int adapter)
{
    sci_error_t err;

    do {
        SCIRemoveSegment(segment, 0, &err);
    } while (err == SCI_ERR_BUSY);

    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to remove local segment: %s\n",
                SCIGetErrorString(err));
    }
}



static void show_controller_info(const struct nvme_device* device, const void* identify_ptr)
{
    const unsigned char* ptr = identify_ptr;
    size_t mdts;
    uint32_t vs;
    uint16_t mjr;
    uint8_t mnr;
    uint8_t ter;
    sci_smartio_device_info_t info;
    char serial_str[21];
    char model_str[41];

    strncpy(serial_str, ptr + 4, 20);
    strncpy(model_str, ptr + 24, 40);

    serial_str[20] = '\0';
    model_str[40]  = '\0';

    query_device(device->device, &info);

    vs = ((const volatile struct registers*) device->mlbar.map_ptr)->vs;
    mjr = get_bits(vs, 31, 16);
    mnr = get_bits(vs, 15, 8);
    ter = get_bits(vs, 7, 0);

    mdts = (1UL << *(ptr + 77)) * get_page_size(&device->mlbar);

    fprintf(stdout, "Fabric device identifier: %x\n", info.fdid);
    fprintf(stdout, "NVMe version            : %u.%u.%u\n", mjr, mnr, ter);
    fprintf(stdout, "Identify vendor         : %02x%02x\n", ptr[1], ptr[2]);
    fprintf(stdout, "PCI vendor              : %04x\n", info.pci_vendor_id);
    fprintf(stdout, "PCI device              : %04x\n", info.pci_device_id);
    fprintf(stdout, "User-defined name       : %s\n", info.user_name);
    fprintf(stdout, "Serial name             : %s\n", serial_str);
    fprintf(stdout, "Model name              : %s\n", model_str);
    fprintf(stdout, "Maximum data transfer   : %llu bytes\n",
            (unsigned long long) mdts);
}



/*
 * Map local segment for device and identify NVMe controller.
 */
int identify_controller(struct nvme_device* device, unsigned int adapter, sci_local_segment_t segment)
{
    int status = 0;
    struct command cmd;
    struct completion cpl;
    sci_error_t err;
    sci_ioaddr_t ioaddr;
    size_t size;
    sci_map_t map;
    const void* ptr;

    size = SCIGetLocalSegmentSize(segment);

    SCISetSegmentAvailable(segment, adapter, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to set segment available: %s\n",
                SCIGetErrorString(err));
        return EIO;
    }

    /* Map segment for device and get the IO address as seen from the device */
    SCIMapLocalSegmentForDevice(segment, adapter, device->device, &ioaddr, 0, size, 0, &err);
    if (err != SCI_ERR_OK) {
        sci_error_t tmp;
        SCISetSegmentUnavailable(segment, adapter, 0, &tmp);
        fprintf(stderr, "Failed to map local segment for device: %s\n",
                SCIGetErrorString(err));
        return EIO;
    }

    /* Map segment into local address space */
    ptr = SCIMapLocalSegment(segment, &map, 0, size, NULL, SCI_FLAG_READONLY_MAP, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to map local segment: %s\n",
                SCIGetErrorString(err));
        status = EIO;
        goto leave;
    }

    /* Prepare and submit identify command */
    prepare_command(&cmd, 0xcafe, NVME_IDENTIFY, 0, (uint64_t) ioaddr);
    cmd.dword[10] = 1;
    cmd.dword[11] = 0;

    fprintf(stdout, "Preparing command (id=%04x, opc=%02Xh)\n", 
            get_bits(cmd.dword[0], 31, 16),
            get_bits(cmd.dword[0], 7, 0));
    fprintf(stdout, "Executing command...");
    fflush(stdout);

    status = nvme_submit_command(&device->asq, &cmd);
    if (status != 0) {
        fprintf(stdout, "FAIL\n");
        fprintf(stderr, "Unexpected error: %s\n", strerror(status));
        goto unmap;
    }

    status = nvme_wait_completion(&device->acq, &cpl);
    sq_update(&device->asq);
    if (status != 0) {
        fprintf(stdout, "FAIL\n");
        fprintf(stderr, "Unexpected error: %s\n", strerror(status));
        goto unmap;
    }
    fprintf(stdout, "OK\n");

    /* Check status of completion */
    fprintf(stdout, "Command status (id=%04x): ", get_bits(cpl.dword[3], 15, 0));
    status = get_bits(cpl.dword[3], 31, 17);
    if (status != 0) {
        fprintf(stdout, "FAIL\ndnr=%u, m=%u, sct=%02Xh, sc=%02Xh\n",
                get_bits(cpl.dword[3], 31, 31),
                get_bits(cpl.dword[3], 30, 30),
                get_bits(cpl.dword[3], 27, 25),
                get_bits(cpl.dword[3], 24, 17));
        status = EIO;
        goto unmap;
    }
    fprintf(stdout, "OK\n");

    /* Show some fields from the identifier struct */
    show_controller_info(device, ptr);

unmap:
    SCIUnmapSegment(map, 0, &err);

leave:
    SCIUnmapLocalSegmentForDevice(segment, adapter, device->device, 0, &err);
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "Failed to unmap segment for device: %s\n",
                SCIGetErrorString(err));
    }

    do {
        SCISetSegmentUnavailable(segment, adapter, 0, &err);
    } while (err == SCI_ERR_BUSY);

    return status;
}



/*
 * Reset NVMe device and set up admin queues.
 */
static int reset_controller(struct nvme_device* device)
{
    volatile struct registers* ptr;
    size_t page_size;
    uint32_t timeout;
    uint16_t acqs;
    uint16_t asqs;
    uint8_t iocqes;
    uint8_t iosqes;

    ptr = device->mlbar.map_ptr;

    page_size = get_page_size(&device->mlbar);

    fprintf(stdout, "Resetting controller...");
    fflush(stdout);

    /* Reset queue descriptors */
    reset_queue(&device->asq);
    reset_queue(&device->acq);

    /* Get timeout in milliseconds */
    timeout = get_timeout(&device->mlbar);

    /* Set enable bit to 1 and wait for ready to transition from 1 to 0 */
    ptr->cc = ptr->cc & ~1;
    while ((ptr->csts & 1) != 0) {
        if (timeout == 0) {
            fprintf(stdout, "FAIL\n");
            return ETIME;
        }

        sleep_ms(100);
        timeout -= _MIN(timeout, 100);
    }

    /* Set admin queue attributes */
    acqs = (page_size / device->acq.es) - 1;
    asqs = (page_size / device->asq.es) - 1;
    ptr->aqa = bitsw(acqs, 27, 16) | bitsw(asqs, 11, 0);

    /* Set admin queue base addresses */
    ptr->acq = (uint64_t) device->acq.ioaddr;
    ptr->asq = (uint64_t) device->asq.ioaddr;

    iocqes = b2log(sizeof(struct completion));
    iosqes = b2log(sizeof(struct command));

    /* Update CC in one go and wait for ready to transition from 0 to 1 */
    ptr->cc = bitsw(iocqes, 23, 20) 
        | bitsw(iosqes, 19, 16) 
        | bitsw(b2log(page_size >> 12), 10, 7) /* MPS */
        | bitsw(0, 6, 4) /* CSS */
        | bitsw(1, 0, 0); /* Enable */

    timeout = get_timeout(&device->mlbar);
    while ((ptr->csts & 1) != 1) {
        if (timeout == 0) {
            fprintf(stdout, "FAIL\n");
            return ETIME;
        }

        sleep_ms(100);
        timeout -= _MIN(timeout, 100);
    }

    fprintf(stdout, "OK\n");
    return 0;
}



int main(int argc, char **argv) 
{
    int status;
    sci_error_t rc;
    sci_error_t tmp;
    sci_desc_t sd;                 /* SISCI virtual descriptor */
    struct nvme_device ctrl = {0}; /* NVMe controller handle */
    sci_local_segment_t segment;
    unsigned int adapter;
    sci_smartio_device_info_t info = {0};

    rc = SCI_ERR_OK;
    status = 0;

    SCIInitialize(0, &rc);
    if (rc != SCI_ERR_OK) {
        fprintf(stderr, "Failed to initialize SISCI API: %s\n", 
                SCIGetErrorString(rc));
        exit(-1);
    }

    SCIOpen(&sd, 0, &tmp);
    if (tmp != SCI_ERR_OK) {
        fprintf(stderr, "Failed to open SISCI API descriptor: %s\n",
                SCIGetErrorString(tmp));
        rc = tmp;
        goto terminate;
    }

    /* Find any NVMe drive discovered by SmartIO and initialize handles */
    status = get_device(&ctrl, sd);
    if (status != 0) {
        fprintf(stderr, "Failed to get controller handle: %s\n", 
                strerror(status));
        goto close_sd;
    }

    tmp = query_device(ctrl.device, &info);
    if (tmp != SCI_ERR_OK) {
        rc = tmp;
        goto release_device;
    }
    adapter = info.adapter;

    /* Create a segment that the device can DMA to and we can read from */
    rc = create_local_segment(&segment, sd, adapter, 0x1000);
    if (rc != SCI_ERR_OK) {
        fprintf(stderr, "Failed to create local segment: %s\n",
                SCIGetErrorString(rc));
        goto release_device;
    }

    /* Reset controller and set up queues */
    status = reset_controller(&ctrl);
    if (status != 0) {
        fprintf(stderr, "Failed to reset controller: %s\n", strerror(status));
        goto remove_segment;
    }

    /* Identify controller */
    status = identify_controller(&ctrl, adapter, segment);
    if (status != 0) {
        fprintf(stderr, "Failed to identify controller: %s\n", strerror(status));
        goto remove_segment;
    }

remove_segment:
    remove_local_segment(segment, adapter);

release_device:
    put_device(&ctrl);

close_sd:
    SCIClose(sd, 0, &tmp);

terminate:
    SCITerminate();

    exit(rc == SCI_ERR_OK && status == 0 ? 0 : -1);
}

