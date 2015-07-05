/*
 * Controller Area Network (CAN) infrastructure.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __LINUX_CAN_CAN_H
#define __LINUX_CAN_CAN_H


#define CAN_FRAME_MAX_DATA_LEN 8
#define CAN_FILTER_REG_NUM     6

#define CAN_IOCTL_COMMAND 	 0	/**< IOCTL command request */
#define CAN_IOCTL_CONFIG 	 1	/**< IOCTL configuration request */
#define CAN_IOCTL_SEND 		 2	/**< IOCTL request */
#define CAN_IOCTL_CONFIGURERTR 	 4	/**< IOCTL request */
#define CAN_IOCTL_STATUS         5      /**< IOCTL status request */

#define CAN_IOC_MAGIC	'C'
#define CAN_IOCTRESET			_IO(CAN_IOC_MAGIC, 0)			/* reset device */
#define CAN_IOCTWAKEUP			_IO(CAN_IOC_MAGIC, 1)			/* wakup device */
#define CAN_IOCTNORMALMODE		_IO(CAN_IOC_MAGIC, 2)			/* go normal mode */
#define CAN_IOCTLOOPBACKMODE	_IO(CAN_IOC_MAGIC, 3)			/* go loopback mode */
#define CAN_IOCTLISTENONLYMODE	_IO(CAN_IOC_MAGIC, 4)			/* go listen only mode */
#define CAN_IOCTSLEEPMODE		_IO(CAN_IOC_MAGIC, 5)			/* go sleep mode */
#define CAN_IOCSRATE			_IOW(CAN_IOC_MAGIC, 6, int)		/* set baud rate */
#define CAN_IOCSFILTER			_IOW(CAN_IOC_MAGIC, 7, long)	/* set receive filter */
#define CAN_IOCGRATE			_IOR(CAN_IOC_MAGIC, 8, int)		/* get baud rate */
#define CAN_IOCGFILTER			_IOR(CAN_IOC_MAGIC, 9, long)	/* get receive filter */

#define MSG_ACTIVE	(0)		/**< Controller Error Active */
#define MSG_BASE	(0)		/**< Base Frame Format */
#define MSG_RTR		(1<<0)		/**< RTR Message */
#define MSG_OVR		(1<<1)		/**< CAN controller Msg overflow error */
#define MSG_EXT		(1<<2)		/**< extended message format */
#define MSG_SELF	(1<<3)		/**< message received from own tx */
#define MSG_PASSIVE	(1<<4)		/**< controller in error passive */
#define MSG_BUSOFF      (1<<5)		/**< controller Bus Off  */
#define MSG_WARNING    	(1<<6)		/**< CAN Warning Level reached */
#define MSG_BOVR	(1<<7)		/**< receive/transmit buffer overflow */
#define CAN_SFF_MASK 0x000007FFU	/* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU	/* extended frame format (EFF) */
#define CANDRIVERERROR  0xFFFFFFFFul	/* invalid CAN ID == Error */


/**
 * struct can_frame_header - Extended CAN frame header fields
 * @id: 11 bit standard ID
 * @srr: Substitute Remote Request bit
 * @ide: Extended frame bit
 * @eid: 18 bit extended ID
 * @rtr: Remote Transmit Request bit
 * @rb1: Reserved Bit 1
 * @rb0: Reserved Bit 0
 * @dlc: 4 bit Data Length Code
 *
 * For a standard frame ensure that ide == 0 and fill in only id, rtr and dlc.
 *
 * srr, rb1, and rb0 are unused and should be set to zero.
 *
 * Note: The memory layout does not correspond to the on-the-wire format for a
 * CAN frame header.
 */
struct can_frame_header {
	unsigned id:11;
	unsigned srr:1;
	unsigned ide:1;
	unsigned eid:18;
	unsigned rtr:1;
	unsigned rb1:1;
	unsigned rb0:1;
	unsigned dlc:4;
};

/**
 * struct can_frame - CAN frame header fields and data
 * @header: CAN frame header
 * @data: 8 bytes of data
 *
 * User space transmits and receives struct can_frame's via the network device
 * socket interface.
 *
 * Note: The memory layout does not correspond to the on-the-wire format for a
 * CAN frame.
 */
struct can_frame {
	struct can_frame_header header;
	unsigned char data[CAN_FRAME_MAX_DATA_LEN];
};

/**
* The CAN message structure.
* Used for all data transfers between the application and the driver
* using read() or write().
*/
typedef struct {
    /** flags, indicating or controlling special message properties */
    int             flags;
    int             cob;	 /**< CAN object number, used in Full CAN  */
    unsigned   long id;		 /**< CAN message ID, 4 bytes  */
    struct timeval  timestamp;	 /**< time stamp for received messages */
    short      int  length;	 /**< number of bytes in the CAN message */
    unsigned   char data[CAN_FRAME_MAX_DATA_LEN]; /**< data, 0...8 bytes */
} canmsg_t;

/** 
 * mask/filter register table
 * MASK    FILTER    RECVID    RECEIVE
 *  0       x          x         yes
 *  1       0          0         yes
 *  1       0          1         no
 *  1       1          0         no
 *  1       1          1         yes
 */

/**
 * struct id_filter - CAN device receive filter setup
 * @id: filter of standard frame id.
 * @ide: enable extend frame id filter
 * @eid: filter of extend frame id or standard frame data(data-0 data-1)
 * @active: active this setup, if not all this struct fields will be zero
 */
struct id_filter {
	unsigned id:11;
	unsigned ide:1;
	unsigned eid:18;
	unsigned active:1;
};

/**
 * struct can_filter - CAN device receive filter setup
 * 
 * MCP2515 have 6 filter register RXF0 -- RXF5. 
 * It have 2 mask register,we set them to same RXM0=RXM1.
 *
 * @fid: see struct id_filter.
 * @sidmask: mask for standard frame id
 * @eidmask: if id_filter.ide=1,it mask for extend frame id. else it mask for standard frame data.
 * @mode: receive filter mode.
 *        0 - Receive all valid messages using either standard or extended identifiers that meet filter
 *        1 - Receive only valid messages with standard identifiers that meet filter criteria
 *        2 - Receive only valid messages with extended identifiers that meet filter criteria
 *        3 - Turn mask/filters off; receive any message
 */
struct can_filter {
	struct id_filter fid[CAN_FILTER_REG_NUM];
	unsigned sidmask:11;
	unsigned eidmask:18;
	unsigned mode:2;
};


#endif /* !__LINUX_CAN_CAN_H */
