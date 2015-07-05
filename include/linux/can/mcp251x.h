/*
 * MCP251x CAN controller driver
 *
 * Copyright (C) 2006 Arcom Control Systems Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __LINUX_CAN_MCP251X_H
#define __LINUX_CAN_MCP251X_H

/**
 * struct mpc251x - MCP251x CAN controller platform data
 *
 * f_osc: input clock frequency in Hz
 * transceiver_enable: enable/disable CAN bus transceivers.  May be NULL if
 *     the transceivers are always enabled.
 */
struct mcp251x_platform_data {
	int f_osc;
	void (*transceiver_enable)(int enable);
	void (*platform_init)(void);
};

#endif /* !__LINUX_CAN_MCP251X_H */
