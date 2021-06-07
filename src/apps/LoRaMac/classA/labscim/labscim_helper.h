/*
 * labscim_helper.h
 *
 *  Created on: 20 de ago de 2020
 *      Author: root
 */

#ifndef EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_
#define EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_

#include <stdint.h>
#include <stdarg.h>

uint64_t LabscimSignalRegister(uint8_t* signal_name);

void LabscimSignalEmit(uint64_t id, double value);

int labscim_printf(const char *fmt, ...);


#endif /* EXAMPLES_6TISCH_SIMPLE_NODE_LABSCIM_HELPER_H_ */
