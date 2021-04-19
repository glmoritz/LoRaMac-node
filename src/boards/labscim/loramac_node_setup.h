/*
 * labscim_contiking_setup.h
 *
 *  Created on: 5 de jun de 2020
 *      Author: root
 */

#ifndef LABSCIM_LORAMAC_SETUP_H_
#define LABSCIM_LORAMAC_SETUP_H_


struct loramac_node_setup
{
    uint8_t mac_addr[8];
    uint64_t startup_time;
    uint64_t IsMaster;
    uint8_t output_logs;
}__attribute__((packed));


#endif /* LABSCIM_LORAMAC_SETUP_H_ */

