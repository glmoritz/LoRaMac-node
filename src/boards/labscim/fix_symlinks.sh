#!/bin/bash
rm labscim_linked_list.c 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/labscim_linked_list.c
rm labscim_linked_list.h 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/labscim_linked_list.h
rm labscim_loramac_setup.h 
ln -s $HOME/LabSCim/labscim/src/common/labscim_loramac_setup.h
rm labscim_protocol.h 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/labscim_protocol.h
rm labscim_socket.c 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/labscim_socket.c
rm labscim_socket.h 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/labscim_socket.h
rm omnet_radio_mode.h 
ln -s $HOME/LabSCim/labscim/src/common/omnet_radio_mode.h
rm shared_mutex.c 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/shared_mutex.c
rm shared_mutex.h 
ln -s $HOME/LabSCim/contiki-ng/arch/platform/labscim/shared_mutex.h
