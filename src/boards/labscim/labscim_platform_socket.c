//labscim related structures
#include <stdarg.h>
#include "labscim_linked_list.h"
#include "labscim_protocol.h"
#include "labscim_socket.h"
#include "labscim_log_levels.h"
#include "labscim_loramac_setup.h"

#define SERVER_PORT (9608)
#define SERVER_ADDRESS "127.0.0.1"
#define SOCK_BUFFER_SIZE (512)

struct labscim_ll gCommands;

buffer_circ_t *gNodeInputBuffer;
buffer_circ_t *gNodeOutputBuffer;
uint8_t* gNodeName;
uint8_t* gServerAddress;
uint64_t gServerPort;
uint64_t gBufferSize;
uint32_t gBootReceived=0;
uint32_t gProcessing=0;
uint64_t gTimeReference=0;

uint8_t mac_addr[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

extern void labscim_signal_arrived(struct labscim_signal* sig);

uint8_t gAPP_KEY[32];

#define DBG_PRINT_BUFFER_SIZE (256)
uint8_t gByteBuffer[DBG_PRINT_BUFFER_SIZE];

extern void radio_irq( void* context );


void labscim_protocol_boot(struct labscim_protocol_boot* msg)
{
	struct loramac_node_setup* cns = (struct loramac_node_setup*)msg->message;
	memcpy((void*)mac_addr,(void*)cns->mac_addr,sizeof(uint8_t)*8);
	labscim_set_time(cns->startup_time);
	gBootReceived = 1;    
    gTimeReference = cns->TimeReference;
    memcpy(gAPP_KEY,cns->AppKey,32);    
	free(msg);
	return;
}


void
platform_process_args(int argc, char**argv)
{
	uint64_t c;

	while ((c = getopt (argc, argv, "a:p:b:n:")) != -1)
		switch (c)
		{
		case 'a':
			gServerAddress = optarg;
			break;
		case 'b':
			if(optarg!=NULL)
			{
				gBufferSize = atoi(optarg);
			}
			else
			{
				gBufferSize = SOCK_BUFFER_SIZE;
			}
			break;
		case 'p':
			if(optarg!=NULL)
			{
				gServerPort = atoi(optarg);
			}
			else
			{
				gServerPort = SERVER_PORT;
			}
			break;
		case 'n':
			gNodeName = optarg;
			break;        
		case '?':
			fprintf (stderr,"Unknown option character `\\x%x'.\n",	optopt);
		}
}




void socket_process_command(struct labscim_protocol_header *hdr)
{
#ifdef LABSCIM_LOG_COMMANDS
    char log[128];
#endif
    switch (hdr->labscim_protocol_code)
    {
    case LABSCIM_PROTOCOL_BOOT:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tPROTOCOL_BOOT\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        labscim_protocol_boot((struct labscim_protocol_boot *)(hdr));
        break;
    }
    case LABSCIM_TIME_EVENT:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tTIME_EVENT\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        labscim_time_event((struct labscim_time_event *)(hdr));
        break;
    }
    case LABSCIM_RADIO_RESPONSE:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tRADIO_RESPONSE\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        labscim_set_time(((struct labscim_radio_response *)(hdr))->current_time);
        labscim_radio_incoming_command((struct labscim_radio_response *)(hdr));
        radio_irq(NULL);
        break;
    }
    case LABSCIM_SIGNAL:
    {
        labscim_set_time(((struct labscim_signal *)(hdr))->current_time);
        labscim_signal_arrived((struct labsim_signal *)(hdr));
        break;
    }
    case LABSCIM_END:
    {
#ifdef LABSCIM_LOG_COMMANDS
        sprintf(log, "seq%4d\tEND\n", hdr->sequence_number);
        labscim_log(log, "pro ");
#endif
        exit(0);
        break;
    }
    default:
    {
        perror("Unhandled Labscim Command\n");
        free(hdr);
    }
    }
}

void socket_process_all_commands()
{
    void *cmd;
    //process returned commands (if any)
    do
    {
        cmd = labscim_ll_pop_front(&gCommands);
        if (cmd != NULL)
        {
            struct labscim_protocol_header *hdr = (struct labscim_protocol_header *)cmd;
            gProcessing = 1;
            socket_process_command(hdr);
        }
    } while (cmd != NULL);
}

void *socket_pop_command(uint32_t command, uint32_t sequence_number)
{
    void *cmd;
    struct labscim_ll_node *it = gCommands.head;
    do
    {
        if (it != NULL)
        {
            struct labscim_protocol_header *hdr = (struct labscim_protocol_header *)it->data;
            if (((hdr->labscim_protocol_code == command) && (sequence_number == 0)) || (hdr->request_sequence_number == sequence_number) || ((command == 0) && (sequence_number == 0)))
            {
                cmd = labscim_ll_pop_node(&gCommands, it);
                return cmd;
            }
            it = it->next;
        }

    } while (it != NULL);
    return NULL;
}

void* socket_wait_for_command(uint32_t command, uint32_t sequence_number)
{
    void *cmd = NULL;
    cmd = socket_pop_command(command, sequence_number);
    while (cmd == NULL)
    {
        //shared memory communication
        pthread_mutex_lock(gNodeInputBuffer->mutex.mutex);
        labscim_socket_handle_input(gNodeInputBuffer, &gCommands);

        while (gCommands.count == 0)
        {
            pthread_cond_wait(gNodeInputBuffer->mutex.more, gNodeInputBuffer->mutex.mutex);
            labscim_socket_handle_input(gNodeInputBuffer, &gCommands);
        }
        cmd = socket_pop_command(command, sequence_number);
        pthread_cond_signal(gNodeInputBuffer->mutex.less);
        pthread_mutex_unlock(gNodeInputBuffer->mutex.mutex);
    }
    return cmd;
}