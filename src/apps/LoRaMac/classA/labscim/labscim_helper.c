#include "labscim_socket.h"
#include <stdarg.h>
extern buffer_circ_t* gNodeOutputBuffer;
void socket_process_command(struct labscim_protocol_header* hdr);
void* socket_wait_for_command(uint32_t command, uint32_t sequence_number);

char gBuffer[256];
int labscim_printf(const char *fmt, ...)
{
	#define LOGLEVEL_INFO (3)    
    va_list args;
    va_start(args, fmt);
    int rc = vsnprintf(gBuffer, sizeof(gBuffer), fmt, args);	
	if(rc>sizeof(gBuffer))
	{
		strcpy(gBuffer+strlen(gBuffer)-4,"...");		
		rc = sizeof(gBuffer);
	}
	gBuffer[sizeof(gBuffer)-1]=0;
    printf("%s",gBuffer);
	print_message(gNodeOutputBuffer,LOGLEVEL_INFO,gBuffer,strlen(gBuffer)+1);
	va_end(args);
    return rc;
}


uint64_t LabscimSignalRegister(uint8_t* signal_name)
{
	uint64_t ret = 0;
	struct labscim_protocol_header* resp;
	uint32_t sequence_number = signal_register(gNodeOutputBuffer, signal_name);
	do{
		resp =  (struct labscim_protocol_header*)socket_wait_for_command(0, 0);
		if(resp->request_sequence_number == sequence_number)
		{
			ret =  ((struct labscim_signal_register_response*)resp)->signal_id;
			free(resp);
			break;
		}
		else
		{
			socket_process_command(resp);
		}
	}while(1); //ugly?
	return ret;
}

void LabscimSignalEmitDouble(uint64_t id, double value)
{
	signal_emit_double(gNodeOutputBuffer, id, value);
}

void LabscimSignalEmitChar(uint64_t id, char* value, uint64_t size)
{
	signal_emit_char(gNodeOutputBuffer, id, value,size);
}

void LabscimSignalSubscribe(uint64_t id)
{
	signal_subscribe(gNodeOutputBuffer,id);	
}


double LabscimExponentialRandomVariable(double mean) //mean is 1/lambda where lambda is the arrival rate (messages/second)
{
	float ret = 0;
	struct labscim_protocol_header* resp;
	union random_number parameter;
	union random_number unused;

	parameter.double_number = mean;
	unused.double_number = 0;

	uint32_t sequence_number = get_random(gNodeOutputBuffer, 1 /*exponential*/, parameter, unused, unused);
	do{
		resp =  (struct labscim_protocol_header*)socket_wait_for_command(0, 0);
		if(resp->request_sequence_number == sequence_number)
		{
			ret =  ((struct labscim_signal_get_random_response*)resp)->result.double_number;
			free(resp);
			break;
		}
		else
		{
			socket_process_command(resp);
		}
	}while(1); //ugly?
	return ret;
}