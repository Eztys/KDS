#ifndef KDS_h
#define KDS_h
#include "Arduino.h"

#define RESPONSE_FINISHED 0
#define	RESPONSE_PENDING 1
#define	RESPONSE_ERROR 2

#define REQUEST_FINISHED 3
#define REQUEST_ERROR 4

#define	RESPONSE_ERROR_ISO_P2_MAX_TIMEOUT 5
#define	RESPONSE_ERROR_BUFFER_OVERFLOW 6
#define	RESPONSE_ERROR_FORMAT_BYTE 7
#define	RESPONSE_ERROR_TARGET_ADDRESS 8
#define	RESPONSE_ERROR_SOURCE_ADDRESS 9
#define	RESPONSE_ERROR_CHECKSUM 10
#define REQUEST_ERROR_ECHO 11

#define INIT_FINISHED 12
#define INIT_ERROR 13

#define INIT_ERROR_STARTCOMM_REQ 14
#define INIT_ERROR_STARTCOMM_RES 15
#define INIT_ERROR_STARTCOMM_NEGRES 16

#define INIT_ERROR_STARTDIAG_REQ 17
#define INIT_ERROR_STARTDIAG_RES 18
#define INIT_ERROR_STARTDIAG_NEGREQ 19

class KDS
{
	public:

	KDS();
	~KDS();
	uint8_t initializeECU();
	uint8_t sendRequest(uint8_t *request);
	uint8_t getResponse(uint8_t *response);
	uint8_t communication_error;

	private:

	uint8_t checksum(uint8_t *message, uint16_t len);
	void resetTimingWindow();
	bool checkTimingWindow(uint16_t time);
	bool checkPositiveResponse(uint8_t *request, uint8_t *response);


	const uint8_t K_IN = 0;
	const uint8_t K_OUT = 1;
	const uint16_t BAUDRATE = 10400;

	const uint8_t ISO_P1_MIN = 5;
	const uint8_t ISO_P1_MAX = 20;
	const uint8_t ISO_P2_MIN = 25;
	const uint16_t ISO_P2_MAX = 50;
	const uint8_t ISO_P3_MIN = 55;
	const uint16_t ISO_P3_MAX = 5000;
	const uint8_t ISO_P4_MIN = 5;
	const uint8_t ISO_P4_MAX = 20;

	const uint8_t ECU_ADDRESS = 0x11;
	const uint8_t TESTER_ADDRESS = 0xF1;
	const uint8_t FORMAT_BYTE = 0x80;

	uint32_t _start_time = 0;
	uint32_t _elapsed_time = 0;


};



#endif
