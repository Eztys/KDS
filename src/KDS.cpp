#include "KDS.h"


/********************************************************************************
 *	Purpose	Class constructor													*
 * 	Input	-																	*
 * 	Output	-																	*
 ********************************************************************************/

KDS::KDS(){
	Serial.begin(BAUDRATE);
}

/********************************************************************************
 *	Purpose	Class destructor													*
 * 	Input	-																	*
 * 	Output	-																	*
 ********************************************************************************/

KDS::~KDS(){
	Serial.end();
}


/********************************************************************************
 *	Purpose	Sent request message to ECU											*
 * 	Input	Pointer to byte array request message								*
 * 	Output	true when request has been send, false otherwise					*
 ********************************************************************************/

uint8_t KDS::sendRequest(uint8_t *request) {

	uint16_t request_length = 0;
	uint8_t echo = 0;

	// Read length byte from request message
	// When format byte = 0x81, the length byte is 1
	if (request[0] == 0x81){
		request_length = 5;
	} else {
		request_length = request[3] + 5;
	}

  // Check if request is send in time
  if (checkTimingWindow(ISO_P3_MAX)){
    communication_error = REQUEST_ERROR_ISO_P3_MAX;
    return REQUEST_ERROR;
  }

	// Ensure the timing window between ECU response and current request
	// is maintained
	while (!checkTimingWindow(ISO_P3_MIN)){};

	for (uint8_t i = 0; i < request_length; i++) {

		// Write one byte
		Serial.write(request[i]);

		// Reset timing window after every byte has been send
		resetTimingWindow();

		// Manditory inter byte delay used to read echo byte
		while (!checkTimingWindow(ISO_P4_MIN)) {

			// At least one byte available in RX buffer
			if (Serial.available() > 0) {

				// Read one byte
				echo = Serial.read();

				// Request failed when echo is incorrect
				if (echo != request[i]) {

					// Unexpected echo
					communication_error = REQUEST_ERROR_ECHO;

					// Request failed
					return REQUEST_ERROR;
				}
			}
		}
	}

	// Request sucessfully sent
	return REQUEST_FINISHED;
}

/********************************************************************************
 *	Purpose	Listen for ECU's response message(s)								*
 * 	Input	Pointer to byte array buffer						 				*
 * 	Output	Response message stored in buffer and response status returned		*
 ********************************************************************************/

uint8_t KDS::getResponse(uint8_t *response) {

	uint8_t new_byte = 0;			// Newly read byte
	uint8_t index = 0;				// Index of received byte
	uint16_t response_length = MAX_RESPONSE_LENGTH;	// Number of data message bytes
	bool response_received = false;

	// Loop untill response is fully received, or an error has occured
	while (response_received == false)	{

		// At least one byte is available in RX buffer
		if (Serial.available() > 0) {

			// Reset timing window as soon as a new byte in RX buffer has been detected
			resetTimingWindow();

			// Store new byte in temporary variable
			new_byte = Serial.read();

			// Concenate new byte to response message buffer
			response[index] = new_byte;

      // Check function of new byte
			switch (index) {

				// Byte 1
				// Format byte
				case 0:

					// Format byte incorrect
					if (new_byte != FORMAT_BYTE) {
						communication_error = RESPONSE_ERROR_FORMAT_BYTE;
						return RESPONSE_ERROR;
					}

					break;

				// Byte 2
				// Target address (ECU)
				case 1:

					// Target address incorrect
					if (new_byte != TESTER_ADDRESS) {
						communication_error = RESPONSE_ERROR_TARGET_ADDRESS;
						return RESPONSE_ERROR;
					}

					break;

				// Byte 3
				// Source address (tester)
				case 2:

					// Source address incorrect
					if (new_byte != ECU_ADDRESS) {
						communication_error = RESPONSE_ERROR_SOURCE_ADDRESS;
						return RESPONSE_ERROR;
					}

					break;

				// Byte 4
				// Number of data bytes
				case 3:

					// Total message length = number of data bytes + header and checksum
					response_length = response[index] + 5;

          // Total message will not fit in response message buffer
          if (response_length > MAX_RESPONSE_LENGTH) {
            communication_error = RESPONSE_ERROR_BUFFER_OVERFLOW;
            return RESPONSE_ERROR;
          }

					break;

				// Byte 5+
				// Data bytes and checksum
				default:

					// Complete message has been received
					if (index == response_length - 1) {

						// Check if checksum is correct
						if (checksum(response, response_length - 1) != response[index]) {
							communication_error = RESPONSE_ERROR_CHECKSUM;
							return RESPONSE_ERROR;
						}

						// Finished reading response
						response_received = true;
					}

					break;

			}

			// Response byte has been stored
			// No error has occured yet
			// Move to next byte
			index++;

		} else {

			// No bytes available in RX buffer
			// Check for ISO_P2_MAX timeout (Request-response or response-response)
			if (checkTimingWindow(ISO_P2_MAX)) {
				communication_error = RESPONSE_ERROR_ISO_P2_MAX_TIMEOUT;
				return RESPONSE_ERROR;
			}
		}
	}

	// One complete response has been received.
	// ECU may send more responses within ISO_P2_MAX window.

	// Open timing window of P2_MAX
	while (!checkTimingWindow(ISO_P2_MAX)) {

		// Data is available in RX buffer: a new response is pending
		if (Serial.available() > 0) {
			return RESPONSE_PENDING;
		}

	}

	// One respons message has been received
	// No response is pending
	return RESPONSE_FINISHED;
}


/********************************************************************************
 *	Purpose	Initialize communication with ECU									*
 * 	Input	-																	*
 * 	Output	true when communication line has been established, false otherwise	*
 ********************************************************************************/

bool KDS::initializeECU(){

	uint8_t start_communication[] = {FORMAT_BYTE+1, ECU_ADDRESS, TESTER_ADDRESS, 0x81, 0x00};
	start_communication[4] = checksum(start_communication,4);

	uint8_t start_diagnostic_mode[] = {FORMAT_BYTE, ECU_ADDRESS, TESTER_ADDRESS, 0x02, 0x10, 0x80, 0x00};
	start_diagnostic_mode[6] = checksum(start_diagnostic_mode,6);

	uint8_t response_buffer[10];
	uint8_t response_status;

	/************************************
	 * 	RESET TIMING WINDOW				*
	 ************************************/

	// Reset the timing window prior to fast init
	// This ensures that the request will be send immediately after the fast initialization sequence
	// Otherwise the request function will wait for at least ISO_P3_MIN (55 ms normally) and ECU will not initiate
	resetTimingWindow();

	/************************************
	 * 	FAST INIT SEQUENCE				*
	 ************************************/

	// End serial Port
	Serial.end();

	// Start fast init sequence
	pinMode(K_OUT, OUTPUT);
	digitalWrite(K_OUT, HIGH);

	// Idle communication to create correct initial conditions
	delay(350);
	digitalWrite(K_OUT, LOW);
	delay(25);
	digitalWrite(K_OUT, HIGH);
	delay(25);

	// Start serial port
	Serial.begin(BAUDRATE);

	/************************************
	 * 	START COMMUNICATION				*
	 ************************************/

	// Send start communication request
	if (!sendRequest(start_communication)) return false;

	// Check if response has been correctly received
	if (getResponse(response_buffer) != RESPONSE_FINISHED) return false;

	// Check if response was positive
	if (!checkPositiveResponse(start_communication, response_buffer)) return false;


	/************************************
	 * 	START STANDARD DIAGNOSTIC MODE	*
	 ************************************/

	// Send start diagnostic mode request
	if(!sendRequest(start_diagnostic_mode)) return false;

	// Check if response has been correctly received
	if (getResponse(response_buffer) != RESPONSE_FINISHED) return false;

	// Check if response was positive
	if (!checkPositiveResponse(start_diagnostic_mode, response_buffer)) return false;

	// Initialization successful
	return true;
}

/********************************************************************************
 *	Purpose	Reset millisecond timer												*
 * 	Input	-																	*
 * 	Output	-																	*
 ********************************************************************************/

void KDS::resetTimingWindow(){
	// Store current micros timer
	_start_time = micros();
}

/********************************************************************************
 *	Purpose	Check if user desired time has passed since timer reset				*
 * 	Input	time in milliseconds												*
 * 	Output	true when time has passed, false otherwise							*
 ********************************************************************************/

bool KDS::checkTimingWindow(uint16_t time){

	// Calculate elapsed time since timer was reset
	// Testing MICROS for improved speed (i.e. us accuracy vs ms accuracy)
	_elapsed_time = micros() - _start_time;

	// Crucial to ensure that elapsed time is GREATER THAN (>) the desired timing window
	// When GREATER OR EQUAL TO (>=) is used, requests are send too soon.
  // Convert input time from milliseconds to microseconds
	if (_elapsed_time > time*1000UL) {
		return true;
	} else {
		return false;
	}
}

/********************************************************************************
 *	Purpose	Calculat 8-bit checksum of request or response message				*
 * 	Input	Message byte array and its length, excluding checksum byte			*
 * 	Output	8-bit checksum														*
 ********************************************************************************/

uint8_t KDS::checksum(uint8_t *message, uint16_t len){

	uint8_t sum = 0;

	// Loop over all message bytes
	for (uint16_t i = 0; i < len; i++) {
		sum += message[i];
	}

	return sum;
}


/********************************************************************************
 *	Purpose	Check if response was positive										*
 * 	Input	send request message and received response message					*
 * 	Output	true when response service id is positive, false otherwise			*
 ********************************************************************************/

bool KDS::checkPositiveResponse(uint8_t *request, uint8_t *response){

	uint8_t request_ServiceId = 0;

	// When format byte is 0x81, there is no length byte and serviceID is at index 3 instead of index 4
	if (request[0] == 0x81) {
		request_ServiceId = request[3];
	} else {
		request_ServiceId = request[4];
	}

	// Positive Response Service Id == Request Service Id + 0x40
	if (response[4] == (request_ServiceId + 0x40)) {
		return true;
	} else {
		return false;
	}

}
