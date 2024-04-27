// ============================================================================================================================= //
#include <SoftwareSerial.h>
#include <util/crc16.h>             // Used for confirmation of SOFTUART test config data.

#define SOFTUART0_TX_PIN     9      // Actual DUT channel
#define SOFTUART0_RX_PIN     8      // Test configuration data
#define SOFTUART0_RTS_PIN    2      // Used for handshaking; master signals when its alright to send more data. These must also be swapped in the harnessing as interrupt resources are limited and both master and slave would like to use pin 2 for generating processor interrupts
#define SOFTUART0_CTS_PIN    4      // Used for handshaking, signifies that slave has sent new data
#define SOFTUART0_BAUD       9600   // This baud rate must be in agreement with the NANO

#define HEARTBEAT_PIN        13
#define MAX_MESSAGE_LENGTH   63 + 1 // Allow up to 64 bytes in a serial transaction (this limit holds for both configuration data and test data)

#define NUM_CFG_OPTS         6      // 6 true options + crc trails the final delimiter ('\')

#define LL_CHECK_PIN(PIN_NO)            (PIND & (1 << (PIN_NO)))  // Read ATMEGA PINx register bit location and return value; works for input and output GPIO
#define LL_DRIVE_PIN(PIN_NO, PIN_LEVEL) (PIN_LEVEL == HIGH) ? (PORTD |= (1 << (PIN_NO))) : (PORTD &= ~(1 << (PIN_NO))) // Set level via ATMEGA PORTx register, handle low and high appropriately via ternary

// ============================================================================================================================= //

// Setup a new software serial object for the test
SoftwareSerial SoftSerial(SOFTUART0_RX_PIN, SOFTUART0_TX_PIN);

char buff[33]; // Must be in agreement with master... Master specifies max length of 256 bytes == ((255 + null) chars )/ 8 == MAX 32 CHAR ASCII STRINGS
uint32_t count = 0;
char newTest_buff[MAX_MESSAGE_LENGTH];
char newTest_buff_no_crc[MAX_MESSAGE_LENGTH];
char rx_buff[MAX_MESSAGE_LENGTH / 4];
char * newTest_cfg_buff[NUM_CFG_OPTS + 1];
uint16_t localCRC = 0;
uint16_t rxCRC = 0;
const bool VERBOSE = true;

const char STOP_CHAR = '\n';
const char DELIM_CHAR = '/';

const char * COMMON_DATA_FORMAT_REMINDER = "Ensure your configuration data is formatted as follows:\n";
const char * COMMON_DATA_FORMAT_EXAMPLE = "duration/data_len_bytes/count_mode/data/baud/transaction_delay_ms/CRC\n";
const char * COMMON_CRC_ERROR_MSG = "Please check that your CRC polynomial corresponds to 'A001'.\n";
const char * COMMON_CRC_ERROR_HANDLE = "CRC_ERROR: ";
const uint8_t COMMON_CRC_ERROR_HANDLE_WIDTH = strlen(COMMON_CRC_ERROR_HANDLE);

// Slave has no job characterizing errors; it does what it is told to do until the master signifies that the test is over
typedef enum {
  BAUD_SLOW   = 9600,
  BAUD_MEDIUM = 57600,
  BAUD_FAST   = 115200
} baud_t;

typedef enum {
  SLOW   = 0,
  MEDIUM = 1,
  FAST   = 2
} baud_cfg_t;

// Rcv formatted from MATLAB, pass to slave at beginning of test. use data locally to stop test
struct uart_test_cfg_t {
  uint16_t duration;              // Duration of test in transaction counts (incremented with validation of handshaking signals, as we cannot rely on bytes coming in)
  uint8_t data_len_bytes;         // Length of transaction data for test in bytes
  bool count_mode;                // Omit user supplied data and instead use integer values representing a running transaction count for the transaction data
  uint8_t * data;                 // User supplied transaction data
  baud_t baud;                    // Baud rate for test. Requires local reconfiguration on both slave and master end. For SoftUART on the Arduino Nano (ATMega328P), limited to 115200 maximum baud rate.
  uint16_t transaction_delay_ms;  // Delay (ms) between transactions for test. Set to 0 for fastest possible transaction cycles.
};

uart_test_cfg_t newTest;

typedef enum {
  TEST_WAITING,
  TEST_READY,
  TEST_START,
  TEST_RUNNING,
  TEST_DONE
} test_state_t;

test_state_t newTestState = TEST_WAITING;

volatile bool interruptOccurred = false;

// Interrupt Service Routine (ISR) for external interrupt (from RTS)
void handleInterrupt() {
  interruptOccurred = true; // Set the volatile interrupt flag; main loop will acknowledge via a poll
}

// ============================================================================================================================= //
void setup() {

  Serial.begin(115200); // HW Serial strictly for debugging
  
  // Heartbeat
  pinMode(HEARTBEAT_PIN, OUTPUT);
  LL_DRIVE_PIN(HEARTBEAT_PIN, LOW);

  // Handshaking signals
  pinMode(SOFTUART0_CTS_PIN, OUTPUT); // To master
  signifySoftUART_NOT_DONE();          // Upon reset, signify that softUART is not ready yet
  
  pinMode(SOFTUART0_RTS_PIN, INPUT); // From master
  attachInterrupt(digitalPinToInterrupt(SOFTUART0_RTS_PIN), handleInterrupt, RISING); // Attach interrupt for SOFTUART0_CTS_PIN
  sei(); // enable interupts globally on the chip

  initializeConfig(&newTest);

  // UART2 Link to Slave NANO
  SoftSerial.begin(SOFTUART0_BAUD);
  //delay(1);
}

// ============================================================================================================================= //
void loop() {
  
  // Wait for configuration data on UART RX from master, validate with CRC, then reinit soft serial if a baud rate change has occurred
  while(newTestState == TEST_WAITING) {
    Serial.println("Waiting for config data...");
    while (!interruptOccurred) {}; // Wait for an interrupt to occur from the master, signifying it has completed the transmission of a config struct
    interruptOccurred = false; //Rst the interrupt flag immediately
    if (SoftSerial.available()) {
      app_serial_read(newTest_buff, SoftSerial.available());

      if (VERBOSE) {
        Serial.print("RX Data: ");
        Serial.println(newTest_buff);
        Serial.println();
      }

      popLastData(newTest_buff, newTest_buff_no_crc, DELIM_CHAR, rx_buff);
      localCRC = computeCRC(newTest_buff_no_crc, strlen(newTest_buff_no_crc)); // NEED TO RIP OFF LAST TRAILING DAT APAST DELIM CHAR
      rxCRC = uint16_t(cStrToUint(rx_buff, 16));
      
      // Check transmitted CRC against local computation. If disagreement, request a new set of test configuration data from master by not toggling the interrupt. Data verification for valid integer values has already been performed
      if (localCRC == rxCRC) {
        parseDelimCstr(newTest_buff_no_crc, DELIM_CHAR, newTest_cfg_buff, NUM_CFG_OPTS); // Split transaction string at delimiters
        
        if (VERBOSE) {
          for(int n = 0; n < NUM_CFG_OPTS; n++) { 
            Serial.print(n + 1);
            Serial.print(": ");
            Serial.println(newTest_cfg_buff[n]);
          }
          Serial.println();
        }

        uint32_t duration = cStrToUint(newTest_cfg_buff[0], 10); // Check duration
        newTest.duration = duration;

        /*
        uint8_t data_len_bytes = cStrToUint(newTest_cfg_buff[1], 10);
        newTest.data_len_bytes = data_len_bytes;
        size_t sizeX = strlen(newTest_cfg_buff[3]);
        uint8_t buff2[sizeof(newTest_cfg_buff[3])];
        cStrToUintArray(newTest_cfg_buff[3], buff2, sizeX);
        newTest.data = buff2;
        */
        //newTest.data[] = {10, 5, 10, 5, 10, 5, 10, 5};
        uint8_t data_len_bytes = cStrToUint(newTest_cfg_buff[1], 10);
        newTest.data_len_bytes = data_len_bytes;
        
        size_t sizeX = strlen(newTest_cfg_buff[3]);
        uint8_t* buff2 = (uint8_t*)malloc(sizeX); // Allocate memory dynamically
        if (buff2 != NULL) {
            cStrToUintArray(newTest_cfg_buff[3], buff2, sizeX); // Copy the data into the dynamically allocated buffer
            newTest.data = buff2; // Assign the pointer to newTest.data
        } else {
            // Failed to allocate memory, handle the error
            Serial.println("Error: Failed to allocate memory for newTest.data");
        }

        uint8_t count_mode =  cStrToUint(newTest_cfg_buff[2], 10);
        newTest.count_mode = count_mode;
       
        uint32_t baud = cStrToUint(newTest_cfg_buff[4], 10);   // Check baud rate
        newTest.baud = (baud_t)baud;
        
        if (baud != SOFTUART0_BAUD) {                         // Temporarily disable interrupts
          if(VERBOSE) {
            Serial.println("Baud change detected!");
            Serial.print("Default : ");
            Serial.println(SOFTUART0_BAUD);
            Serial.print("New     : ");
            Serial.println(baud);
            Serial.println();  
          } 
          SoftSerial.begin((uint32_t)newTest.baud);            // Reconfigure the slave at the new requested baud rate
          Serial.println("DUT baud rate updated.\n");
        }

        uint16_t transaction_delay_ms = cStrToUint(newTest_cfg_buff[5], 10); // Check transaction delay
        newTest.transaction_delay_ms  = transaction_delay_ms;

        // Data is (very likely) valid by virtue of checksum alone; has been validated on master end prior to sending
        newTestState = TEST_READY;
        printTestConfig(&newTest);                  // Print out the local copy of the test config for local verification
        signifySoftUART_DONE();                     // Kick an interrupt back to the master indicating that configuration is complete 
        signifySoftUART_NOT_DONE();
        Serial.println("Ready to begin test.");
        break;
        //delay(3000);
      }

      // CRC FAILED
      else {
         Serial.print(COMMON_CRC_ERROR_HANDLE);
         Serial.println("Mismatch occurred between CRC computations!");
         app_serial_buffer_console(COMMON_CRC_ERROR_HANDLE_WIDTH, ' ');
         Serial.print("Local CRC16: ");
         Serial.println(localCRC, HEX);
         app_serial_buffer_console(COMMON_CRC_ERROR_HANDLE_WIDTH, ' ');
         Serial.print("RX CRC16: ");
         Serial.println(rxCRC, HEX);
         app_serial_buffer_console(COMMON_CRC_ERROR_HANDLE_WIDTH, ' ');
         Serial.println(COMMON_CRC_ERROR_MSG);
         Serial.println(COMMON_DATA_FORMAT_REMINDER);
         Serial.println(COMMON_DATA_FORMAT_EXAMPLE);
      }
    }
    else {
      Serial.println("No bytes received.\n");
    }
  }
  // When done, signal to master via UART_done that we're initialized for the test, at which point, the master will initiate the test by generating an interrupt on the RTS input pin
  //sprintf(buff, "%d", count); // will use hardcoded data or a running count for the data
  // Wait for master to signify we are ready to send more data by driving this pin high
  //interruptOccurred = false;
  
  while (newTestState == TEST_RUNNING || newTestState == TEST_READY) {
    signifySoftUART_DONE();
    while (!interruptOccurred) {}; // wait for an interrupt to occur
    signifySoftUART_NOT_DONE();
    interruptOccurred = false;  // rst the isr flag
    // This grouping of print statements does not transmit the null char. Each data value is terminated with LF (0xA)
    //SoftSerial.print("XXXXXXXX");
    //SoftSerial.print("1234567");
    //for (size_t i = 0; i < newTest.data_len_bytes; i++) {
    //SoftSerial.write(newTest.data, newTest.data_len_bytes - 1);
    //}
    //SoftSerial.print('\n'); // necessary in current build to send newline
    app_serial_print_bytes(newTest.data, newTest.data_len_bytes);
    toggleHeartbeat();
    //count++;
    //sprintf(buff, "%d", count); // Update the count value 
    delay(newTest.transaction_delay_ms);
    //signifySoftUART_DONE();
  }
}

// ============================================================================================================================= //
// Function to compute CRC-16 checksum for a C string (for verifying test cfg data from MATLAB, and during test setup amongst master->slave
uint16_t computeCRC(const char * data, size_t data_len) {
    uint16_t crc = 0xFFFF; // Initial value

    for (size_t i = 0; i < data_len; i++) {
        crc = _crc16_update(crc, data[i]); // Update CRC with next byte
    }

    return crc;
}

// ============================================================================================================================= //
void popLastData(char *cStr, char *cStr_buff, char delim, char *poppedData) {
    int len = strlen(cStr);
    int i = len - 1;
    strncpy(cStr_buff, cStr, len);

    // Find the position to split the string
    while (i >= 0 && cStr_buff[i] != delim) {
        i--;
    }
    
    // If delimiter found, extract the data after delimiter
    if (i >= 0) {
        strcpy(poppedData, cStr_buff + i + 1); // Copy the popped data into the poppedData buffer
        cStr_buff[i] = '\0'; // Null-terminate cStr after the delimiter to remove the popped data and delimiter
    } else {
        poppedData[0] = '\0'; // No delimiter found, so no data popped
    }
}

// ============================================================================================================================= //
void parseDelimCstr(char *cStr, char delim, char *buff[], int buffSize) {
    // This function has been adapted to avoid the use of strtok, which modifies the string it is passed by reference
    int index = 0;       // Index for the buffer array
    int tokenStart = 0;  // Index to mark the start of the current token

    // Loop through the string character by character
    for (int i = 0; cStr[i] != '\0'; ++i) {
        // If the current character is the delimiter or the end of the string
        if (cStr[i] == delim || cStr[i + 1] == '\0') {
            // Allocate memory for the token
            int tokenLength = i - tokenStart + (cStr[i] == delim ? 0 : 1);   // Exclude delimiter if present
            buff[index] = (char *)malloc((tokenLength + 1) * sizeof(char));  // Include space for null terminator

            // Copy the token into the buffer array
            strncpy(buff[index], cStr + tokenStart, tokenLength);  // Copy the token without the delimiter
            buff[index][tokenLength] = '\0';  // Add null terminator

            // Move to the next index in the buffer array
            ++index;

            // Update the start index of the next token
            tokenStart = i + 1;

            // Break if the buffer array is full
            if (index >= buffSize) {
                break;
            }
        }
    }

    // Ensure that the buffer is null-terminated
    buff[index] = NULL;
}

// ============================================================================================================================= //
// Polling option for verifying the SOFTUART RTS signal
bool checkSoftUART_ready(void) {
  return LL_CHECK_PIN(SOFTUART0_RTS_PIN);
}

// ============================================================================================================================= //
// These functions must be called in pairs!
bool signifySoftUART_DONE(void) {
  if (not LL_CHECK_PIN(SOFTUART0_CTS_PIN)) {
    LL_DRIVE_PIN(SOFTUART0_CTS_PIN, HIGH);
    return true;
  }
  else return false;
}

// ============================================================================================================================= //
bool signifySoftUART_NOT_DONE(void) {
  if (LL_CHECK_PIN(SOFTUART0_CTS_PIN)) {
    LL_DRIVE_PIN(SOFTUART0_CTS_PIN, LOW);
    return true;
  }
  else return false;
}

// ============================================================================================================================= //
uint32_t cStrToUint(const char *input_str, uint8_t base) {
    // Convert the input string to an integer using the strtol function
    return strtol(input_str, NULL, base);
}


// ============================================================================================================================= //
void cStrToUintArray(const char * input_str, uint8_t * ascii_array, size_t array_size) { // For converting ascii (C) strings to raw byte integers
    for (size_t i = 0; input_str[i] != '\0' && i < array_size; i++) {
        ascii_array[i] = (uint8_t)input_str[i]; // Directly assign ASCII value
    }
}

// ============================================================================================================================= //
void uIntArrayToCStr(uint8_t * input_data, char * output_ascii, size_t data_size) { // For converting raw byte integers to ascii (C) strings
    static char temp[sizeof(uint32_t) + 1]; // Temporary buffer for ASCII representation. Allow for storage of up to one 32bit number. Static in order to not create a bunch of unmanaged arrays
    for (size_t i = 0; i < data_size; i++) {
        temp[i] = (char)(input_data[i] + '0');  // Convert uint8_t to ASCII
    }
    temp[data_size] = '\0'; // Null-terminate the string
    //strncpy(temp, output_ascii, strlen(temp)); // Update the const char *
    //output_ascii = temp;
    strcpy(output_ascii, temp);
}

// ============================================================================================================================= //
void toggleHeartbeat(void) {
  static bool firstCall = true;
  static bool heartbeatState;
  if (firstCall) {
    heartbeatState = LL_CHECK_PIN(HEARTBEAT_PIN); // Get the initial state, and flip it prior to driving the pin for the first time
    firstCall = false;
  }
  LL_DRIVE_PIN(HEARTBEAT_PIN, !heartbeatState);
  heartbeatState = !heartbeatState;
}

// ============================================================================================================================= //
void initializeConfig(uart_test_cfg_t * cfg) {
    cfg->duration = 0;
    cfg->data_len_bytes = 0;
    cfg->count_mode = 0;
    cfg->data = nullptr; // Initialize data array to an empty string
    cfg->baud = (baud_t)0;
    cfg->transaction_delay_ms = 0;
}

// ============================================================================================================================= //
void printTestConfig(uart_test_cfg_t * cfg) {
    Serial.println("======== Test Configuration ========");
    Serial.print("Duration               : ");
    Serial.println(cfg->duration);
    Serial.print("Data length (bytes)    : ");
    Serial.println(cfg->data_len_bytes);
    Serial.print("Count mode             : ");
    Serial.println(cfg->count_mode);
    // KNOWN ISSUE: IMPORPER STRING TERMINATION FOR BUFF HERE, RESULTING IN ODD PRINTS TO CONSOLE. BUT DATA LOOKS OK 
    Serial.print("Transaction data       : ");
    //uIntArrayToCStr(cfg->data, buff, cfg->data_len_bytes);
    Serial.print("[");
    for (size_t i = 0; i < cfg->data_len_bytes; i++) {
        Serial.print((char)(cfg->data[i]));
        if (i < cfg->data_len_bytes - 1) {
            Serial.print(", ");
        }
    }
    Serial.println("]");
    //Serial.println(buff);
    Serial.print("Baud rate              : ");
    Serial.println(cfg->baud);
    Serial.print("Transaction delay (ms) : ");
    Serial.println(cfg->transaction_delay_ms);
    Serial.println();
}

// ============================================================================================================================= //
void app_serial_read(char * buff, uint8_t msg_len) {
  uint8_t num_bytes = 0;
  char c;
  while(num_bytes < msg_len) {
      c = SoftSerial.read();
      if (c == '\n' || c == '\r') {  // Check for line feed or carriage return
          break;
      }
      buff[num_bytes] = c;           // Store the character in the message array
      num_bytes++;
  }
  buff[num_bytes] = '\0';            // Null-terminate the string
}

// ============================================================================================================================= //
void app_serial_buffer_console(uint8_t numColumns, const char delim) {
  for(int i = 0; i < numColumns; i++) {
    Serial.print(delim);
  }
}

// ============================================================================================================================= //
void app_serial_print_bytes(uint8_t bytes[], uint8_t numBytes) {
  for(int i = 0; i < numBytes; i++) {
    SoftSerial.write(bytes[i]);
  }
}
