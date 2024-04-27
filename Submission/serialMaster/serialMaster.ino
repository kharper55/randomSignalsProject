// ============================================================================================================================= //
// The Arduino Nano features an ATMEga328P 8-bit microcontroller. See below for the datasheet
// https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf

//6500/8/0/A5A5A5A5/58600/0/8C30
#include <SoftwareSerial.h>
#include <util/crc16.h>               // Used for confirmation of SOFTUART test config data. HW UART connection to MATLAB is assumed stable
#include <stdlib.h>                   // For strtoi

#define SOFTUART0_TX_PIN     9        // Used to sending configuration data to the slave
#define SOFTUART0_RX_PIN     8        // The channel being tested (Slave Tx to Master Rx). CRC is not used to prevent tampering with the data transaction and instead a direct comparison of the data is made to classify failures.
#define SOFTUART0_RTS_PIN    4        // Used for handshaking, rising edge signifies that master is ready to receive new data
#define SOFTUART0_CTS_PIN    2        // Used for handshaking, rising edge signifies that slave has completed transmission of data
#define SOFTUART0_BAUD       9600     // Configured at 9600 upon reset; changed thereafter via transmission over SOFTUART0_TX of struct uart_test_cfg_t
#define TIMEOUT_MS           100      // Timeout in milliseconds for the test configuration data. If the timeout period is reached, the master should send the cfg data once again, until the slave acknowledges both receipt and reconfiguration 

#define DEFAULT_NUM_MSGS     1000     // Will not be featured
#define MAX_MESSAGE_LENGTH   63 + 1   // Sets max size of char buffer for transaction data

#define HEARTBEAT_PIN        13       // Other GPIO
#define HW_UART_BAUD         115200   // HW UART Config (to MATLAB)

#define MAX_TEST_DURATION    1000000  // Max Number of Transactions to Queue 1E6, 1Million
#define MAX_DATA_LEN_BYTES   8        // Max Number OF Bytes Per Transaction
#define MAX_TRANSACTION_DELAY_MS  100 // Max number of ms delay between subsequent transactions while the test is running

#define NUM_CFG_OPTS         6        // 6 true options + CRC trails the final delimiter ('\') (so 5 delimiters amongst valid data members x/x/x/x/x/x(/CRC))

// Low level GPIO driver macros
#define LL_CHECK_PIN(PIN_NO)            (PIND & (1 << (PIN_NO)))  // Read ATMEGA PINx register bit location and return value; works for input and output GPIO
#define LL_DRIVE_PIN(PIN_NO, PIN_LEVEL) (PIN_LEVEL == HIGH) ? (PORTD |= (1 << (PIN_NO))) : (PORTD &= ~(1 << (PIN_NO))) // Set level via ATMEGA PORTx register, handle low and high appropriately via ternary

// ============================================================================================================================= //

// Setup a new software serial object for the test
SoftwareSerial TestChannel(SOFTUART0_RX_PIN, SOFTUART0_TX_PIN);

const char STOP_CHAR = '\n';
const char DELIM_CHAR = '/';
const bool VERBOSE = true;
const char * COMMON_WARNING_HANDLE = "\nWARNING: ";
const uint8_t COMMON_WARNING_HANDLE_WIDTH = strlen(COMMON_WARNING_HANDLE) - 1; // account for \n
const char * COMMON_TIMEOUT_ELAPSED_HANDLE = "\nTIMEOUT: ";
const uint8_t COMMON_TIMEOUT_ELAPSED_HANDLE_WIDTH = strlen(COMMON_TIMEOUT_ELAPSED_HANDLE) - 1; // account for \n
const char * COMMON_TIMEOUT_ELAPSED_MSG = "Timeout occurred... Please verify your connections.";
const char * COMMON_TIMEOUT_ELAPSED_MSG_2 = "The device will continue attempting to establish communication indefinitely.\n";
const char * COMMON_DATA_INVALID_MSG = "Please check your test configuration entry for errors.\n";
const char * COMMON_DATA_INVALID_HANDLE = "DATA_INVALID: ";
const uint8_t COMMON_DATA_INVALID_HANDLE_WIDTH = strlen(COMMON_DATA_INVALID_HANDLE);
const char * COMMON_CRC_ERROR_MSG = "Please check that your CRC polynomial corresponds to 'A001'.\n";
const char * COMMON_CRC_ERROR_HANDLE = "CRC_ERROR: ";
const uint8_t COMMON_CRC_ERROR_HANDLE_WIDTH = strlen(COMMON_CRC_ERROR_HANDLE);
const char * COMMON_DATA_FORMAT_REMINDER = "Ensure your configuration data is formatted as follows:\n";
const char * COMMON_DATA_FORMAT_EXAMPLE = "duration/data_len_bytes/count_mode/data/baud/transaction_delay_ms/CRC\n";

byte rxBytes[MAX_MESSAGE_LENGTH];

typedef enum {
  FAIL,
  PASS
} transaction_result_t;

const char * msg_err_names[] = { // Used for local debugging purposes
  "FAIL",
  "PASS"
};

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

struct byte_err_t {
  uint8_t data;
  uint8_t bad_bits_mask;          // ORed representation of error bits (ie, if bit 1 and 2 were bad, then bad_bits_mask = (1 << 1) | (1 << 2). The bit shift should not exceed 7)
};

// All of the error information, per each single transaction, gets packed into this struct and relayed back to MATLAB via the existing HW UART port
struct transaction_err_t {
  uint16_t transaction_num;       // Transaction number out of total specified transactions
  uint32_t time_ms;               // Time of occurence relative to transaction start
  uint8_t data_len_bytes;  // Length in bytes of the transaction
  transaction_result_t result;    // Enum type. 0 == FAIL, 1 == PASS
  byte_err_t * byte_err;          // An array of structs which hold classifying information per the byte errors
};

// Rcv formatted from MATLAB, pass to slave at beginning of test. use data locally to stop test
struct uart_test_cfg_t {
  uint16_t duration;              // Duration of test in transaction counts (incremented with validation of handshaking signals, as we cannot rely on bytes coming in)
  uint8_t data_len_bytes;         // Length of transaction data for test in bytes
  bool count_mode;                // Omit user supplied data and instead use integer values representing a running transaction count for the transaction data
  uint8_t * data;                 // User supplied transaction data
  baud_t baud;                    // Baud rate for test. Requires local reconfiguration on both slave and master end. For SoftUART on the Arduino Nano (ATMega328P), limited to 115200 maximum baud rate.
  uint16_t transaction_delay_ms;  // Delay (ms) between transactions for test. Set to 0 for fastest possible transaction cycles.
  // MAKE THE ABOVE delay us instead
};

typedef enum {
  TEST_WAITING,
  TEST_READY,
  TEST_START,
  TEST_RUNNING,
  TEST_DONE
} test_state_t;

// Volatile flag to indicate interrupt occurrence
volatile bool interruptOccurred = false;

// Interrupt Service Routine (ISR) for external interrupt
void handleInterrupt() {
  interruptOccurred = true; // Set the interrupt flag, do nothing else here. Flag is acknowledged via polling in the main loop
}

// ============================================================================================================================= //
void setup() {

  // Serial channels
  Serial.begin(HW_UART_BAUD);         // HW Serial to MATLAB
  TestChannel.begin(SOFTUART0_BAUD);  // DUT Channel

  // Test channel handshaking GPIO
  pinMode(SOFTUART0_RTS_PIN, OUTPUT);
  signifySoftUART_NOT_READY();
  pinMode(SOFTUART0_CTS_PIN, INPUT);

  // Heartbeat GPIO
  pinMode(HEARTBEAT_PIN, OUTPUT);
  LL_DRIVE_PIN(HEARTBEAT_PIN, LOW);

  pinMode(SOFTUART0_CTS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SOFTUART0_CTS_PIN), handleInterrupt, RISING); // Attach interrupt for SOFTUART0_CTS_PIN
  sei(); // enable interupts globally on the ATMEGA328p
  //delay(1);
}

// ============================================================================================================================= //
void loop() {

  uart_test_cfg_t newTest;
  transaction_result_t result = PASS;
  test_state_t newTestState = TEST_WAITING;
  initializeConfig(&newTest);
  byte_err_t byte_err;
  transaction_err_t transaction_result;
  int timeout_count = 0;
  
  // New test configuration parameters
  char message[MAX_MESSAGE_LENGTH]; // Declare message locally within the loop function. Allocate MAX_MESSAGE_LENGTH + 1 byte for null char to be appended after data reception.
  uint8_t msg_len = 0;
  uint32_t count = 0;
  uint32_t failCount = 0;
  int i = 0; 
  uint8_t buff[4];
  unsigned long t_start = 0;
  bool timeout_flag = false;
  uint16_t localCRC = 0;
  char rx_buff[MAX_MESSAGE_LENGTH];
  uint16_t rxCRC = 0;
  bool invalidData_flag = false;
  bool baud_change_flag = false;
  
  char newTest_buff[MAX_MESSAGE_LENGTH];
  char newTest_buff_no_crc[MAX_MESSAGE_LENGTH];
  char * newTest_cfg_buff[NUM_CFG_OPTS + 1];

  // ======== MATLAB TEST CONFIG RETRIEVAL LOOP (CRC CHECK, DATA VALIDITY CHECK) (TEST_WAITING STATE) ======== //
  while (newTestState != TEST_READY) {
    timeout_count = 0;
    while(true){          // Loop will not exit until any # of bytes are received at a timeout increment
      t_start = millis(); // Fetch current time elapsed since runtime inception
      if (timeout_count == 0) {
        Serial.println("Waiting for test configuration struct...");
      }
      while (millis() - t_start <= TIMEOUT_MS * 10) {};
      if (Serial.available()) {
        delay(100); // Allow a write to potentially finish
        break;      // Timeout occurred and bytes came in, exit the loop
      }
      else {
        if (timeout_count == 0) {
          if (VERBOSE) {
            Serial.print(COMMON_TIMEOUT_ELAPSED_HANDLE);
            Serial.println(COMMON_TIMEOUT_ELAPSED_MSG);
            app_serial_buffer_console(COMMON_TIMEOUT_ELAPSED_HANDLE_WIDTH, ' ');
            Serial.println(COMMON_TIMEOUT_ELAPSED_MSG_2);
          }
        }
        timeout_count++;
      } 
    }
    
    msg_len = Serial.available();           // Determine how many bytes are currently in the serial buffer
    app_serial_read(newTest_buff, msg_len); // Capture, parse, and verify data from MATLAB via HW UART into local buffer

    if (VERBOSE) {
      Serial.print("RX Data: ");
      Serial.println(newTest_buff);
      Serial.println();
    }
      
    popLastData(newTest_buff, newTest_buff_no_crc, DELIM_CHAR, rx_buff);
    localCRC = computeCRC(newTest_buff_no_crc, strlen(newTest_buff_no_crc)); // NEED TO RIP OFF LAST TRAILING DAT APAST DELIM CHAR
    rxCRC = uint16_t(cStrToUint(rx_buff, 16));

    // CRC Passed. Verify numerical validity. If not ok, request new data from MATLAB
    if (localCRC == rxCRC) {
      baud_change_flag = false; // reset from possible earlier runs
      parseDelimCstr(newTest_buff_no_crc, DELIM_CHAR, newTest_cfg_buff, NUM_CFG_OPTS); // Split transaction string at delimiters
      
      if (VERBOSE) {
        for(int n = 0; n < NUM_CFG_OPTS; n++) { 
          Serial.print(n + 1);
          Serial.print(": ");
          Serial.println(newTest_cfg_buff[n]);
        }
        Serial.println();
      }

      // Upon finding invalid data, we do not immediately break so as to inform the user of all pertinent error information
      
      uint32_t duration = cStrToUint(newTest_cfg_buff[0], 10); // Check duration
      if (!(duration <= MAX_TEST_DURATION) && !(duration == 0)) {
        Serial.print(COMMON_DATA_INVALID_HANDLE);
        Serial.println("Invalid test duration setting.");
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_INVALID_MSG);
        invalidData_flag = true;
      }
      else {
        newTest.duration = duration;
      }

      uint8_t data_len_bytes = cStrToUint(newTest_cfg_buff[1], 10);
      if (!(data_len_bytes <= MAX_DATA_LEN_BYTES and data_len_bytes > 0)) {    // Check data length
        Serial.print(COMMON_DATA_INVALID_HANDLE);
        Serial.println("Invalid data length setting.");
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_INVALID_MSG);
        invalidData_flag = true;
      }
      else {
        // Add check here to verify that the length of the data buffer should match data_len_bytes. If not, truncate the data.
        newTest.data_len_bytes = data_len_bytes;
        size_t sizeX = strlen(newTest_cfg_buff[3]);
        if ((uint8_t)sizeX == data_len_bytes) {
          uint8_t * buff = (uint8_t*)malloc(sizeX); // Allocate memory dynamically
          if (buff != NULL) {
              cStrToUintArray(newTest_cfg_buff[3], buff, sizeX); // Copy the data into the dynamically allocated buffer
              newTest.data = buff; // Assign the pointer to newTest.data
          } else {
              // Failed to allocate memory, handle the error
              Serial.println("Error: Failed to allocate memory for newTest.data");
          }  
        }
        else {
          Serial.print(COMMON_WARNING_HANDLE);
          app_serial_buffer_console(COMMON_WARNING_HANDLE_WIDTH, ' ');
          Serial.println("Transaction data length (bytes) does not match actual length of transaction data.");
        } 
      }

      uint8_t count_mode =  cStrToUint(newTest_cfg_buff[2], 10);
      if (count_mode) {                                 // Check count mode
        Serial.print(COMMON_DATA_INVALID_HANDLE);
        Serial.println("Invalid count mode setting. This feature is not yet implemented.");
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_INVALID_MSG);
        invalidData_flag = true;
      }
      else {
        newTest.count_mode = count_mode;
      }

      uint32_t baud = cStrToUint(newTest_cfg_buff[4], 10);   // Check baud rate
      if (!((baud == SLOW || baud == 9600) || (baud == MEDIUM || baud == 58600) || (baud == FAST || baud == 115200))) {
        Serial.print(COMMON_DATA_INVALID_HANDLE);
        Serial.println("Invalid baud rate setting.");
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_INVALID_MSG);
        invalidData_flag = true;
      }
      else {
        newTest.baud = (baud_t)baud;
        if (baud != SOFTUART0_BAUD) {
          if(VERBOSE) {
            Serial.println("Baud change detected!");
            Serial.print("Default : ");
            Serial.println(SOFTUART0_BAUD);
            Serial.print("New     : ");
            Serial.println(baud);
            Serial.println();  
          } 
          baud_change_flag = true;            // Set a flag indicating we require a reconfiguration of the softuart port prior to beginning the test
        }
      }
      
      uint16_t transaction_delay_ms = cStrToUint(newTest_cfg_buff[5], 10); // Check transaction delay
      if (!(transaction_delay_ms <= MAX_TRANSACTION_DELAY_MS)) {
        Serial.print(COMMON_DATA_INVALID_HANDLE);
        Serial.println("Invalid transaction delay setting.");
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_INVALID_MSG);
        invalidData_flag = true;
      }
      else {
        newTest.transaction_delay_ms = transaction_delay_ms;
      }

      // Data is valid
      if (!invalidData_flag) {
        newTestState = TEST_READY;
      }

      // Data is NOT valid (but CRC passed)
      else {
        Serial.println(COMMON_DATA_FORMAT_REMINDER);
        app_serial_buffer_console(COMMON_DATA_INVALID_HANDLE_WIDTH, ' ');
        Serial.println(COMMON_DATA_FORMAT_EXAMPLE);
      }
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

  // ======== BEGIN RELAYING TEST CFG DATA TO SLAVE (TEST_READY STATE) ======== //
  timeout_flag = false;                         // Reset from earlier timeouts
  timeout_count = 0;                            // Reset from earlier use
  
  while (newTestState != TEST_START) {
    if (VERBOSE && timeout_count == 0) {
        printTestConfig(&newTest);
        delay(30 * TIMEOUT_MS);                 // Provide the user 3 seconds to readback the test configuration about to be sent
    }
    
    Serial.println("Forwarding new test configuration to slave...");

    //t_start = millis();                         // Fetch current time elapsed since runtime inception
    while (!interruptOccurred) {                // Wait for interrupt from slave signifying confirmation that the data has been received and reconfiguration has been performed. Loop will exit automatically once this occurs             
      if ((millis() - t_start >= TIMEOUT_MS * 10) || (timeout_count == 0)) {
        t_start = millis(); 
        if (timeout_count == 1) {
          Serial.print(COMMON_TIMEOUT_ELAPSED_HANDLE);
          Serial.println(COMMON_TIMEOUT_ELAPSED_MSG);
          app_serial_buffer_console(COMMON_TIMEOUT_ELAPSED_HANDLE_WIDTH, ' ');
          Serial.println(COMMON_TIMEOUT_ELAPSED_MSG_2);
        }
        TestChannel.print(newTest_buff);        // Send new test configuration parameters to slave
        delayMicroseconds(10);
        signifySoftUART_READY();                // Force an interrupt to encourage the slave to recognize the new bytes
        delayMicroseconds(10);
        signifySoftUART_NOT_READY();
        timeout_count++;
        t_start = millis();  
      }
    }
    interruptOccurred = false;    // reset the interrupt flag
    signifySoftUART_NOT_READY();  // ensure this pin is driven low
        
    // Upon confirmation of the slave's reconfiguration, reconfigure the master for the test, and then advance the test state thru the fsm
    if (baud_change_flag) {
      Serial.print("\nReconfiguring DUT for ");
      Serial.print(newTest.baud);
      Serial.println(" baud.\n");
      TestChannel.begin(newTest.baud);
      Serial.println("DUT baud rate updated.\n");
      newTestState = TEST_START;
    }
  }

  // Kick the test off
  count = 0;
  Serial.println("Starting test...");
  
  
  initializeTransaction(&transaction_result, &newTest);
  initializeByteError(&byte_err);
  t_start = millis();
  newTestState = TEST_RUNNING;
  
  while (newTestState != TEST_DONE) {

    transaction_result.result = PASS; // only updated if a failure occurs

    // Tell the slave it can send data
    signifySoftUART_READY();

    // Wait for the slave to signal that it has completed its transfer
    while (!interruptOccurred) {};
    signifySoftUART_NOT_READY();

    // Grab number of bytes received
    msg_len = TestChannel.available();

    if (msg_len) { // Bytes have appeared in the peripheral buffer -- read them out and check for errors, classify the errors, and report to MATLAB
      
      transaction_result.transaction_num = count;
      transaction_result.time_ms = millis() - t_start;

      if (VERBOSE) {
        Serial.print("(");
        Serial.print(count);    // Expected value
        Serial.print(") RX : ");
      }

      // CLASSIFY TRANSACTION RESULTS
      // Bytes have appeared in the peripheral buffer. Process them and populate transaction_byte_len byte_err_t's
      uint8_t num_bytes = 0;
      byte xByte;
      while (num_bytes < msg_len && num_bytes < newTest.data_len_bytes) {
          xByte = TestChannel.read(); // Read the incoming byte
          
          byte_err.data = (uint8_t)xByte;

          //Serial.println((byte)byte_err.data);
          
          if (VERBOSE) {
            Serial.print(xByte);
          }
          
          if (!(newTest.data[newTest.data_len_bytes - num_bytes - 1] == xByte)) { // This line compares the bytes read in to the bytes that are in the test cfg data buffer, in reverse order
            // Byte mistmatch detected!
            if (VERBOSE) {
                Serial.print("MISMATCH");
            }
            uint8_t err_bits = newTest.data[newTest.data_len_bytes - num_bytes - 1] ^ xByte;
            byte_err.bad_bits_mask = err_bits;
            // err_bis holds a binary uint8_t representing all the bad bits
            Serial.print(err_bits);
          }
          else {
            byte_err.bad_bits_mask = 0;
          }

          // Append byte error results to transaction err struct
          
          //transaction_result.byte_err[newTest.data_len_bytes - num_bytes - 1] = byte_err;
          
          num_bytes++; // Increment byte count
      }
      Serial.println();
      
      // Check if the incorrect bytes were read
      //result = (atoi(message) == count - 1) ? PASS : FAIL;
      if (result == FAIL) {
        transaction_result.result = FAIL;
        failCount++;
      }
    }
        
    else { // No bytes were read, despite the slave indicating it has sent more data (having toggled the softUART_DONE signal)
      if (count != 0) {
        transaction_result.result = FAIL;
        failCount++;
      }
    }

    // Send newest info to MATLAB
    /*
     * Serial.println()
     * 
     */
    //printTransaction(&transaction_result);
    count++;                   // Increment the test count
    toggleHeartbeat();         // Toggle the heartbeat for a visual indication of the running test
    interruptOccurred = false; // Reset the interrupt flag
    if (count == newTest.duration) {
      newTestState = TEST_DONE;
      break;
    }
  }
  printResults(count - 1, failCount);
  while(1);
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
void app_serial_read(char * buff, uint8_t msg_len) {
  uint8_t num_bytes = 0;
  char c;
  while(num_bytes < msg_len) {
        c = Serial.read();
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
// Signify to slave that master is ready for more data. Should always be called in complement with signifySoftUART_NOT_READY()
bool signifySoftUART_READY(void) {
  if (not LL_CHECK_PIN(SOFTUART0_RTS_PIN)) {
    LL_DRIVE_PIN(SOFTUART0_RTS_PIN, HIGH);
    return true;
  }
  else {
    return false;
  }
}

// ============================================================================================================================= //
// Signify to slave that master is NOT ready for more data
bool signifySoftUART_NOT_READY(void) {
  if (LL_CHECK_PIN(SOFTUART0_RTS_PIN)) {
    LL_DRIVE_PIN(SOFTUART0_RTS_PIN, LOW);
    return true;
  }
  else {
    return false;
  }
}

// ============================================================================================================================= //
bool checkSoftUART_DONE(void) { // Polling option for checking CTS status for SOFTUART
  return (LL_CHECK_PIN(SOFTUART0_CTS_PIN));
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
void initializeTransaction(transaction_err_t * transaction, uart_test_cfg_t * cfg) {
    transaction->transaction_num = 0;
    transaction->time_ms = 0;
    transaction->data_len_bytes = cfg->data_len_bytes;
    transaction->result = PASS; // Initialize data array to an empty string
    transaction->byte_err = nullptr;
}

// ============================================================================================================================= //
void initializeByteError(byte_err_t * byte_err) {
    byte_err->data = 0;
    byte_err->bad_bits_mask = 0;
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

void printTransaction(transaction_err_t * transaction) {
  Serial.println("======== Transaction Info ========");
  Serial.print("Transaction #       : ");
  Serial.println(transaction->transaction_num);
  Serial.print("Time (ms)           : ");
  Serial.println(transaction->time_ms);
  Serial.print("Data length (bytes) : ");
  Serial.println(transaction->data_len_bytes);
  Serial.print("Result              : ");
  Serial.println(msg_err_names[(int)transaction->result]);
  Serial.print("Byte err            : ");
  /*
   * for (int i = 0; i < transaction->data_len_bytes; i++) {
    Serial.print("Byte ");
    Serial.print(i);
    Serial.print(" - Data: ");
    Serial.print(transaction->byte_err[i].data);
    Serial.print(", Bad Bits Mask: ");
    Serial.println(transaction->byte_err[i].bad_bits_mask, BIN);
  }
   */
  // Loop through and print all byte errors. Write a function that prints a single byte err
  Serial.println("dummy");
  Serial.println();
}

// ============================================================================================================================= //
void printResults(uint32_t total, uint32_t failCount) {
  Serial.print("Results: ");
  Serial.print("Total = ");
  Serial.print(total);
  Serial.print(" Fails = ");
  Serial.println(failCount);
}
