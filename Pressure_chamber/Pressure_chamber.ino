#include <Wire.h>

//////////// 
// Notes
// STEPPER: Initialize the location parameters to some value
// STEPPER: Put the pins in the right mode in the setup
// AD, MS: Add Wire.begin() in the setup
// AD, MS: Reset both devices in the setup
// AD: Set the registers to what they should be
// Don't forget to Serial.begin(115200);
// STEPPER: Review constants
////////////

////////////////////////////////////////////////////////////////
//THIS PART WAS ADDED PURELY TO TEST A CHANGE PUSHED TO GITHUB//
////////////////////////////////////////////////////////////////


//*****************************************************************************************
// DEFINITIONS
//********************************************************
// AD715x definitions 
//********************************************************
// AD715X register addresses****
#define AD715X_ADDRESS_W   0x48 // 0100 1000 7-bit address 
#define AD715X_ADDRESS_R   0x48 // 0100 1000 7-bit address
#define STATUS_REG         0x00 // Default value: 0x03  
#define CH1_DATA_MSB_REG   0x01 // Default value: 0x00
#define CH1_DATA_LSB_REG   0x02 // Default value: 0x00 
#define CH2_DATA_MSB_REG   0x03 // Default value: 0x00
#define CH2_DATA_LSB_REG   0x04 // Default value: 0x00  
#define CH1_CAP_SETUP_REG  0x0B // Default value: 0x00
#define CH2_CAP_SETUP_REG  0x0E // Default value: 0x00
#define CONFIGURATION_REG  0x0F // Default value: 0x00
#define CAPDAC_POS_REG     0x11 // Default value: 0x00
#define CAPDAC_NEG_REG     0x12 // Default value: 0x00
#define CONFIGURATION2_REG 0x1A // Default value: 0x00
// STATUS_REG*******************
#define DATA_READY_POS_CH1 0x01
#define DATA_READY_POS_CH2 0x02
// CH1_CAP_SETUP_REG************
// CH2_CAP_SETUP_REG************
#define RANGE_2pF          0x00       
#define RANGE_05pF        0x40
#define RANGE_025pF       0x80
#define RANGE_4pF          0xC0
#define DIFF_MODE          0x20
#define SINGLE_ENDED_MODE  0x00
// CONFIGURATION_REG************
#define CH1_ENABLE         0x10
#define CH2_ENABLE         0x08
#define CH1_DISABLE        0x00
#define CH2_DISABLE        0x00
#define IDLE_MODE          0x00
#define CONT_CONV_MODE     0x01
#define SINGLE_CONV_MODE   0x02
#define POWER_DOWN_MODE    0x03
#define CAP_SYS_OFFSET_CAL 0x05
#define CAP_SYS_GAIN_CAL   0x06
// CAPDAC_POS_REG***************
#define DACP_ENABLE        0x80  
#define DACP_DISABLE       0x00
// CAPDAC_NEG_REG***************
#define DACN_ENABLE        0x80
#define DACN_DISABLE       0x00
// CONFIGURATION2_REG***********
#define CONV_TIME_5MS      0x00
#define CONV_TIME_20MS     0x10
#define CONV_TIME_50MS     0x20
#define CONV_TIME_60MS     0x30
// CONFIGURATION2_REG***********
#define RESET_COMMAND      0xBF
// GENERAL**********************
#define POS_CAPDAC         1
#define NEG_CAPDAC         2
#define CAPDAC_UPPER_RANGE 5 // The upper range of the CAPDAC is 5pF (it can be 5pF(1 +- 20%) )
#define CAPDAC_UP_RANGE_HX 0x1F
#define TIMEOUT_CAP_RETURN 0xFFFFFFFF 
#define CAP_READ_TIMEOUT   150 
//********************************************************
// NRF51822 register configurations 
//********************************************************
// ENABLE***********************
#define ENABLE_TWI         0x00000005
#define DISABLE_TWI        0x00000000
// PSELSCL**********************
#define SCL_SELECT_PIN_5   0x00000020
// PSELSDA**********************
#define SDA_SELECT_PIN_6   0x00000040
// FREQUENCY********************
#define TWI_FREQ_100KHz    0x01980000
#define TWI_FREQ_250KHz    0x04000000
#define TWI_FREQ_400KHz    0x06680000
// ADDRESS**********************
#define MASTER_ADDRESS     0x0000006F
// GPIO->DIR********************
#define PIN_5_6_INPUT      0xFFFFFF9F
// GPIO->PIN_CNF****************
#define DIRECTION_INPUT    0x00000000
#define DIRECTION_OUTPUT   0x00000001
#define CONNECT_INPUT_BUF  0x00000000
#define INPUT_NO_PULL      0x00000000 
#define INPUT_PULL_UP      0x0000000C
#define DRIVE_S0_D1        0x00000600
// Start conditions*************
#define READ_START_COND    2
#define WRITE_START_COND   1
#define NO_START_COND      0
//********************************************************
// MS5803 definitions 
//********************************************************
// MS5803 device address********
#define MS5803_ADDRESS     0x76 // Pin CSB connected to HIGH by default on the Sparkfun board                        
// MS5803 commands**************
#define RESET_MS5803       0x1E
#define CONVERT_1_256      0x40
#define CONVERT_1_512      0x42
#define CONVERT_1_1024     0x44
#define CONVERT_1_2048     0x46
#define CONVERT_1_4096     0x48
#define CONVERT_2_256      0x50
#define CONVERT_2_512      0x52
#define CONVERT_2_1024     0x54
#define CONVERT_2_2048     0x56
#define CONVERT_2_4096     0x58
#define ADC_READ           0x00
#define PROM_READ          0xA0 // OR it with calibration coefficient number 
// Maximum conversion times*****
#define MAX_CONV_TIME_256   1  // Actual is 0.60ms
#define MAX_CONV_TIME_512   2  // Actual is 1.17ms
#define MAX_CONV_TIME_1024  3  // Actual is 2.28ms
#define MAX_CONV_TIME_2048  5  // Actual is 4.54ms
#define MAX_CONV_TIME_4096  10 // Actual is 9.04ms
#define SAFE_MAX_CONV_TIME  10 // Safe conversion time for the ADC to return valid data for all OSRs
#define MS_ADC_READ_TIMEOUT 50 // 50ms to time_out after a wait for data from the MS through the I2C bus
//********************************************************
// SERIAL COMMUNICATION definitions 
//********************************************************
// Parsed commands**************
#define SLOW_MOVE           1
#define FAST_MOVE           2
#define HOMING              3
#define CALIBRATION         4
#define DYNAMIC             5
#define CALIBRATION_DATA    6
#define COMMAND_ERROR       0
// Miscellaneous****************
#define INIT_COMM_TIMEOUT   100
#define INIT_COMM_TIMEOUT_LP 10
//********************************************************
// STEPPER definitions 
//********************************************************
// Dimensiosn and limits********
#define MAX_STEP_POSITION          38400 // The length of the whole track in steps
#define DISTANCE_PER_SINGLE_STEP   312.5 // 312.5uInch is moved per single motor step
#define MAX_DISTANCE               12 // 12 inches is the length of the whole track
#define MAX_POSITION_SAFETY_MARGIN 100 // Distance from the maximum position not to be trespassed
// Pins for driver**************
#define DIR_pin                    2 // Pin 2 is to be used to control the stepping direction
#define STEP_pin                   3 // Pin 3 is to be used for generating the steps
#define TOUCH_SENSOR_pin           4 // Pin 4 is to be used for the touch sensor 
#define SLEEP_PIN                  4 // Pin 4 is the sleep pin       
// Pin sleep states*************
#define DRIVER_SLEEP_ON            HIGH
#define DRIVER_SLEEP_OFF           LOW
// Pin states' directions*******
#define DIR_forward_pin_state      LOW // Direction is forward at the DIR pin of the stepper driver 
#define DIR_backward_pin_state     HIGH // Direction is backward at the DIR pin of the stepper driver
// Periods between steps********
#define SLOW_MOVE_STEP_PERIOD      40 // The period between a step pulse and another in the SLOW MOVE mode
#define FAST_MOVE_STEP_PERIOD      30 // The period between a step pulse and another in the FAST MOVE mode
#define HOMING_MOVE_STEP_PERIOD    10 // The period between a step pulse and another in the HOMING motion mode
#define CALIBRATION_STEP_PERIOD    15 // The period between a step pulse and another in the CALIBRATION mode
// Single step details**********
#define STEP_PULSE_HIGH_PERIOD     1 // The period for which the step pulse remains high
#define STEP_PULSE_LOW_PERIOD      2 // The period for which the step pulse remains low


//*****************************************************************************************
// STRUCTURES
//********************************************************
// AD715x structures
//********************************************************
// AD7152 parameters************
struct AD7152_parameters{
  uint32_t active_channel; // The current active channel (CH1_ENABLE => Channel 1, CH2_ENABLE => Channel 2, CH1_DISABLE or CH2_DISABLE => All channels disabled) 
  uint32_t conversion_mode; // Continuous, idle, single...etc
  uint32_t measurement_mode; // Differential or single ended
  uint32_t range; // The measurement range
  uint32_t active_CAPDAC; // The CAPDAC that is active (1 => Positive CAPDAC, 2 => Negative CAPDAC, 0 => All CAPDACs disabled)
  uint32_t pos_CAPDAC_offset ; // The offset value for the positive CAPDAC
  uint32_t neg_CAPDAC_offset ; // The offset value for the negative CAPDAC
  uint32_t conversion_time_ms; // The time required for a conversion in ms
};
struct AD7152_parameters AD7152_param;
//********************************************************
// MS5803 structures
//********************************************************
// MS5803 coefficients********** 
// (review datasheet page 7 of** 
// 20 for the meaning of c1, c2...c4)
struct calibration_parameters{
  uint32_t c1;
  uint32_t c2;
  uint32_t c3;
  uint32_t c4;
  uint32_t c5;
  uint32_t c6;
};
struct calibration_parameters calib_param;
//********************************************************
// SERIAL COMMUNICATION structures
//********************************************************
// Slow Move parameters*********
struct SLOW_MOVE_parameters{
  int desired_position;
};
struct SLOW_MOVE_parameters SLOW_MOVE_param;
// Fast Move parameters*********
struct FAST_MOVE_parameters{
  int desired_position;
};
struct FAST_MOVE_parameters FAST_MOVE_param;
// Homing parameters************
struct HOMING_parameters{
};
struct HOMING_parameters HOMING_param;
// Calibration parameters*******
struct CALIBRATION_parameters{
  int start_position;
  int end_position;
  int increments;
  int wait;
};
struct CALIBRATION_parameters CALIBRATION_param;
// Dynamic parameters***********
struct DYNAMIC_parameters{
  int start_position;
  int end_position;
  int frequency;
};
struct DYNAMIC_parameters DYNAMIC_param;
// Calibration data parameters**
struct CALIBRATION_DATA_parameters{
};
struct CALIBRATION_DATA_parameters CALIBRATION_DATA_param;
//********************************************************
// STEPPER structures
//********************************************************
// Current position parameters **
struct position_parameters{
  int current_position_in_steps; // Current position of the stepper motor
  boolean is_homed; // True only when the homing touch sensor is touched, any movement afterwards turns it to false
};
struct position_parameters position_param;



//*****************************************************************************************
// GLOBAL VARIABLES
//********************************************************
// SERIAL COMMUNICATION global variables
//********************************************************
boolean is_connected_to_MATLAB = false; // Boolean to store connection status with MATLAB
boolean reached_upper_end = false; // Boolean to check if the stepper reached the upper limit of distance
boolean reached_lower_end = false; // Boolean to check if the stepper reached the lower limit of distance
String raw_command; // The raw command received 
char init_ackn_char; // Initial acknowledgement character
int last_command; // The last command received from MATLAB
boolean last_command_executed = false; // A boolean to keep track of command execution
boolean is_I2C_reading_timed_out = false; // A boolean determining if a time_out has taken place in the transmit_data_and_wait function on the I2C bus
int data_acq_counter = 0;
//*****************************************************************************************
// Setup method 

void setup() {
  // Communication enabling 
  
  Wire.begin(); // Enable I2C
  Serial.begin(115200); // Enable Serial at 115200 bps baud rate
  
  // AD7152 setup steps:
  // Reset the AD
  // Determine the measurement mode and the range
  // Determine the conversion mode
  // Determine the conversion time
  // Disable the CAPDACs or enable them
  // Determine the conversion time for the program memory
  // 1.4pF negative CAPDAC offset
  // 0.5 Range
  AD_reset(); // Reset AD7152
  delay(5); // Delay 50ms to make sure the reset is finished
  write_byte_to_register(AD715X_ADDRESS_W, CH1_CAP_SETUP_REG, RANGE_05pF | DIFF_MODE); // Determine the range, and the measurement mode  
  delay(5);
  write_byte_to_register(AD715X_ADDRESS_W, CONFIGURATION_REG, CH1_ENABLE | CONT_CONV_MODE); // Define the conversion mode to be a single conversion 
  delay(5);
  write_byte_to_register(AD715X_ADDRESS_W, CONFIGURATION2_REG, CONV_TIME_60MS); // Determine the OSR to be the highest, so that the conversion time is 60ms
  delay(5); 
  set_negative_CAPDAC_offset(1.4, 1);
  // write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_POS_REG, DACP_DISABLE); // Make sure positive CAPDAC is disabled
  delay(5);
  // write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_NEG_REG, DACN_DISABLE); // Make sure negative CAPDAC is disabled 
  delay(5);
  AD7152_param.conversion_time_ms = 60;
  //Serial.println("Step 1 done...");
  // MS5803 setup steps:
  // Reset the MS5893
  write_byte_on_bus(MS5803_ADDRESS, RESET_MS5803); // Reset the MS5803 
  delay(50); // Delay 50ms to make sure the reset is finished
  //Serial.println("Step 2 done...");
  // Serial setup
  // Initialize communication with MATLAB
  initialize_MATLAB_communication();
  //Serial.println("Step 3 done...");
  // Stepper setup
  // Initialize the current position in the program memory to be in the middle (total distance in 38400, half of this is 14200) 
  // Set the pins in the right mode
  // Initialize the HOMING flag to indicate that the stepper is not homed
  // Execute homing 
  position_param.current_position_in_steps = 14200;
  pinMode(TOUCH_SENSOR_pin, INPUT_NOPULL); // Put the pin to sense the touch sensor in an input mode
  pinMode(DIR_pin, OUTPUT);
  pinMode(STEP_pin, OUTPUT);
  position_param.is_homed = false; // Initialize is_homed to be false
  Serial.print('b');
  STEPPER_execute_HOMING();
  Serial.print('c');
  //Serial.println("Step 5 done...");
  //STEPPER_move_step_number(2, 4000, true);
}



//*****************************************************************************************
// Loop method
void loop() {   
  
  if(Serial.available() != 0){ // Check if there is anything on the serial buffer
    last_command = parse_command(); // Store the latest command integer equivalent in the last_command variable
    last_command_executed = false; // Tell the program that there is a new command to be executed
  }else if(!last_command_executed){ // In case there is nothing in the buffer, and that the last command is still not executed
    if(last_command == SLOW_MOVE){  
      STEPPER_move_to_point(SLOW_MOVE_STEP_PERIOD, SLOW_MOVE_param.desired_position); // Move to the required point in a slow speed
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if(last_command == FAST_MOVE){
      STEPPER_move_to_point(FAST_MOVE_STEP_PERIOD, FAST_MOVE_param.desired_position); // Move to the required point in a fast speed      
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if(last_command == HOMING){
      STEPPER_execute_HOMING(); // Execute the homing prucedure    
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if(last_command == CALIBRATION){
      execute_CALIBRATION(); // Execute the standard calibration
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if(last_command == DYNAMIC){
      //execute_DYNAMIC_calibration(); // Execute the dynamic calibration      
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if(last_command == CALIBRATION_DATA){
      MS_store_coefs_in_memory();
      send_MS_calibration_coefficients(); // Read the calibration data from MS and send them to MATLAB      
      last_command_executed = true; // Tell the program that we have executed the last command
    }else if (last_command == COMMAND_ERROR){
      //Serial.print("XoX"); // Send a dead fish face to MATLAB
      last_command_executed = true; // Tell the program that we have executed the last command 
    }
  }
  
} 



//*****************************************************************************************
// Functions
//********************************************************
// AD7152 functions
//********************************************************
//*********************************************************************************************************
// AD7152
// Function to read the capacitance from the AD7152 data registers
// Parameters:
//   >> channel_number: The number of the channel to read capacitance from (1 => Channel_1, 2 => Channel_2)  
// Return: Capacitance read from the AD7152
//*********************************************************************************************************
uint32_t AD_read_capacitance(int channel_number){
  uint32_t capacitance; // Capacitance value
  byte LS_Byte_capacitance, MS_Byte_capacitance; // The two data bytes
  
  Wire.beginTransmission(AD715X_ADDRESS_W); // Start by writing the addresses of the data registers to the Address Pointer register
  if(channel_number == 1){
    Wire.write(CH1_DATA_LSB_REG); // Point to data address of channel 1 in AD751X
  }else if(channel_number == 2){
    Wire.write(CH2_DATA_LSB_REG); // Point to data address of channel 2 in AD751X
  }  
  
  Wire.requestFrom(AD715X_ADDRESS_R, 2); // Request reading data from the register already pointed to in the Address Pointer register
  while(Wire.available() == 0){} // Wait until data is available
  LS_Byte_capacitance = Wire.read(); // Store LSB
  MS_Byte_capacitance = Wire.read(); // Store MSB
  
  capacitance = ((MS_Byte_capacitance << 8) | (LS_Byte_capacitance)) & 0x0000FFF0; // Masking to get the least significant 12 bits
  
  return capacitance; // Return the capacitance read
} 
  
//*********************************************************************************************************
// AD7152
// Function to request the capacitance from the AD7152 data registers
// Parameters:
//   >> channel_number: The number of the channel to read capacitance from (1 => Channel_1, 2 => Channel_2)  
// Return: Capacitance read from the AD7152
////////Notes:
// This function assumes the data conversion mode is Single Data Conversion mode
// This function is particularly useful in the calibration process
//*********************************************************************************************************
void AD_request_capacitance(int channel_number){
  //Serial.print(" CBR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
  // Enable the chosen channel for a Single Conversion
  if(channel_number == 1){ 
    write_byte_to_register(AD715X_ADDRESS_W, CONFIGURATION_REG, CH1_ENABLE | SINGLE_CONV_MODE);  
  }else if(channel_number == 2){
    write_byte_to_register(AD715X_ADDRESS_W, CONFIGURATION_REG, CH1_ENABLE | SINGLE_CONV_MODE);
  }
  //Serial.print("Requested C..."); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  //Serial.print(" CAR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
} 
  
//*********************************************************************************************************
// AD7152
// Function to read the requested capacitance from the AD7152 data registers
// Parameters:
//   >> channel_number: The number of the channel to read capacitance from (1 => Channel_1, 2 => Channel_2)  
// Return: Capacitance read from the AD7152
////////Notes:
// The channel number must correspond to the channel_number parameter in the 
// AD_request_capacitance function, otherwise the data might be corrupted 
// The function will not return until a data point is ready, it uses a time out CAP_READ_TIMEOUT (150ms) 
// If timeout is exceeded, the function will break returning a TIMEOUT_CAP_RETURN (0xffffffff)
//*********************************************************************************************************
uint32_t AD_read_requested_capacitance(int channel_number){
  unsigned long start_time;
  uint32_t capacitance; // Capacitance value
  byte LS_Byte_capacitance, MS_Byte_capacitance; // The two data bytes
  start_time = millis();
  while(!AD_is_data_ready(channel_number)){ // Keep looping until data is ready
    if(millis() - start_time > CAP_READ_TIMEOUT){ // Make sure we don't loop indifinetely in case of some error
      return(TIMEOUT_CAP_RETURN); // Return 0xFFFFFFFF in case of timeout TIMEOUT_CAP_RETURN 
    }
    //Serial.println("Waiting for capacitance..."); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  }
  // Now we know data is ready to pick
  Wire.beginTransmission(AD715X_ADDRESS_W); // Start by writing the addresses of the data registers to the Address Pointer register
  if(channel_number == 1){
    Wire.write(CH1_DATA_LSB_REG); // Point to data address of channel 1 in AD751X
  }else if(channel_number == 2){
    Wire.write(CH2_DATA_LSB_REG); // Point to data address of channel 2 in AD751X
  }  
  //Serial.print(" C bef request "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  Wire.requestFrom(AD715X_ADDRESS_R, 2); // Request reading data from the register already pointed to in the Address Pointer register
  while(Wire.available() == 0){} // Wait until data is available
  MS_Byte_capacitance = Wire.read(); // Store LSB
  LS_Byte_capacitance = Wire.read(); // Store MSB
  capacitance = ((MS_Byte_capacitance << 8) | (LS_Byte_capacitance)) & 0x0000FFF0; // Masking to get the least significant 12 bits
  //Serial.println("C is read"); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  //Serial.print(" C aft request "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
  return capacitance; // Return the capacitance read
}

//*********************************************************************************************************
// AD7152
// Function to reset the AD7152
// Parameters: None
// Return: None
//*********************************************************************************************************
void AD_reset(){
  Wire.beginTransmission(AD715X_ADDRESS_W);
  Wire.write(RESET_COMMAND);
  Wire.endTransmission();  
  delay(5);
}

//*********************************************************************************************************<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// AD7152
// Function to set the offset in the positive CAPDAC and enable it 
// Parameters: 
//   >> offset_value: The value of capacitance that is to be offset by the positive CAPDAC 
//   >> is_enable_PCAPDAC: True => Positive CAPDAC is enabled, False =>Positive CAPDAC is enabled
// Return: None
//*********************************************************************************************************
void set_positive_CAPDAC_offset(float offset_value, boolean is_enable_PCAPDAC){
  float offset_float;
  int offset_int;
  // Calculate the integer hexadecimal equivalent of the offset 
  offset_float = (offset_value/CAPDAC_UPPER_RANGE)*(CAPDAC_UP_RANGE_HX); 
  offset_int = (int)offset_float;
  // Or the hexadecimal offset value with the enable or disable commands  
  // Send the commands to the positive CAPDAC configuration register
  if(is_enable_PCAPDAC){
    write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_POS_REG, offset_int | DACP_ENABLE);
  }else{
    write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_POS_REG, offset_int | DACP_DISABLE);
  }
  
  // TO DO: Keep track of the changes in the AD7152 parameters structure ##########################
}

//*********************************************************************************************************<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// AD7152
// Function to set the offset in the negative CAPDAC and enable it 
// Parameters: 
//   >> offset_value: The value of capacitance that is to be offset by the negative CAPDAC 
//   >> is_enable_NCAPDAC: True => Negative CAPDAC is enabled, False => Negative CAPDAC is enabled
// Return: None
//*********************************************************************************************************
void set_negative_CAPDAC_offset(float offset_value, boolean is_enable_NCAPDAC){
  float offset_float;
  int offset_int;
  // Calculate the integer hexadecimal equivalent of the offset 
  offset_float = (offset_value/CAPDAC_UPPER_RANGE)*(CAPDAC_UP_RANGE_HX); 
  offset_int = (int)offset_float;
  // Or the hexadecimal offset value with the enable or disable commands  
  // Send the commands to the positive CAPDAC configuration register
  if(is_enable_NCAPDAC){
    write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_NEG_REG, offset_int | DACN_ENABLE);
  }else{
    write_byte_to_register(AD715X_ADDRESS_W, CAPDAC_NEG_REG, offset_int | DACN_DISABLE);
  }
  
  // TO DO: Keep track of the changes in the AD7152 parameters structure ##########################
}

//*********************************************************************************************************
// AD7152
// Function to check if the data for a certain channel is ready or not
// Parameters: 
//   >> channel_number: Number of the channel to read data from (1 => Channel_1, 2 => Channel_2)
// Return: Data status (true => Data from specified channel is ready, false => Data from specified channel is not ready)
//*********************************************************************************************************
boolean AD_is_data_ready(int channel_number){
  uint8_t status_reg; // Contents of the data register in the AD7152

  status_reg = read_byte_from_register(AD715X_ADDRESS_R, STATUS_REG); // Read status register value in the AD7152 and store it in the variable  

  if(channel_number == 1){ 
    if(status_reg & DATA_READY_POS_CH1 == 0){ // Compare with the position of the DATA_READY for channel 1 in the status register
      return true;
    }else if(status_reg & DATA_READY_POS_CH1 == 1){
      return false;
    }    
  }else if(channel_number == 2){ // Compare with the position of the DATA_READY for channel 2 in the status register
    if(status_reg & DATA_READY_POS_CH2 == 0){
      return true;
    }else if(status_reg & DATA_READY_POS_CH2 == 1){
      return false;
    }
  }
}

//*********************************************************************************************************
// AD7152 & others
// Function to write one byte to a register in a given device
// Parameters:
//   >> device_address_WR: The address of the device to write the data to
//   >> register_address: The register address in the specified device
//   >> data_to_write: The data to be written to the specified register in the specified device
// Return: None
////////Notes:
// This function can be used to send commands too, cautiously
//*********************************************************************************************************
void write_byte_to_register(unsigned char device_address_WR, unsigned char register_address, unsigned char data_to_write){
  Wire.beginTransmission(device_address_WR); // Send the device address 
  Wire.write(register_address); // Indicate the register address
  Wire.write(data_to_write); // Indicate the data to be written to the specified register
  Wire.endTransmission(); // Send the data
}

//*********************************************************************************************************
// AD7152 & others
// Function to read a byte from a register in a given device
// Parameters: 
//   >> device_address_WR: The address of the device to read the data from
//   >> register_address: The register address in the specified device to read the data from
// Return: Byte read from the specified register in the specified device
//*********************************************************************************************************
byte read_byte_from_register(unsigned char device_address_WR, unsigned char register_address){
  Wire.beginTransmission(device_address_WR); // Send the device address  
  Wire.write(register_address); // Write the register address to the Address Pointer register
  Wire.endTransmission(false); // End transmission without generating a stop condition
  Wire.requestFrom(device_address_WR,1); // Request reading data from the register already pointed to in the Address Pointer register
  
  while(Wire.available() == 0){} // Wait until data is ready
 
  return Wire.read(); // Returning the byte that was recieved
}
//********************************************************
// MS5803 functions
//********************************************************
//*********************************************************************************************************
// MS5803 mostly                                                                                         
// Function to write a command byte on the bus, mostly used with MS5803    
// Parameters: 
//   >> device_address_WR: Address of the device to be written to
//   >> data_to_write: The data to be written on the bus right after addressing the device
// Return: None
//////// Notes:
// The transmission is initiated by a start condition before putting the device_address_WR and a stop condition after putting data_to_write on the line
//*********************************************************************************************************
void write_byte_on_bus(unsigned char device_address_WR, unsigned char data_to_write){
  Wire.beginTransmission(device_address_WR); // Generate a start condition then clock out the device address
  Wire.write(data_to_write); 
  Wire.endTransmission(); // Clock out the data_to_write then generate a stop condition
}

//*********************************************************************************************************
// MS5803                                                                                                  
// Function to read the factory calibration coefficients and store them in a data structure in memory    
// Parameters: None                                                                                    
// Return: None
//*********************************************************************************************************
void MS_store_coefs_in_memory(){ // This function will set the pressure_param and temperature_param members
  // Send the PROM read command 
  calib_param.c1 = MS_read_coef_from_bus(1); // To store coefficient C1 in the global data structure calib_param
  calib_param.c2 = MS_read_coef_from_bus(2); // To store coefficient C2 in the global data structure calib_param
  calib_param.c3 = MS_read_coef_from_bus(3); // To store coefficient C3 in the global data structure calib_param
  calib_param.c4 = MS_read_coef_from_bus(4); // To store coefficient C4 in the global data structure calib_param
  calib_param.c5 = MS_read_coef_from_bus(5); // To store coefficient C5 in the global data structure calib_param
  calib_param.c6 = MS_read_coef_from_bus(6); // To store coefficient C6 in the global data structure calib_param
}

//*********************************************************************************************************
// MS5803                                                                                               
// Function to request a PROM read of a certain calibration coefficient                                 
// Parameters:                                                                                           
//   >> coefficient_number: The number of the coefficient to be requested for reading (1 for C1, 2 for C2...etc)  
// Return: None                                                                                       
//////// Notes:
// This function can be used to request 16-bit data from the PROM on addresses 0 and 7 too
//*********************************************************************************************************
void MS_request_read_PROM_coef(uint8_t coefficient_number){ 
  coefficient_number = coefficient_number << 1; // Coefficient number represents the 3-bit address in the PROM
  write_byte_on_bus(MS5803_ADDRESS, PROM_READ | coefficient_number);
}

//*********************************************************************************************************
// MS5803
// Function to read the factory calibration coefficient after it is requested using MS_request_read_PROM_coef function
// Parameters: 
//   >> coefficient_number: The number of the coefficient to be read after being requested (1 for C1, 2 for C2...etc)
// Return: None
//////// Notes:
// This function can be used to read an already requested 16-bit data from the PROM on addresses 0 and 7 too
//*********************************************************************************************************
uint32_t MS_read_coef_from_bus(uint8_t coefficient_number){
  uint8_t MSB_byte, LSB_byte;
  uint32_t coefficient;
  
  MS_request_read_PROM_coef(coefficient_number);
    
  Wire.requestFrom(MS5803_ADDRESS,2);  
  while(Wire.available() == 0){}
  MSB_byte = Wire.read(); // ATTENTION, MSB byte is returned first
  LSB_byte = Wire.read();
  coefficient = (MSB_byte << 8) | LSB_byte;
  return coefficient;
}

//*********************************************************************************************************
// MS5803
// Function to read the pressure 24 bits from the ADC, the function handles all the pressure reading stages (Conversion request, ADC read request, ADC reading)
// Parameters:
//   >> oversampling_ratio_OSR: A factor related to the resolution of the ADC, values are (256, 512, 1024, 2048, 4096) higher OSR correspond to higher resolution
// Return: Raw pressure value as read from the ADC
//*********************************************************************************************************
uint32_t MS_read_pressure(int oversampling_ratio_OSR){ // OSR defines the resolution
  uint16_t conversion_command; // Conversion command is the command that is to be sent to the sensor to perform a conversion with a certain OSR
  uint8_t MSB_press_byte, MediumSB_press_byte, LSB_press_byte; 
  uint32_t pressure;
  int max_conversion_time; // Check the datasheet, page 2 of 18 for the conversion times corresponding to different OSR (time is rounded)

  switch(oversampling_ratio_OSR){ // Choosing the ADC conversion time to fit the OSR chosen
    case 256  :
      conversion_command = CONVERT_1_256;
      //max_conversion_time = MAX_CONV_TIME_256;
      break;
    case 512  :
      conversion_command = CONVERT_1_512;
      //max_conversion_time = MAX_CONV_TIME_512;
      break;
    case 1024 :
      conversion_command = CONVERT_1_1024;
      //max_conversion_time = MAX_CONV_TIME_1024;
      break;
    case 2048 :
      conversion_command = CONVERT_1_2048;
      //max_conversion_time = MAX_CONV_TIME_2048;
      break;
    case 4096 :
      conversion_command = CONVERT_1_4096;
      //max_conversion_time = MAX_CONV_TIME_4096;
      break;
    default   :
      conversion_command = CONVERT_1_4096;
      //max_conversion_time = MAX_CONV_TIME_4096;
  }
  //Serial.print(" PBR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  
  write_byte_on_bus(MS5803_ADDRESS, conversion_command); // Send the conversion command to the ADC 
  delay(SAFE_MAX_CONV_TIME); // Currently always using the safe maximum conversion time because of discrepancy between physical device and datasheet 
  
  write_byte_on_bus(MS5803_ADDRESS, ADC_READ); // Send the ADC read command  

  //Serial.print("Requested P..."); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  
  pressure = MS_read_ADC_bytes(); // Read the 24-bit result
  //Serial.println("P is read"); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  //Serial.print(" PAR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  
  return pressure; // Return the pressure value read
}

//*********************************************************************************************************
// MS5803
// Function to read the temperature 24 bits from the ADC, the function handles all the temperature reading stages (Conversion request, ADC read request, ADC reading)
// Parameters:
//   >> oversampling_ratio_OSR: A factor related to the resolution of the ADC, values are (256, 512, 1024, 2048, 4096) higher OSR correspond to higher resolution
// Return: Raw temperature value as read from the ADC
//*********************************************************************************************************
uint32_t MS_read_temperature(int oversampling_ratio_OSR){
  uint16_t conversion_command; // Conversion command is the command that is to be sent to the sensor to perform a conversion with a certain OSR
  uint8_t MSB_temp_byte, MediumSB_temp_byte, LSB_temp_byte; 
  uint32_t temperature;
  int max_conversion_time; // Check the datasheet, page 2 of 18 for the conversion times corresponding to different OSR (time is rounded)
  
  switch(oversampling_ratio_OSR){ // Choosing the ADC conversion time to fit the OSR chosen
    case 256  :
      conversion_command = CONVERT_2_256;
      //max_conversion_time = MAX_CONV_TIME_256;
      break;
    case 512  :
      conversion_command = CONVERT_2_512;
      //max_conversion_time = MAX_CONV_TIME_512;
      break;
    case 1024 :
      conversion_command = CONVERT_2_1024;
      //max_conversion_time = MAX_CONV_TIME_1024;
      break;
    case 2048 :
      conversion_command = CONVERT_2_2048;
      //max_conversion_time = MAX_CONV_TIME_2048;
      break;
    case 4096 :
      conversion_command = CONVERT_2_4096;
      //max_conversion_time = MAX_CONV_TIME_4096;
      break;
    default   :
      conversion_command = CONVERT_2_4096;
      //max_conversion_time = MAX_CONV_TIME_4096;
  }
  //Serial.print(" TBR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  // Send the conversion command 
  write_byte_on_bus(MS5803_ADDRESS, conversion_command);
  delay(SAFE_MAX_CONV_TIME); // Currently always using the safe maximum conversion time because of discrepancy between physical device and datasheet 
  
  write_byte_on_bus(MS5803_ADDRESS, ADC_READ); // Send the ADC read command 

  //Serial.print("Requested T..."); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  
  temperature = MS_read_ADC_bytes(); // Read the 24-bit result
  //Serial.println("T is read"); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
  //Serial.print(" TAR "); ///////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  return temperature; // Return the temperature value read
}

//*********************************************************************************************************
// MS5803 
// Function to read the 24 bits from the ADC (the conversion result) after being requested by an ADC conversion command
// Parameters: None
// Return: The ADC conversion result (24 bits contained in a uint32_t variable)
//*********************************************************************************************************
uint32_t MS_read_ADC_bytes(){
  uint8_t MSB_ADC_byte, MediumSB_ADC_byte, LSB_ADC_byte; 
  uint32_t ADC_data;
  unsigned long start_time;
  
  start_time = millis();
  
  //Serial.print("Requested ADC reading..."); ///////////////////////////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  Wire.requestFrom(MS5803_ADDRESS,3);  
  while(Wire.available() == 0){
    //Serial.println("Waiting for ADC"); ///////////////////////////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    if( (millis() - start_time) > MS_ADC_READ_TIMEOUT ){
      Serial.println("Error 1>>>>>>>>>"); 
      break;
    }
  } 
  MSB_ADC_byte = Wire.read(); // ATTENTION, MSB byte is returned first
  MediumSB_ADC_byte = Wire.read(); // Middle pressure byte
  LSB_ADC_byte = Wire.read(); // LSB pressure byte
  
  ADC_data = ((MSB_ADC_byte << 16) | (MediumSB_ADC_byte << 8) | (LSB_ADC_byte)) & 0x00FFFFFF;
  //Serial.println("ADC reading finished");
  return ADC_data;
} 
//********************************************************
// SERIAL COMMUNICATION functions
//********************************************************
//*********************************************************************************************************
// SERIAL
// Function that returns an integer value corresponding the the command recieved from MATLAB 
// See definitions for the integer values allocated to commands
// Stores parameters of received command in the appropriate structures
// Parameters: None
// Return: The integer value corresponding to the received command
////////Notes:
// The function only parses the command and its parameters, the actions following a certain command 
// are to be conducted in the loop based on the last command received
//*********************************************************************************************************
int parse_command(){
  int parsed_command; // The integer equivalent of the parsed command
  
  while(Serial.available() == 0){} // Wait for MATLAB command 
  
  raw_command =  Serial.readStringUntil(' '); // Keep reading the input until a space is seen
  
  // Figure out what command is sent and store the parameters based on it, only data from MATLAB is dealt with here
  // Data needed to be obtained from sensors is obtained in the loop or other functions
  if(raw_command == "G0"){ // Stepper to move slowly to target position
    parsed_command = SLOW_MOVE; 
    read_parameters_SLOW_MOVE();
  }else if(raw_command == "G1"){ // Stepper to move quickly to target position
    parsed_command = FAST_MOVE;
    read_parameters_FAST_MOVE();
  }else if(raw_command == "G2"){ // Stepper to execute homing
    parsed_command = HOMING;
  }else if(raw_command == "C0"){ // Stepper and sensors to execute calibration (data transmission to MATLAB needed)
    parsed_command = CALIBRATION;
    read_parameters_CALIBRATION();
  }else if(raw_command == "D0"){ // Stepper to execute dynamic calibration (data transmission to MATLAB needed)
    parsed_command = DYNAMIC;
    read_parameters_DYNAMIC();
  }else if(raw_command == "M0"){ // MS5803 to send its coefficients (data transmission to MATLAB needed)
    parsed_command = CALIBRATION_DATA; // Get the calibration coefficients read from the MS5803 not here but in the loop    
  }else{ // Typically if the parsed command had an error (parsed_command = COMMAND_ERROR)
    parsed_command = COMMAND_ERROR; // The parsed command is unknown, so an error must have happened
    // TO DO: send an error message to MATLAB ###############
  }
  return parsed_command;
}

//*********************************************************************************************************
// SERIAL
// Function to read SLOW_MOVE parameters (desired_position to move to) and 
// store them in the appropriate structure
// Parameters: None
// Return: None
//*********************************************************************************************************
void read_parameters_SLOW_MOVE(){
  // Read the desired position and store it in the SLOW_MOVE_param global structure
  SLOW_MOVE_param.desired_position = Serial.readStringUntil(' ').toInt();
}

//*********************************************************************************************************
// SERIAL
// Function to read FAST_MOVE parameters (desired_position to move to) and 
// store them in the appropriate structure
// Parameters: None
// Return: None
//*********************************************************************************************************
void read_parameters_FAST_MOVE(){
  // Read the desired position and store it in the FAST_MOVE_param global structure
  FAST_MOVE_param.desired_position = Serial.readStringUntil(' ').toInt();
} 
  
//*********************************************************************************************************
// SERIAL
// Function to read CALIBRATION parameters (start_position, end_position, increment amount, 
// and the wait between increments) and 
// store them in the appropriate structure
// Parameters: None
// Return: None
//*********************************************************************************************************
void read_parameters_CALIBRATION(){  
  CALIBRATION_param.start_position = Serial.readStringUntil(' ').toInt(); // Read the start position  
  CALIBRATION_param.end_position = Serial.readStringUntil(' ').toInt(); // Read the end position
  CALIBRATION_param.increments = Serial.readStringUntil(' ').toInt(); // Read the increments  
  CALIBRATION_param.wait = Serial.readStringUntil(' ').toInt(); // Read the wait
} 
  
//*********************************************************************************************************
// SERIAL
// Function to read the DYNAMIC calibration parameters (start_position, end_position, and frequency) and 
// store them in the appropriate structure
// Parameters: None
// Return: None
//*********************************************************************************************************
void read_parameters_DYNAMIC(){
  DYNAMIC_param.start_position = Serial.readStringUntil(' ').toInt(); // Read the start position
  DYNAMIC_param.end_position = Serial.readStringUntil(' ').toInt(); // Read the end position
  DYNAMIC_param.frequency = Serial.readStringUntil(' ').toInt(); // Read the frequency
} 
  
//*********************************************************************************************************<<<<<<<<<<<<<<<<<<<<<<<<
// SERIAL
// Function to send the stored MS5803 calibration coefficients to MATLAB serially
// Parameters: None
// Return: None
//*********************************************************************************************************
void send_MATLAB_CALIBRATION_DATA(){
} 
  
//*********************************************************************************************************
// SERIAL
// Function to initialize communication to MATLAB, it keeps sending an 'a' every 200ms until it receives 
// an 'a' back from MATLAB, thus acknowledging the beginning of communications
// Parameters: None 
// Return: None
////////Notes:
// Function doesn't time out, so it will keep looping forever sending an a every 200ms until an a is sent back
// TO DO: A good idea is to set a global variable that the function sets when connection is established ######################
//*********************************************************************************************************
void initialize_MATLAB_communication(){ 
  int time_out_loops; // number of loops before sending another 'a'
  time_out_loops = INIT_COMM_TIMEOUT_LP; // After 10 loops of 20ms we declare a time out and send an 'a' again (INIT_COMM_TIMEOUT = 10)
  
  while(!is_connected_to_MATLAB){ // Loop until we are connected to MATLAB      
    Serial.print('a'); // Send the declaration letter to MATLAB    
     
    while(Serial.available() == 0){ // Keep waiting for the response from MATLAB until time-out or data is received      
      if(time_out_loops == 0){ // Break if timed out to send another 'a' to MATLAB
        break;
      } // end if
      delay(INIT_COMM_TIMEOUT); // Delay by 20ms everytime we loop 
      time_out_loops--;
    } // end while
  
    time_out_loops = INIT_COMM_TIMEOUT_LP; // Reset time-out
    
    if(Serial.available() != 0){ // Check if we broke from the while loop because something is available on the serial port
      if(Serial.read() == 'a'){ // If the byte that is available at the buffer is an 'a', then we got acknowledgement from MATLAB
        is_connected_to_MATLAB = true; //   
      } // end if
    } // end if    
  } // end while
  is_connected_to_MATLAB = false; // Reset the boolean (not necessary since for the meantime this function is used only once in the life time of the program)
} 
//*********************************************************************************************************
// SERIAL
// Function to send an integer to MATLAB after converting it to a string and concatenating it with a character
// Parameters: 
//   >> data_label_character: A character defining the data that is to be sent after ()
//   >> data_integer: Integer holding the data to be sent to MATLAB 
// Return: None
////////Notes:
// This function is mainly used to send the capacitance, pressure, and temperature data to MATLAB
//*********************************************************************************************************
void send_MATLAB_integer_data(char data_label_character, uint32_t data_integer){
  uint8_t buffer[4];
  buffer[0] = data_integer & 0x000000ff;
  buffer[1] = (data_integer >> 8) & 0x000000ff;
  buffer[2] = (data_integer >> 16) & 0x000000ff;
  buffer[3] = (data_integer >> 24) & 0x000000ff;
  //Serial.print(data_label_character);
  Serial.print(data_integer);
  //Serial.write(data_label_character); // Send the defining label to MATLAB
  //Serial.print(buffer[0]);//added for debugging
  //Serial.print(buffer[1]);//added for debugging
  //Serial.print(buffer[2]);//added for debugging
  //Serial.println(buffer[3]);//added for debugging
  //Serial.println(data_integer);//added for debugging
  //Serial.write(buffer,4); // Follow the label by the corresponding data
}    
//*********************************************************************************************************
// SERIAL 
// Function to transmit the data to MATLAB while conducting a delay for a certain amount of time
// Parameters: 
//   >> total_wait_period: The period of the delay to be conducted
//   >> maximum_data_conversion_wait: The maximum interval of time between two 
//      data points (a property of the devices on the bus and their ADCs) 
// Return: None
////////Notes:
// The conversion mode for the AD7162 assumed in this function is the Single Conversion mode
// The general assumption in this function is that the conversion time chosen for the AD7152 is 
// longer (60ms) than the conversion time of the temperature (10ms) and pressure (10ms) from the MS5803
// It is advised that the maximum_data_conversion_wait to be the actual conversion time plus several milliseconds for safety
//*********************************************************************************************************
void transmit_data_and_wait(unsigned long total_wait_period, unsigned long maximum_data_conversion_wait, int AD_channel_number, int OSR_temp, int OSR_press){
  unsigned long start_time; // The time at which the wait and transmit algorithm is to start
  boolean test_boolean = true; ///////////////////////////////////////////////////////////////////////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>
  // Keep looping until reaching the point in time where if you do the next conversion 
  // you will exceed the specified wait time
  start_time = millis(); // Store the start time 
  // Start timer here
  start_timer();
  //pinMode(SLEEEP_PIN, );  
  while(total_wait_period - (millis() - start_time) >  maximum_data_conversion_wait){
    // AD_request_capacitance(AD_channel_number); // Function to request data from the AD7152 
    if(!is_I2C_reading_timed_out){ // In case a time_out event has occured skip the current reading 
      send_MATLAB_integer_data('P', MS_read_pressure(OSR_press)); // Send MATLAB the pressure value you read
    }
    Serial.print(' ');
    if(!is_I2C_reading_timed_out){ // In case a time_out event has occured skip the current reading 
      send_MATLAB_integer_data('T', MS_read_temperature(OSR_temp)); // Function to read the temperature from the AD7152
    }    
    Serial.print(' ');
    if(!is_I2C_reading_timed_out){ // In case a time_out event has occured skip the current reading 
      send_MATLAB_integer_data('C', AD_read_capacitance(1)); // Send temperature data here   
    }
    Serial.print(' ');
    send_MATLAB_integer_data('L', position_param.current_position_in_steps); ///////////////////////////////////////////////////////////////////////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>
    data_acq_counter++;
    Serial.print(' ');
    //Serial.print('N');
    Serial.print(data_acq_counter);
    Serial.println(' '); ///////////////////////////////////////////////////////////////////////////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>
    // Clear timer here
    clear_timer();
    // Clear the is_I2C_reading_timed_out since a successful reading is finished
    is_I2C_reading_timed_out = false;
  } 
  // Stop timer here after the wait period is finished
  stop_timer();
  //disable_timer_interrupt();
  //pinMode(SLEEEP_PIN, );
  // If broke out of the loop before total_wait_period is complete, loop until it is complete
  while(total_wait_period - (millis() - start_time) > 0){}
} 
  
//*********************************************************************************************************
//*********************************************************************************************************
void TIMER0_IRQHandler(){
  if(NRF_TIMER0 -> EVENTS_COMPARE[0] != 0){
    NRF_TIMER0 -> EVENTS_COMPARE[0] = 0;
  }
  // Close the I2C bus by sending a stop condition
  Wire.endTransmission();
  delay(20); // Give the bus time to breath
  Serial.println("ERR");
  // For skipping the current I2C reading we use a global boolean that will be set in the interrupt method of the 
  //   timer indicating a time-out has occured 
  is_I2C_reading_timed_out = true;
  //stop_timer();  
}

//*********************************************************************************************************
// SERIAL
//*********************************************************************************************************
void send_MS_calibration_coefficients(){
  delay(1000);
  //Serial.print('1');
  Serial.print(' ');
  Serial.print(calib_param.c1);
  //delay(1000);
  //Serial.print('2');
  Serial.print(' ');
  Serial.print(calib_param.c2);  
  //delay(1000);
  //Serial.print('3');
  Serial.print(' ');
  Serial.print(calib_param.c3);
  //delay(1000);
  //Serial.print('4');
  Serial.print(' ');
  Serial.print(calib_param.c4);
  //delay(1000);
  //Serial.print('5');
  Serial.print(' ');
  Serial.print(calib_param.c5);
  //delay(1000);
  //Serial.print('6');
  Serial.print(' ');
  Serial.print(calib_param.c6);
}

//********************************************************
// STEPPER functions
//********************************************************
//*********************************************************************************************************
// STEPPER
// Function to move the stepper motor to a desired position 
// Parameters:
//   >> pulse_period: The period to wait between sending the step pulses to the driver
//   >> desired_position: The position to be travelled to in step numbers
// Return: None
////////Notes:
// This function doesn't account for special cases such as the case when the current position stored in memory is wrong
//*********************************************************************************************************
void STEPPER_move_to_point(int pulse_period, int desired_position){
  int displacement; // The algebraic difference between the current position and the desired point in steps
  
  position_param.is_homed = false; // Motor status as 'homed' is invalidated at the moment a movement by the stepper is made
  displacement = (desired_position) - (position_param.current_position_in_steps); // Calculate displacement
  
  if (displacement > 0){ // Displacement is positive => desired position > current position => move forward
    digitalWrite(DIR_pin, DIR_forward_pin_state); // Change the DIR_pin state so that we move forward
    while(displacement > 0){ // Loop until the desired position is reached
      STEPPER_send_step_to_driver(); // Send a step pulse to the driver
      displacement--; // Decrement displacement for everytime we move forward towards the target position
      position_param.current_position_in_steps++; // Increment the current position since we are moving forwards [Keep track of current position]
      delay(pulse_period); // Delay the next step pulse by the required period
    }    
  }else if(displacement < 0){ // Displacement is negative => desired position < current position => move backward
    digitalWrite(DIR_pin, DIR_backward_pin_state); // Change the DIR_pin state so that we move backwards
    while(displacement < 0){
      STEPPER_send_step_to_driver(); // Send a step pulse to the driver
      displacement++; // Increment displacement for everytime we move forward towards the target position
      position_param.current_position_in_steps--; // Decrement the current position since we are moving backwards [Keep track of current position]
      delay(pulse_period); // Delay the next step pulse by the SLOW MOVE period
    }
  }  
}

//*********************************************************************************************************
// STEPPER
// Moves by a certain nember of steps
// Parameters:
//   >> pulse_period: The period to wait between sending the step pulses to the driver
//   >> number_of_steps: Number of steps to be moved (forward or backwards)
//   >> is_forward: True => move forward, False => move backwards
// Returns: None
////////Notes:
// This function has no error or safety checking procedure, it can move until it hits the boundaries. It must be used cautiously.
//*********************************************************************************************************
void STEPPER_move_step_number(int pulse_period, int number_of_steps, boolean is_forward){

  position_param.is_homed = false; // Motor status as 'homed' is invalidated at the moment a movement by the stepper is made

  if(is_forward){ // Check if the number of steps is to be moved backwards or forwards
     digitalWrite(DIR_pin, DIR_forward_pin_state); // Steps are to be moved forward
     while(number_of_steps > 0){ // Keep looping until the number of steps is equal to zero
       STEPPER_send_step_to_driver(); // Send a step command to the stepper driver
       delay(pulse_period); // Wait for a certain amount corresponding to the motion mode
       position_param.current_position_in_steps++; // Update position (Increment steps, moving forwards)
       number_of_steps--; // Decrement the number of steps
     }
  }else{
     digitalWrite(DIR_pin, DIR_backward_pin_state); // Steps are to be moved backwards
     while(number_of_steps > 0){ // Keep looping until the number of steps is equal to zero
       STEPPER_send_step_to_driver(); // Send a step command to the stepper driver
       delay(pulse_period); // Wait for a certain amount corresponding to the motion mode
       position_param.current_position_in_steps--; // Update position (decrement steps, moving backwards)
       number_of_steps--; // Decrement the number of steps
     }
  }
}

//*********************************************************************************************************
// STEPPER
// Function to execute the homing procedure (executes homing to the minimum position)
// Parameters: None
// Return: None 
//*********************************************************************************************************
void STEPPER_execute_HOMING(){
  int step_count = 0; // Integer to keep track of the steps travelled 
  
  digitalWrite(DIR_pin, DIR_backward_pin_state); // Change the DIR_pin so the stepper is moving backwards
  position_param.is_homed = is_touch_sensor(); // Check if the touch sensor is already touched before homing is started
  while(!position_param.is_homed){ // Keep looping until we are homed
    //Serial.println("Homing loop..."); /////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  
    STEPPER_send_step_to_driver(); // Send a step to the driver 
    position_param.is_homed = is_touch_sensor(); // Check if the touch sensor is touched
    position_param.current_position_in_steps--; // Update the current position
    step_count++; // Increment the step count
    if(step_count == MAX_STEP_POSITION){ // If for some reason the count is larger than the maximum count => stop the homing and generate an error message
      // TO DO: Generate an error message to MATLAB#######################
      break;
    }
    //Serial.println("Before delay...");
    delay(HOMING_MOVE_STEP_PERIOD); // Delay by a certain amount corresponding to the homing speed
    //Serial.println("After delay...");
  }
  position_param.current_position_in_steps = 0;// Zero the position when we hit the touch sensor (homing is finished)
  //Serial.println("Homing loop...DONE"); /////////////////////////////////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  

}

//*********************************************************************************************************
// STEPPER, MS5803, AD715X
// Function to perform the whole calibration process
// Parameters: None
// Return: None
////////Notes:
// This function must control the stepper, stream data points from the MS5803 and the AD7152 all at the same time.
// The function takes the data points in the wait interval, not while in motion.
//*********************************************************************************************************
void execute_CALIBRATION(){
  int minimum_data_conversion_wait; 
  
  // Find the minimum data conversion interval
  if(AD7152_param.conversion_time_ms < SAFE_MAX_CONV_TIME){
    minimum_data_conversion_wait = SAFE_MAX_CONV_TIME;
  }else{
    minimum_data_conversion_wait = AD7152_param.conversion_time_ms;
  }
  // Send an error message to MATLAB in case the wait is less than the minimum_data_conversion_wait
  // // TO DO: Generate an error message #######################
    
  // Perform homing 
  //STEPPER_execute_HOMING();
  // Move to the initial position
  STEPPER_move_to_point(CALIBRATION_STEP_PERIOD, CALIBRATION_param.start_position); 
  // Initialize the timer before you start the wait and transmit sequence
  init_timer(2*CALIBRATION_param.wait); // Initialize TIMER0 and have it trigger an interrupt at double the wait time indicated in the calibration parameters
  data_acq_counter = 0; 
  
  // Increment the steps by the "increment" parameter and keep moving until you reach the final position 
  while((CALIBRATION_param.end_position - position_param.current_position_in_steps) > 0){ 
    STEPPER_move_step_number(CALIBRATION_STEP_PERIOD ,CALIBRATION_param.increments ,true); // Move the number of steps required 
     
    transmit_data_and_wait(CALIBRATION_param.wait, minimum_data_conversion_wait, 1, 4096, 4096); // Perform the wait between increments and send data then
    
    // Check if moving the next step would get us beyond the maximum distance limit
    
    if( CALIBRATION_param.increments > (MAX_STEP_POSITION - position_param.current_position_in_steps) ){
      CALIBRATION_param.increments = (MAX_STEP_POSITION - position_param.current_position_in_steps) - MAX_POSITION_SAFETY_MARGIN; // The final increment is reduced to a value that doesn't make it hit the maximum margin
      CALIBRATION_param.end_position = MAX_STEP_POSITION - MAX_POSITION_SAFETY_MARGIN; // The end positin is changed such that only one more increment is performed
    }
    
  }
}

//*********************************************************************************************************
// STEPPER
// Function to send a step command to the stepper driver
// Parameters: None
// Return: None                        
//*********************************************************************************************************
void STEPPER_send_step_to_driver(){
  digitalWrite(STEP_pin, HIGH); // Send a HIGH pulse
  delay(STEP_PULSE_HIGH_PERIOD); // Remain HIGH for a time equal to STEP_PULSE_HIGH_PERIOD
  digitalWrite(STEP_pin, LOW); // Send a LOW pulse
  delay(STEP_PULSE_LOW_PERIOD); // Remain LOW for a time equal to STEP_PULSE_HIGH_PERIOD
}

//*********************************************************************************************************<<<<<<<<<<<<<<<<<<<<<<<<
// STEPPER
// Function to check if the touch sensor is activated (by being touched by the stepper)
// Parameters: None
// Return: (Touch sensor compressed => True, Touch sensor released => False)
//*********************************************************************************************************
boolean is_touch_sensor(){ // Needs to be checked, not final
  if(digitalRead(TOUCH_SENSOR_pin) == LOW){
    return true;  
  }else if(digitalRead(TOUCH_SENSOR_pin) == HIGH){
    return false;
  }  
}

//*********************************************************************************************************
// TIMER
//*********************************************************************************************************
void disable_timer_interrupt(){
  NVIC_DisableIRQ(TIMER0_IRQn);
}
//*********************************************************************************************************
// TIMER
//*********************************************************************************************************
void start_timer(){
  NRF_TIMER0 -> TASKS_START = 1; // Start the timer
}
//*********************************************************************************************************
// TIMER
//*********************************************************************************************************
void stop_timer(){
  NRF_TIMER0 -> TASKS_STOP = 1;
}
//*********************************************************************************************************
// TIMER
//*********************************************************************************************************
void clear_timer(){
  NRF_TIMER0 -> TASKS_CLEAR = 1;
}
//*********************************************************************************************************
// TIMER
// time_out is the amount of time after which the timer saturates and triggers an interrupt
// in the current setup time out is given in ms and converted into us inside the init_timer function
//*********************************************************************************************************
void init_timer(int time_out){
  NVIC_EnableIRQ(TIMER0_IRQn); // Enable interrupts from timer
  
  NRF_TIMER0 -> TASKS_STOP = 1; // Stop the tasks before configuring the timer
  NRF_TIMER0 -> MODE = 0x00000000; // Mode is timer (not counter)
  NRF_TIMER0 -> BITMODE = 0x00000003; // Bit width is 32 bits  
  NRF_TIMER0 -> PRESCALER = 0x00000004; // Prescaler value is 4 so the frequency is 1MHz
  NRF_TIMER0 -> TASKS_CLEAR = 1; // Clear timer to zero
  NRF_TIMER0 -> CC[0] = time_out*1000; // Compare to the given time value
  NRF_TIMER0 -> INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; // Enable the 
  NRF_TIMER0 -> SHORTS = 0x00000001; // Connect the compare event to clear task so that it is cleared every time it reaches the predetermined wait value
}
