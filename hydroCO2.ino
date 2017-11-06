/**
*  @file    CO2 Monitoring and ControlL.ino
*  @author  peter c
*  @date    11/3/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  NEOL Arduino Nano
** @section HISTORY
** 2017Oct25 - created
*/
#include <HardwareSerial.h>

#include <Streaming.h>
#include <DA_Analoginput.h>

#include <DA_DiscreteOutput.h>

#include <DA_NonBlockingDelay.h>


#define CO2_INTERRUPT_PIN 2 // pin does not work return 0 for now to host
#define ENABLE_CO2_SENSOR_RISING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN), on_B1N1_CO_AT_001_Rising, RISING)
#define ENABLE_CO2_SENSOR_FALLING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN), on_B1N1_CO_AT_001_Falling, FALLING)
#define DISABLE_CO2_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(CO2_INTERRUPT_PIN))


#include "unitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
#define ALARM_REFRESH_INTERVAL 10 // ms


DA_DiscreteOutput B1N1_CO_XY_001 = DA_DiscreteOutput(3, LOW); // V1
DA_DiscreteOutput B1N1_CO_XY_002 = DA_DiscreteOutput(11, LOW); // V2
DA_DiscreteOutput B1N1_CO_XY_003 = DA_DiscreteOutput(10, LOW); // V3
DA_DiscreteOutput B1N1_CO_XY_004 = DA_DiscreteOutput(9, LOW); // V4

DA_AnalogInput B1N1_CO_PT001 = DA_AnalogInput(A1, 0.0, 1023.); // min max
DA_AnalogInput B1N1_CO_PT002 = DA_AnalogInput(A2, 0.0, 1023.); // min max
DA_AnalogInput B1N1_CO_PT003 = DA_AnalogInput(A6, 0.0, 1023.); // min max
DA_AnalogInput B1N1_CO_PT004 = DA_AnalogInput(A7, 0.0, 1023.); // min max



// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay( 2000, &doOnPoll);

volatile unsigned long timeOnStart = 0 ;
volatile unsigned long timeOffStart = 0;
volatile unsigned long timeOn = 0 ;
volatile unsigned long timeOff = 0;

// HEARTBEAT
unsigned int heartBeat = 0;



#ifdef PROCESS_TERMINAL
HardwareSerial *tracePort = & Serial2;
#endif

void on_B1N1_CO_AT_001_Rising()
{

  unsigned long timestamp = micros();
  //timeCycle = micros();
  timeOnStart = timestamp;
  timeOff = timeOnStart - timeOffStart;
  ENABLE_CO2_SENSOR_FALLING_INTERRUPTS;
}


void on_B1N1_CO_AT_001_Falling()
{

  unsigned long timestamp = micros();

  timeOn = timestamp - timeOnStart;
  timeOffStart = timestamp; 
  ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}


void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

  randomSeed(analogRead(3));
ENABLE_CO2_SENSOR_RISING_INTERRUPTS;
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif
pollTimer.refresh();

}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
doReadAnalogs();
  heartBeat++;
}



void doReadAnalogs()
{
  B1N1_CO_PT001.refresh();
  B1N1_CO_PT002.refresh();
  B1N1_CO_PT003.refresh();
  B1N1_CO_PT004.refresh();

#ifdef TRACE_3NALOGS
  B1N1_CO_PT001.serialize(tracePort, true);
  B1N1_CO_PT002.serialize(tracePort, true);
  B1N1_CO_PT003.serialize(tracePort, true);
  B1N1_CO_PT004.serialize(tracePort, true);
#endif

}



// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{


  modbusRegisters[HR_PRESSURE1] = B1N1_CO_PT001.getRawSample();
  modbusRegisters[HR_PRESSURE2] = B1N1_CO_PT002.getRawSample();
  modbusRegisters[HR_PRESSURE3] = B1N1_CO_PT003.getRawSample();
  modbusRegisters[HR_DIFFDP1] = B1N1_CO_PT004.getRawSample();
  modbusRegisters[HR_HEARTBEAT] = heartBeat;

  modbusRegisters[B1N1_CO_AT_001_MB] = ( 2000 * ( timeOn - .002)/(timeOn+timeOff - .004) ) * 10;
}


bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkAndActivateDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->activate();

  #ifdef TRACE_MODBUS
    *tracePort << "Activate DO:";
    aDO->serialize(tracePort, true);
    LED.activate();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void checkAndResetDO(unsigned int bitOffset, DA_DiscreteOutput * aDO)
{
  // look for a change from 0 to 1
  if (!getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, bitOffset))
  {
    aDO->reset();

  #ifdef TRACE_MODBUS
    *tracePort << "Reset DO:";
    aDO->serialize(tracePort, true);
    LED.reset();
  #endif

   // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, bitOffset, false); // clear the bit
  }
}

void processValveCommands()
{
  checkAndActivateDO(VALVE1_OPEN_CLOSE, & B1N1_CO_XY_001);
  checkAndResetDO(VALVE1_OPEN_CLOSE, & B1N1_CO_XY_001);

  checkAndActivateDO(VALVE2_OPEN_CLOSE, & B1N1_CO_XY_002);
  checkAndResetDO(VALVE2_OPEN_CLOSE, & B1N1_CO_XY_002);

  checkAndActivateDO(VALVE3_OPEN_CLOSE, & B1N1_CO_XY_003);
  checkAndResetDO(VALVE3_OPEN_CLOSE, & B1N1_CO_XY_003);

  checkAndActivateDO(VALVE4_OPEN_CLOSE, & B1N1_CO_XY_004);
  checkAndResetDO(VALVE4_OPEN_CLOSE, & B1N1_CO_XY_004);
}

void processModbusCommands()
{
  processValveCommands();
}

#endif
