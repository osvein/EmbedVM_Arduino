#include <EEPROM.h>

#include "embedvm.h"

#define UNUSED __attribute__((unused))

#define MEMORY_SIZE 1*1024 //bytes of RAM allocated to the VM
#define PROGRAM_LENGTH 128 //bytes of instruction read by the VM
#define PROGRAM_OFFSET 0 //addressess of program in EEPROM
#define PROGRAM_ENTRY 0 //addressess of first instruction (relative to PROGRAM_OFFSET

#define BAUDRATE 9600 //UART baudrate
#define VERBOSE false //verbose mode

boolean stop = false; //whether the VM should skip execution of program
byte memory[MEMORY_SIZE]; //VM memory

//put in seperate namespace to prevent Arduino IDE from preprocessing it; Arduino preprocessor is ST00PID
namespace no {
  struct embedvm_s vm = {
    0xffff, 0, 0, NULL,
    &memoryRead, &memoryWrite, &callUserFunction
  };
}

void setup()
{
  Serial.begin(BAUDRATE);
  
  //copy program from EEPROM to RAM; EmbedVM can read program from RAM only
  for(int address = 0; address < PROGRAM_LENGTH; address++)
    memory[address] = EEPROM.read(address + PROGRAM_OFFSET);
  embedvm_interrupt(&no::vm, PROGRAM_ENTRY); //start EmbedVM
}

void loop()
{
  if (stop) return; //skip if stopped

  //terminate if main function returns
  if (no::vm.ip = 0xFFFF)
  {
    Serial.println("Main function returned => Terminating.");
    if (no::vm.sp != 0 || no::vm.sfp != 0)
      Serial.println("Unexpected stack configuration on program exit: SP=" + String(no::vm.sp) + ", SFP=" + String(no::vm.sfp));
    Serial.flush();
    stop = true;
  }

  //verbose
  if (VERBOSE)
  {
    Serial.print("IP: " + String(no::vm.ip) + " (" + String(memory[no::vm.ip]) + " " + String(memory[no::vm.ip + 1]) + " " + String(memory[no::vm.ip + 2]) + " " + String(memory[no::vm.ip + 3]) + "),  "); //print next instruction to execute
    Serial.print("SP: " + String(no::vm.sp) + " (" + String(memory[no::vm.sp]) + String(memory[no::vm.sp + 1]) + " " + String(memory[no::vm.sp + 2]) + String(memory[no::vm.sp + 3]) + " " + String(memory[no::vm.sp + 4]) + String(memory[no::vm.sp + 5]) + " " + String(memory[no::vm.sp + 6]) + String(memory[no::vm.sp + 7]) + "),  "); //print last pushed words on the stack
    Serial.print("SFP: " + String(no::vm.sfp)); //print the addressess of the first local variable on the stack
    Serial.println("");
    Serial.flush();
  }

  embedvm_exec(&no::vm); //execute next instruction
}

//callback used by EmbedVM to read memory
static int16_t memoryRead(uint16_t address, bool is16bit, void *ctx UNUSED)
{
  if (is16bit)
    return (memory[address] << 8) | memory[address+1];
  return memory[address];
}

//callback used by EmbedVM to write memory
static void memoryWrite(uint16_t address, int16_t value, bool is16bit, void *ctx UNUSED)
{
  if (is16bit) {
    memory[address] = value >> 8;
    memory[address+1] = value;
  } 
  else
    memory[address] = value;
}

//callback used by EmbedVM to call user functions
static int16_t callUserFunction(uint8_t function, uint8_t argc, int16_t *argv, void *ctx UNUSED)
{
  int16_t ret = 0;
  int i;

  //user function ID 0 = stop
  if (function == 0) {
    stop = true;
    Serial.println("Called user function 0 => stop.");
    Serial.flush();
    return ret;
  }

  //print user function call details (function ID, argument count, argument list)
  Serial.print("Called user function " + String(function) + " with " + String(argc) + " args:");
  for (i = 0; i < argc; i++) {
    Serial.print(" " + argv[i]);
    ret += argv[i];
  }

  Serial.println("");
  Serial.flush();

  return ret ^ function;
}


