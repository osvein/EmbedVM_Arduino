#include <EEPROM.h>

#include "embedvm.h"

#define UNUSED __attribute__((unused))

#define MEM_SIZE 1*1024 //bytes of RAM allocated to the VM
#define PROG_LENGTH 128 //bytes of instruction read by the VM
#define PROG_OFFSET 0 //address of program in EEPROM
#define PROG_START 0 //address of first instruction (relative to PROG_OFFSET
#define BAUDRATE 9600 //UART baudrate
#define VERBOSE false //verbose mode

boolean stop = false;
byte memory[MEM_SIZE];

//put in seperate namespace to prevent Arduino IDE from preprocessing it; Arduino preprocessor is ST00PID
namespace no {
  struct embedvm_s vm = {
    0xffff, 0, 0, NULL,
    &mem_read, &mem_write, &call_user
  };
}

void setup()
{
  Serial.begin(BAUDRATE);
  for(int addr = 0; addr < PROG_LENGTH; addr++)
    memory[addr] = EEPROM.read(addr + PROG_OFFSET);
  embedvm_interrupt(&no::vm, PROG_START);
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
    Serial.print("SFP: " + String(no::vm.sfp)); //print the address of the first local variable on the stack
    Serial.println("");
    Serial.flush();
  }

  embedvm_exec(&no::vm); //execute next instruction
}


static int16_t mem_read(uint16_t addr, bool is16bit, void *ctx UNUSED)
{
  if (is16bit)
    return (memory[addr] << 8) | memory[addr+1];
  return memory[addr];
}

static void mem_write(uint16_t addr, int16_t value, bool is16bit, void *ctx UNUSED)
{
  if (is16bit) {
    memory[addr] = value >> 8;
    memory[addr+1] = value;
  } 
  else
    memory[addr] = value;
}

static int16_t call_user(uint8_t funcid, uint8_t argc, int16_t *argv, void *ctx UNUSED)
{
  int16_t ret = 0;
  int i;

  if (funcid == 0) {
    stop = true;
    Serial.println("Called user function 0 => stop.");
    Serial.flush();
    return ret;
  }

  Serial.print("Called user function " + String(funcid) + " with " + String(argc) + " args:");

  for (i = 0; i < argc; i++) {
    Serial.print(" " + argv[i]);
    ret += argv[i];
  }

  Serial.println("");
  Serial.flush();

  return ret ^ funcid;
}


