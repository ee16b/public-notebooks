/*
 * dynamics_data.ino
 *
 * Collects dynamics data (position) with
 * some varying (bounded) input PWM
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2019
 * Mia Mirkovic
 *
 * EE16B Spring 2021
 * Hossein Najafi
 */
#include <EEPROM.h>
#define MOTOR                  9
#define RED_LED				   12

#define GREEN_LED		   13
#define my_delay 			100
int pwm = 0;
int dir = 1;
const int ARRAY_SIZE = 2;
const int STARTING_EEPROM_ADDRESS = 105;
unsigned long int numbers[ARRAY_SIZE] = {  0xbfc00000,0x40600000   }; //
unsigned long int newNumbers[ARRAY_SIZE];

void writeIntArrayIntoEEPROM(int address, unsigned long int numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++) 
  {
    EEPROM.write(addressIndex + 0, (numbers[i] & 0xFF000000) >> 24); //
    EEPROM.write(addressIndex + 1, (numbers[i] & 0x00FF0000) >> 16); //
    EEPROM.write(addressIndex + 2, (numbers[i] & 0x0000FF00) >> 8); //
    EEPROM.write(addressIndex + 3, (numbers[i] & 0x000000FF)); //
    addressIndex += 4;
  }
}

void readIntArrayFromEEPROM(int address, unsigned long int my_numbers[], int arraySize)
{
  int addressIndex = address;
  for (int i = 0; i < arraySize; i++)
  {
    my_numbers[i]  = (unsigned long int)(EEPROM.read(addressIndex))<<24;
    my_numbers[i] =  my_numbers[i] + ((unsigned long int)(EEPROM.read(addressIndex + 1)) << 16);
    my_numbers[i] =  my_numbers[i] +  ((unsigned long int)(EEPROM.read(addressIndex + 2)) << 8);
    my_numbers[i] =  my_numbers[i] +  (unsigned long int)EEPROM.read(addressIndex + 3);
    addressIndex += 4;
  }
}

void setup(void) {
  Serial.begin(38400);
  float f;
  
  writeIntArrayIntoEEPROM(STARTING_EEPROM_ADDRESS, numbers, ARRAY_SIZE);
    
  readIntArrayFromEEPROM(STARTING_EEPROM_ADDRESS, newNumbers, ARRAY_SIZE);
  for (int i=0;i<ARRAY_SIZE;i++)
  {
    
    Serial.print(newNumbers[i],HEX);
	Serial.print("\n");
  }
  Serial.print("Readout Done\n");
  
  f = *((float *)&(newNumbers[1]));
  Serial.print(*(&f));
  Serial.print("\n");
}

void loop(void) {

  
}


