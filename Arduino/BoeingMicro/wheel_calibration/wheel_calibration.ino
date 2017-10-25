/*
 *  @brief Calibration script used to find the conversion value from rad/s to Kangaroo Units (KU). 
 *  This is done by measuring each wheel with a tachometer and entering in desired KU/sec into the serial monitor of the Arduino IDE.
 *  Adjust the inputs for the wheel until the tachometer reads about 60 RPM. Repeat for each wheel. 
 *  
 *  Note: When inputting desired KU speed values, add a period . to the end of the input. For some reason, Arduino does not like or know how to take in return characters, so this is a work around that.
 */


#include <Kangaroo.h>

// Kangaroo Motion Controller Library and Definitions
//   - Independent mode channels on Kangaroo are, by default, '1' and '2'.
//   - Serial1 Pins: 18 - TX (White), 19 - RX (Red)
//   - Serial2 Pins: 16 - TX (White), 17 - RX (Red)
#define TX_PIN_1 16
#define RX_PIN_1 17
#define TX_PIN_2 18
#define RX_PIN_2 19


// Setting up Kangaroo serial communication
KangarooSerial  K1(Serial1);
KangarooChannel K1_1(K1, '1');
KangarooChannel K1_2(K1, '2');

KangarooSerial  K2(Serial2);
KangarooChannel K2_1(K2, '1');
KangarooChannel K2_2(K2, '2');

// Defining variables used to store inputs, speeds, and position values.
int wheel_speed[4];
char temp;
char val[40];
int index = 0;
int pos1;
int pos2;
int pos3;
int pos4;

// Setup USB communication and Kangaroo baud rate. Zero current wheel positions.
void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println("Enter four numbers: ");

  Serial1.begin(115200);
  Serial2.begin(115200);
  
  K1_1.start();
  K1_1.home();
  K1_2.start();
  K1_2.home();

  K2_1.start();
  K2_1.home();
  K2_2.start();
  K2_2.home();
}

void loop()
{
  if(Serial.available() > 0)
  {
    temp = Serial.read();
    val[index] = temp;
    index++;
    if(temp == '.')
    {
      sscanf(val, "%d %d %d %d", &wheel_speed[0], &wheel_speed[1], &wheel_speed[2], &wheel_speed[3]);
      Serial.println(wheel_speed[0]);
      Serial.println(wheel_speed[1]);
      Serial.println(wheel_speed[2]);
      Serial.println(wheel_speed[3]);
      Serial.println("1");
      K1_1.s(wheel_speed[1]);
      Serial.println("2");
      K1_2.s(wheel_speed[0]);

      pos1 = K1_1.getP().value();
      pos2 = K1_2.getP().value();
      Serial.print("Wheel 1: ");
      Serial.println(pos1);
      Serial.print("Wheel 2: ");
      Serial.println(pos2);
      Serial.println("3");
      K2_1.s(wheel_speed[2]);
      Serial.println("4");
      K2_2.s(wheel_speed[3]);

      pos3 = K2_1.getP().value();
      pos4 = K2_2.getP().value();

      Serial.print("Wheel 3: ");
      Serial.println(pos3);
      Serial.print("Wheel 4: ");
      Serial.println(pos4);
      
      Serial.print("Enter two numbers: ");
      memset(val,'\0', 40);
      index = 0;
      
    }
  }
}
