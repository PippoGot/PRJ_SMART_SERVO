#include "AS5600.h"
#include "Wire.h"

AS5600L as5600(0x36);   //  use default Wire
int raw_angle = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  Serial.print("Hysteresis:\t ");
  Serial.println(as5600.getHysteresis(), HEX);

  Serial.print("ZMCO:\t ");
  Serial.println(as5600.getZMCO());

  
  Serial.print("Magnet:\t ");
  Serial.println(as5600.detectMagnet());
}


void loop()
{
  /*
  Serial.print("STATUS:\t ");
  Serial.println(as5600.readStatus(), HEX);
  Serial.print("CONFIG:\t ");
  Serial.println(as5600.getConfigure(), HEX);
  Serial.print("  GAIN:\t ");
  Serial.println(as5600.readAGC(), HEX);
  */
  raw_angle = as5600.readAngle();
  Serial.print("Variable_1:");
  Serial.print(raw_angle);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.print(4096);  
  Serial.print(",");
  Serial.print("Variable_3:");
  Serial.println(-1);
  /*
  Serial.print("MAGNET:\t ");
  Serial.println(as5600.readMagnitude(), HEX);
  Serial.print("DETECT:\t ");
  Serial.println(as5600.detectMagnet(), HEX);
  Serial.print("M HIGH:\t ");
  Serial.println(as5600.magnetTooStrong(), HEX);
  Serial.print("M  LOW:\t ");
  Serial.println(as5600.magnetTooWeak(), HEX);
  Serial.println();
  */

  delay(100);
}
