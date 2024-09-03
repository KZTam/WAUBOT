#include "Waubot.h"


Waubot wbot = Waubot(true);



void setup() {
  Serial.begin(115200);

  wbot.calibrate();
}

void loop() {

   
    

    wbot.moveDelay(BLACK_LINE, 100, 0.1, 0.0, 1000);
    //wbot.moveForwardCustomDelay(100, 100, 1000);
    // wbot.moveForwardDelay(233,344);
wbot.moveForwardDelay(0,2000);
     wbot.moveBackwardDelay(0,2000);

}
