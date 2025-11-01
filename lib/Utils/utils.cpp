#include "utils.h"

// Prints controller stick/button state
void printControllerStruct()
{
    Serial.print(F("ButtonWord: "));
    // Arduino printf() doesn’t support %b (binary), so we print manually:
    for (int i = 15; i >= 0; i--)
    {
        Serial.print(bitRead(currentState.buttonWord, i));
    }

    Serial.print(F(" | LX: "));
    Serial.print(currentState.leftX);
    Serial.print(F(" | LY: "));
    Serial.print(currentState.leftY);
    Serial.print(F(" | RX: "));
    Serial.print(currentState.rightX);
    Serial.print(F(" | RY: "));
    Serial.println(currentState.rightY);
}

// Prints PWM and control variable states
void printControlVariables()
{
    Serial.print(F(" | pwmLY "));
    Serial.print(leftYPWM);
    Serial.print(F(" | pwmLX "));
    Serial.print(leftXPWM);
    Serial.print(F(" | pwmRY "));
    Serial.print(rightYPWM);
    Serial.print(F(" | pwmSTR "));
    Serial.print(rightXPWM);

    Serial.print(F(" | ENAF "));
    Serial.print(enableAMotorFwd);
    Serial.print(F(" | ENAR "));
    Serial.print(enableAMotorRev);
    Serial.print(F(" | ENBF "));
    Serial.print(enableBMotorFwd);
    Serial.print(F(" | ENBR "));
    Serial.print(enableBMotorRev);

    Serial.print(F(" | EN: "));
    Serial.print(enableToggle ? F("ON") : F("OFF"));
    Serial.print(F(" | T_MD: "));
    Serial.print(tankMode ? F("ON") : F("OFF"));
    Serial.print(F(" | p_Brk: "));
    Serial.println(parking_Brake ? F("ON") : F("OFF"));
}
