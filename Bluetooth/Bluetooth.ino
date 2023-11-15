#include "BluetoothSerial.h"
#include "ESP32Servo.h"

// CONEXIONES PARA EL BLUETOOTH.

BluetoothSerial SerialBT;

// MOTOR 1.

int Motor1A = 17;
int Motor1B = 5;

// MOTOR 2.

int Motor2A = 18;
int Motor2B = 19;

void setup(){

    // put your setup code here, to run once:
    Serial.begin(115200);
    SerialBT.begin("PROMEESP32"); // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    pinMode(Motor1A, OUTPUT);
    pinMode(Motor2A, OUTPUT);
    pinMode(Motor1B, OUTPUT);
    pinMode(Motor2B, OUTPUT);

    digitalWrite(Motor1A, LOW);
    digitalWrite(Motor2A, LOW);
    digitalWrite(Motor1B, LOW);
    digitalWrite(Motor2B, LOW);
}

int flag1 = -1;
int flag2 = -1;

void loop()
{
    if (SerialBT.available())
    {
        char toSend = (char)SerialBT.read();
        if (toSend == 'S')
        {

            flag1 = 0;
            flag2 = 0;

            digitalWrite(Motor1A, LOW);
            analogWrite(Motor1B, LOW);

            digitalWrite(Motor2A, LOW),
              analogWrite(Motor2B, LOW);
        }
        if (toSend == 'F' || toSend == 'G' || toSend == 'I')
        {
            if (flag1 != 1)
            {
                // ESTOS HARAN QUE VAYA PARA ADELANTE EL CARRITO.
                flag1 = 1;
                digitalWrite(Motor1A, HIGH);
                analogWrite(Motor1B, 0);
                digitalWrite(Motor2A, HIGH);
                analogWrite(Motor2B, 0);
            }
        }
        if (toSend == 'B' || toSend == 'H' || toSend == 'J')
        {
            if (flag1 != 2)
            {
                // ESTOS HARAN LA REVERSA DEL CARRITO.
                flag1 = 2;
                digitalWrite(Motor1B, HIGH);
                analogWrite(Motor1A, 0);
                digitalWrite(Motor2B, HIGH);
                analogWrite(Motor2A, 0);
            }
        }
        if (toSend == 'L' || toSend == 'G' || toSend == 'H')
        {
            if (flag2 != 1)
            {
                // ESTOS HARAN QUE GIRE HACIA LA IZQUIERDA.
                flag2 = 1;
                digitalWrite(Motor2B, HIGH);
                analogWrite(Motor2A, 0);
                digitalWrite(Motor1A, HIGH);
                analogWrite(Motor1B, 0);
            }
        }
        else if (toSend == 'R' || toSend == 'I' || toSend == 'J')
        {
            if (flag2 != 2)
            {
                // ESTOS HARAN QUE GIRE HACIA LA DERECHA.
                flag2 = 2;
                digitalWrite(Motor1B, HIGH);
                analogWrite(Motor1A, 0);
                digitalWrite(Motor2A, HIGH);
                analogWrite(Motor2B, 0);
            }
        }
        else
        {
            if (flag2 != 3)
            {
                flag2 = 3;
                digitalWrite(Motor2A, LOW);
                analogWrite(Motor2B, LOW);
                digitalWrite(Motor2B, LOW);
                analogWrite(Motor2A, LOW);
            }
        }
    }
}
