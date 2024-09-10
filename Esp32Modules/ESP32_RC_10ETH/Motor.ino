void AdjustFlow()
{
    switch (MDL.MotorDriverType)
        {
        default:
      //Serial.println("Motor driver default");
        for (int i = 0; i < MDL.ProductCount; i++)
        {
/*          Serial.print("AdJustFlow - i: ");
          Serial.println(i);*/
          switch (Sensor[i].ControlType)
          {
          case 1:
            // fast close valve, used for flow control and on/off
            if (Sensor[i].FlowEnabled)
            {
                if (Sensor[i].pwmSetting >= 0)
                {
                    //increase
                    if (Sensor[i].pwmSetting > 250) Sensor[i].pwmSetting = 255;
                  /*
                  Serial.println("Keveroszelep novel - default+1");
                    Serial.println(
                      sectionHandler(7+i, "on", Sensor[i].pwmSetting*8)
                    );
                    */
                    sectionHandler(7+i, "on", Sensor[i].pwmSetting*8);
                    //ledcWrite(PWM1_Ch[i], Sensor[i].pwmSetting);
                    //ledcWrite(PWM2_Ch[i], 0);     
                }
                else
                {
                  //Serial.println("Keveroszelep csokkent - default+1");
                    //decrease
                    if (Sensor[i].pwmSetting < -250) Sensor[i].pwmSetting = -255;
                    /*
                    Serial.println(
                      sectionHandler(7+i, "off", abs(Sensor[i].pwmSetting*8) )
                    );
                    */
                    sectionHandler(7+i, "off", abs(Sensor[i].pwmSetting*8) );
                    //ledcWrite(PWM2_Ch[i], -Sensor[i].pwmSetting);
                    //ledcWrite(PWM1_Ch[i], 0);  
                }
            }
            else
            {
                //Serial.println("Keveroszelep OFF - default+1");
                sectionHandler(7+i, "off", 4095);
                // stop flow
                 //ledcWrite(PWM2_Ch[i], 255);
                 //ledcWrite(PWM1_Ch[i], 0);
            }
            break;

          case 2:
          case 3:
          case 4:
             // motor control
            if (Sensor[i].FlowEnabled)
            {
                if (Sensor[i].pwmSetting >= 0)
                {
                    //increase
                    //Serial.println("Keveroszelep novel - default+4");
                    sectionHandler(7+i, "on", Sensor[i].pwmSetting*8);
                    //ledcWrite(PWM1_Ch[i], Sensor[i].pwmSetting);
                    //ledcWrite(PWM2_Ch[i], 0);
                }
                else
                {
                    //decrease
                    //Serial.println("Keveroszelep csokkent - default+4");
                     sectionHandler(7+i, "off", abs(Sensor[i].pwmSetting*8) );
                     //ledcWrite(PWM2_Ch[i], -Sensor[i].pwmSetting);
                     //ledcWrite(PWM1_Ch[i], 0);// offsets the negative pwm value
                }
            }
            else
            {
                // stop motor
                Serial.println("Keveroszelep OFF - default+4");
                sectionHandler(7+i, "off", 4095);
                //ledcWrite(PWM2_Ch[i], 255);
                //ledcWrite(PWM1_Ch[i], 0);
            }
            break;

          default:
            // standard valve, flow control only
            // standard valve, flow control only
            if (Sensor[i].pwmSetting >= 0)
            {
              /*
                //increase                
                Serial.println("Keveroszelep novel - default+default");
                //0-255 -> 0-4096  *16
                Serial.print("++++++++++++++++++++++++++++++++");
                Serial.println(
                sectionHandler(7+i, "on", Sensor[i].pwmSetting*16)
                );
                */
                sectionHandler(7+i, "on", Sensor[i].pwmSetting*16);
                //ledcWrite(PWM1_Ch[i], Sensor[i].pwmSetting);
                //ledcWrite(PWM2_Ch[i], 0);  
            }
            else
            {
              /*
                //decrease
                Serial.println("Keveroszelep csokkent - default+default");
                Serial.print("-------------------------------");
                Serial.println(
                sectionHandler(7+i, "off", abs(Sensor[i].pwmSetting*16))
                );
                */
                sectionHandler(7+i, "off", abs(Sensor[i].pwmSetting*16));
                //ledcWrite(PWM2_Ch[i], -Sensor[i].pwmSetting);
                //ledcWrite(PWM1_Ch[i], 0);   // offsets the negative pwm value
            }
            break;
        }
    }
    break;



    case 1:
    Serial.println("Motor driver 1");
    for (int i = 0; i < MDL.ProductCount; i++)
    {
        switch (Sensor[i].ControlType)
        {
        case 1:
            // fast close valve, used for flow control and on/off
            if (Sensor[i].FlowEnabled)
            {
                if (Sensor[i].pwmSetting >= 0)
                {
                    //increase
                    if (Sensor[i].pwmSetting > 250)  Sensor[i].pwmSetting = 255;

                    digitalWrite(Sensor[i].DirPin, MDL.FlowOnDirection);
                    analogWrite(Sensor[i].PWMPin, Sensor[i].pwmSetting);
                }
                else
                {
                    //decrease
                    if (Sensor[i].pwmSetting < -250) Sensor[i].pwmSetting = -255;

                    digitalWrite(Sensor[i].DirPin, !MDL.FlowOnDirection);
                    analogWrite(Sensor[i].PWMPin, -Sensor[i].pwmSetting); // offsets the negative pwm value
                }
            }
            else
            {
                // stop flow
                digitalWrite(Sensor[i].DirPin, !MDL.FlowOnDirection);
                analogWrite(Sensor[i].PWMPin, 255);
            }
            break;

        case 2:
        case 3:
        case 4:
            // motor control
            if (Sensor[i].FlowEnabled)
            {
                if (Sensor[i].pwmSetting >= 0)
                {
                    //increase
                    digitalWrite(Sensor[i].DirPin, MDL.FlowOnDirection);
                    analogWrite(Sensor[i].PWMPin, Sensor[i].pwmSetting);
                }
                else
                {
                    //decrease
                    digitalWrite(Sensor[i].DirPin, !MDL.FlowOnDirection);
                    analogWrite(Sensor[i].PWMPin, -Sensor[i].pwmSetting); // offsets the negative pwm value
                }
            }
            else
            {
                // stop motor
                analogWrite (Sensor[i].PWMPin, 0);
            }
            break;

        default:
            // standard valve, flow control only
            if (Sensor[i].pwmSetting >= 0)
            {
                //increase
                digitalWrite(Sensor[i].DirPin, MDL.FlowOnDirection);
                analogWrite(Sensor[i].PWMPin, Sensor[i].pwmSetting);
            }
            else
            {
                //decrease
                digitalWrite(Sensor[i].DirPin, !MDL.FlowOnDirection);
                analogWrite(Sensor[i].PWMPin, -Sensor[i].pwmSetting); // offsets the negative pwm value
            }
            break;
        }
    }
     break;
}
}
