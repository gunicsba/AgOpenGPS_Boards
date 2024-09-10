uint8_t Rlys;
uint8_t NewLo;
uint8_t NewHi;

void CheckRelays()
{
    NewLo = 0;
    NewHi = 0;

    if (Sensor[0].FlowEnabled || Sensor[1].FlowEnabled)
    {
        {
            // normal relay control
            NewLo = RelayLo;
            NewHi = RelayHi;
        }
    }

    // power relays, always on
    NewLo |= PowerRelayLo;
    NewHi |= PowerRelayHi;

    switch (MDL.RelayControl)
    {
    case 1:
        // rs485
        break;

    case 2:
        // PCA9555 8 relays
        Rlys = NewLo;
        //pcf.write(Rlys);
        for (int i = 0; i < 7; i++)
            {
                if (bitRead(Rlys, i)) {
                  sectionHandler(i, "on", 4096);
//                  digitalWrite(pcf, i, MDL.RelayOnSignal); 
                } else {
                  sectionHandler(i, "off", 4096);
//                  digitalWrite(pcf, i, !MDL.RelayOnSignal);
                }
            }
        break;

    case 3:
         
        break;

    case 4:
        // MCP23017
        break;

    case 5:
        // GPIOs
        for (int j = 0; j < 2; j++)
        {
            if (j < 1) Rlys = NewLo; else Rlys = NewHi;
            for (int i = 0; i < 8; i++)
            {
                if (MDL.RelayPins[i + j * 8] > 1) // check if relay is enabled
                {
                    if (bitRead(Rlys, i)) digitalWrite(MDL.RelayPins[i + j * 8], MDL.RelayOnSignal); else digitalWrite(MDL.RelayPins[i + j * 8], !MDL.RelayOnSignal);
                }
            }
        }
        break;
    }
}
