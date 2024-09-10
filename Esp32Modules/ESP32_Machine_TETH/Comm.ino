byte MSBusb;
byte LSBusb;

uint16_t PGNusb;
uint16_t PGNethernet;

byte DataUSB[MaxReadBuffer];

byte DataOut[50];

void SendData()
{

    //checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_237_Size; i++)
    {
      CK_A = (CK_A + PGN_237[i]);
    }
    PGN_237[PGN_237_Size] = CK_A;

    // to ethernet
    if (ETH.linkUp() == LinkON)
    { 
      UDPcomm.broadcastTo(PGN_237, sizeof(PGN_237), portDestination);
      //UDPcomm.beginPacket(DestinationIP, DestinationPort);
      //UDPcomm.write(DataOut, PGN32613Length);
      //UDPcomm.endPacket();
    }
}

void ReceiveData()
{
  //ethernet
  if (ETH.linkUp() == LinkON)
  {
    uint16_t len = DataEthernetLen;
    if (!len) return;
  }

  uint8_t* udpData = DataEthernet;
   if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) //Data
    {

        if (udpData[3] == 239)  //machine data
        {
            uTurn = udpData[5];
            gpsSpeed = (float)udpData[6];//actual speed times 4, single uint8_t

            hydLift = udpData[7];
            tramline = udpData[8];  //bit 0 is right bit 1 is left

            relayLo = udpData[11];          // read relay control from AgOpenGPS
            relayHi = udpData[12];

            if (aogConfig.isRelayActiveHigh)
            {
                tramline = 255 - tramline;
                relayLo = 255 - relayLo;
                relayHi = 255 - relayHi;
            }

            //Bit 13 CRC

            //reset watchdog
            esp_task_wdt_reset();
        }

        else if (udpData[3] == 200) // Hello from AgIO
        {
            if (udpData[7] == 1)
            {
                relayLo -= 255;
                relayHi -= 255;
                esp_task_wdt_reset();
            }

            helloFromMachine[5] = relayLo;
            helloFromMachine[6] = relayHi;
            
            UDPcomm.writeTo(helloFromMachine, sizeof(helloFromMachine), IPAddress(ipDestination), portDestination);
        }


        else if (udpData[3] == 238)
        {
            aogConfig.raiseTime = udpData[5];
            aogConfig.lowerTime = udpData[6];
            aogConfig.enableToolLift = udpData[7];

            //set1 
            uint8_t sett = udpData[8];  //setting0     
            if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

            aogConfig.user1 = udpData[9];
            aogConfig.user2 = udpData[10];
            aogConfig.user3 = udpData[11];
            aogConfig.user4 = udpData[12];

            //crc

            //save in EEPROM and restart
            EEPROM.put(6, aogConfig);
            EEPROM.commit();
            delay(100);
            ESP.restart();
            //resetFunc();
        }

        else if (udpData[3] == 201)
        {
            //make really sure this is the subnet pgn
            if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201)
            {
                networkAddress.ipOne = udpData[7];
                networkAddress.ipTwo = udpData[8];
                networkAddress.ipThree = udpData[9];

                //save in EEPROM and restart
                EEPROM.put(50, networkAddress);
                EEPROM.commit();
                delay(100);
                ESP.restart();
            }
        }

        //Scan Reply
        else if (udpData[3] == 202)
        {
            //make really sure this is the subnet pgn
            if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202)
            {
                uint8_t scanReply[] = { 128, 129, 123, 203, 7,
                    networkAddress.ipOne, networkAddress.ipTwo, networkAddress.ipThree, 123,
                    src_ip[0], src_ip[1], src_ip[2], 23 };

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                {
                    CK_A = (CK_A + scanReply[i]);
                }
                scanReply[sizeof(scanReply) - 1] = CK_A;

                uint16_t portDest = 9999; //AOG port that listens

                //off to AOG
                UDPcomm.broadcastTo(scanReply, sizeof(scanReply), portDestination);
            }
        }

        else if (udpData[3] == 236) //EC Relay Pin Settings 
        {
            for (uint8_t i = 0; i < 24; i++)
            {
                pin[i] = udpData[i + 5];
            }

            //save in EEPROM and restart
            EEPROM.put(20, pin);
        }
    }
}
