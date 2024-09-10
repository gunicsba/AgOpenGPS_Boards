
//#ifdef ETH_CLK_MODE
//#undef ETH_CLK_MODE
//#endif
#define ETH_CLK_MODE    ETH_CLOCK_GPIO17_OUT
//#define ETH_CLK_MODE    ETH_CLOCK_GPIO0_OUT
////#define ETH_POWER_PIN   12
#define ETH_POWER_PIN   -1
#define ETH_POWER_PIN_ALTERNATIVE 17
#define ETH_TYPE        ETH_PHY_LAN8720
#define ETH_ADDR        1   //OK?  OK
#define ETH_MDC_PIN     23  //OK
#define ETH_MDIO_PIN    18  //OK

//WiFiUDP Udp;                      //Create UDP object
//unsigned int localUdpPort = 4196; //local port

// Set it based on the IP address of the router
//IPAddress LocalIP(192, 168, 1, 200);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 255, 0);
//IPAddress dns(192, 168, 1, 1);

void Eth8720_Start() {

  WiFi.onEvent(WiFiEvent);
    
  ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE); //start with ETH

  // write confir for static IP, gateway,subnet,dns1,dns2
  if (ETH.config(myip, gwip, mask) == false) {
    Serial.println("LAN8720 Configuration failed.");
  } else {
    Serial.println("LAN8720 Configuration success.");
  }

  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(ETH.localIP());

  //Udp.begin(localUdpPort); //begin UDP listener
}


void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void testClient(const char * host, uint16_t port)
{
  Serial.print("\nconnecting to ");
  Serial.println(host);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    Serial.write(client.read());
  }

  Serial.println("closing connection\n");
  client.stop();
}
