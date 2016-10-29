#include <EEPROM.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <lib_dmx.h>     // deskontrol four universes DMX library
#include "artnet_node.h"
#include "common.h"      // definitions from libartnet
#include "packets.h"     // headers from libartnet, striped version

#define DEFAULTIP (1)
#define DEFAULT_ADDRESS (DEFAULTIP + sizeof(DefaultIp))
#define FLAGS (0)
#define RESET_PIN (9)

// In order to be more consistent between MEGA and UNO version, now UNIVERSE_0 is UART0, UNIVERSE_1 is UART1, ...

// ***************************************************************************************************************
//                             ***        READ THIS        ***
//    To program the Arduino via USB will be necessary to remove the DMX shield, and put it back once programmed!
// ***************************************************************************************************************
//#define   USE_UNIVERSE_0_TX      // remember universe 0 now is USART 0, Arduino output pin 1
#ifndef USE_UNIVERSE_0_TX
  //#define   USE_UNIVERSE_0_RX
#endif

// more than 1 universe, only in Arduino MEGA
// more than 2 output universes -> slow frame rate
// more than 3 output universes -> unstable or crash :(     we need more processor...
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) && !defined(USE_TINKERKIT_SHIELD)
  //#define   USE_UNIVERSE_1_TX      
      #ifndef USE_UNIVERSE_1_TX
        #define   USE_UNIVERSE_1_RX
      #endif
  
  //#define   USE_UNIVERSE_1      // remember universe 1 now is USART 1, Arduino output pin 18
  //#define   USE_UNIVERSE_2      // remember universe 2 now is USART 2, Arduino output pin 16
  //#define   USE_UNIVERSE_3      // remember universe 3 now is USART 3, Arduino output pin 14
  //#define  USE_UNIVERSE_1_RX   
#endif

// New DMX modes *** EXPERIMENTAL ***
#define        DMX512            (0)    // (250 kbaud - 2 to 512 channels) Standard USITT DMX-512
#define        DMX1024           (1)    // (500 kbaud - 2 to 1024 channels) Completely non standard - TESTED ok
#define        DMX2048           (2)    // (1000 kbaud - 2 to 2048 channels) called by manufacturers DMX1000K, DMX 4x or DMX 1M ???


struct status_type {
  unsigned user_ip: 1;
  unsigned user_config: 1;
  unsigned reset: 1;
} status;

typedef struct ipprog_t {
  uint8_t ip [4];
  uint8_t mask [4];
  uint16_t port;
};

ipprog_t DefaultIp {
  2,    0,  0,  10,
  255,  0,  0,  0,
  0x1936
  };

typedef struct address_t {
  uint8_t  net;
  uint8_t  subnet;
  uint8_t  swin[ARTNET_MAX_PORTS];
  uint8_t  swout[ARTNET_MAX_PORTS];
};

address_t DefaultAddress {
  0x0,               //Net settings
  0x0,                //Subnet settings
  0x0, 0x1, 0x2, 0x3, //SwIn Settings
  0x0, 0x1, 0x2, 0x3  //SwOut Settings
};

typedef struct node_status_t {
  uint8_t   numports;
  uint8_t   porttypes [ARTNET_MAX_PORTS];
  uint8_t   goodinput [ARTNET_MAX_PORTS];
  uint8_t   goodoutput [ARTNET_MAX_PORTS];
  };

node_status_t NodeStatus {
  
};

artnet_reply_t            ArtPollReply;
artnet_ipprog_reply_t     ArtIpprogReply;
//artnet_dmx_t              ArtDmx;
artnet_packet_type_t      packet_type;
EthernetUDP               Udp;

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  const int MAX_BUFFER_UDP = 1650;  // For Arduino MEGA
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  const int MAX_BUFFER_UDP = 560; // For Arduino UNO, due to only have 2kb of RAM, and only 1 UART
#endif

uint8_t packetBuffer [MAX_BUFFER_UDP]; // buffer to store incoming UDP data

uint8_t factory_mac          [6] = {0xA4, 0xB1, 0x33, 0x7E, 0xB3, 0x11}; // the mac address of node
uint8_t factory_broadcastIp  [4] = {   2, 255, 255, 255};           // broadcast IP address
uint8_t factory_gateway      [4] = {   2,   0,   0,   1};           // gateway IP address (use ip address of controller)

long previousMillis = 0;
long interval = 1000;

void setup() {
  //Serial.begin(9600);
  
  pinMode(RESET_PIN, INPUT_PULLUP);
  
  EEPROM.get(FLAGS, status);
  
  if(status.reset == 1 | digitalRead(RESET_PIN) == 0){ //Check first on and reset button pressed during powering up
    EEPROM.put(DEFAULTIP, DefaultIp);                  //put default ip to eeprom
    EEPROM.put(DEFAULT_ADDRESS, DefaultAddress);       //put default address to eeprom
    status.reset = 0;
    EEPROM.put(FLAGS, status);
  }
 
  EEPROM.get(DEFAULTIP, DefaultIp);
  EEPROM.get(DEFAULT_ADDRESS, DefaultAddress);

  NodeStatus.numports  = 0;
  
#if defined(USE_UNIVERSE_0_TX)
 
  // change pin number for Arduino UNO (or use -1, and connect pins 2 and 3 of max485 to +5 Vcc)
  ArduinoDmx0.set_control_pin(4);     // max485 input/output control (connect to 485 pins 2-3) 
  ArduinoDmx0.set_tx_address(1);       // set tx0 start address
  ArduinoDmx0.set_tx_channels(512);    // number of TX channels
  ArduinoDmx0.init_tx(DMX512);         // starts universe 0 as tx, standard DMX512   ***new***
  NodeStatus.numports ++;
  NodeStatus.porttypes [0] = 0x80;
  NodeStatus.goodinput [0] = 0x0;
  NodeStatus.goodoutput [0] = 0x80;
#endif

#if defined(USE_UNIVERSE_0_RX)
  ArduinoDmx0.set_control_pin(4);     // max485 input/output control (connect to 485 pins 2-3) 
  ArduinoDmx0.set_rx_address(1);       // set tx0 start address
  ArduinoDmx0.set_rx_channels(512);    // number of TX channels
  ArduinoDmx0.attachRXInterrupt  (frame_received); 
  ArduinoDmx0.init_rx(DMX512);         // starts universe 0 as tx, standard DMX512   ***new***
  NodeStatus.numports ++;
  NodeStatus.porttypes [0] = 0x40;
  NodeStatus.goodinput [0] = 0x80;
  NodeStatus.goodoutput [0] = 0x00;
#endif

#if defined(USE_UNIVERSE_1_TX)
  ArduinoDmx1.set_control_pin(24);     // max485 input/output control (connect to 485 pins 2-3) 
  ArduinoDmx1.set_tx_address(1);       // set tx1 start address
  ArduinoDmx1.set_tx_channels(512);    // number of TX channels
  ArduinoDmx1.init_tx(DMX512);         // starts universe 1 as tx, standard DMX512   ***new***
  NodeStatus.numports ++;
  NodeStatus.porttypes [1] = 0x80;
  NodeStatus.goodinput [1] = 0x0;
  NodeStatus.goodoutput [1] = 0x80;
#endif

#if defined(USE_UNIVERSE_1_RX)
  ArduinoDmx1.set_control_pin(24);     // max485 input/output control (connect to 485 pins 2-3) 
  ArduinoDmx1.set_rx_address(1);       // set tx1 start address
  ArduinoDmx1.set_rx_channels(512);    // number of TX channels
  ArduinoDmx1.attachRXInterrupt  (frame_received); 
  ArduinoDmx1.init_rx(DMX512);         // starts universe 1 as tx, standard DMX512   ***new***
  NodeStatus.numports ++;
  NodeStatus.porttypes [1] = 0x40;
  NodeStatus.goodinput [1] = 0x80;
  NodeStatus.goodoutput [1] = 0x0;
#endif

  fill_art_poll_reply    (&ArtPollReply);
  fill_art_ipprog_reply  (&ArtIpprogReply, &ArtPollReply);
  //fill_dmx           (&ArtDmx);
  
  Ethernet.begin(factory_mac, DefaultIp.ip, factory_gateway, factory_gateway, DefaultIp.mask); 
  Udp.begin(DefaultIp.port);
}

void loop() {
  if(Udp.parsePacket()){
    if( Udp.available() > ARNET_HEADER_SIZE ) 
   handle_packet();
   read_dmx();
  }
}

void handle_packet()
{
  Udp.read((uint8_t *)&packetBuffer, MAX_BUFFER_UDP);  
  
  packet_type = (artnet_packet_type_t) get_packet_type((uint8_t *)&packetBuffer);
    
  if(packet_type == 0)  // bad packet
  {
    return;
  }  
  if(packet_type == ARTNET_DMX)
  {
    if(sizeof(packetBuffer) < sizeof(artnet_dmx_t)) 
      return;
    else
      handle_dmx((artnet_dmx_t *)&packetBuffer);
  }   
  else if(packet_type == ARTNET_POLL)
  {
    if(sizeof(packetBuffer) < sizeof(artnet_poll_t)) 
      return;
    else
      handle_poll((artnet_poll_t *)&packetBuffer);
  }
  else if(packet_type == ARTNET_IPPROG)
  {
    if(sizeof(packetBuffer) < sizeof(artnet_ipprog_t))
      return;
    else
      handle_ipprog((artnet_ipprog_t *)&packetBuffer);
  } 
  else if(packet_type == ARTNET_ADDRESS)
  {
    if(sizeof(packetBuffer) < sizeof(artnet_address_t))
      return;
    else
      handle_address((artnet_address_t *)&packetBuffer);
  } 
}

uint16_t get_packet_type(uint8_t *packet) //this get artnet packet type
{
  if (! memcmp( packet, ArtPollReply.id, 8)) 
  {
    return bytes_to_short(packet[9], packet[8]); 
  } 
  return 0;  // bad packet
}

int handle_dmx(artnet_dmx_t *packet)
{
  if(packet->universe == DefaultAddress.swout[0])
  {   
    #if defined(USE_UNIVERSE_0_TX)
      memcpy ((uint8_t *)ArduinoDmx0.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
  else if(packet->universe == DefaultAddress.swout[1])
  {   
    #if defined(USE_UNIVERSE_1_TX)
      memcpy ((uint8_t *)ArduinoDmx1.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
  else if(packet->universe == DefaultAddress.swout[2])
  {   
    #if defined(USE_UNIVERSE_2)
      //memcpy ((uint8_t *)ArduinoDmx2.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
  else if(packet->universe == DefaultAddress.swout[3])
  {   
    #if defined(USE_UNIVERSE_3)
      //memcpy ((uint8_t *)ArduinoDmx3.TxBuffer, (uint8_t *)packet->data, ARTNET_DMX_LENGTH);
    #endif
  }
}

int handle_poll(artnet_poll_t *packet) 
{
  if((packet->ttm & 1) == 1) // controller say: send unicast reply
  {
    send_reply(UNICAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
  }
  else // controller say: send broadcast reply
  {
    send_reply(BROADCAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
  }
}

int handle_ipprog(artnet_ipprog_t *packet) 
{
  //Serial.println("handle IpProg");
  if((packet->Command & (1<<7)) != 0) //enable any programming
  {
     
    if((packet->Command & (1<<0)) != 0) //Program Port
    {
     //Serial.println("Programm Port");
    }
    if((packet->Command & (1<<1)) != 0) //Program Subnet Mask
    {
     DefaultIp.mask [0] = packet->ProgSmHi;
     DefaultIp.mask [1] = packet->ProgSm2;
     DefaultIp.mask [2] = packet->ProgSm1;
     DefaultIp.mask [3] = packet->ProgSmLo;    
     //Serial.println("Programm SubNet");
    }
    if((packet->Command & (1<<2)) != 0) //Program IP Address
    {
     //Serial.println("Programm IP");
     DefaultIp.ip [0] = packet->ProgIpHi;
     DefaultIp.ip [1] = packet->ProgIp2;
     DefaultIp.ip [2] = packet->ProgIp1;
     DefaultIp.ip [3] = packet->ProgIpLo; 
    }
    if((packet->Command & (1<<6)) != 0) //enable DHCP
    {
     //Serial.println("Enable DHCP");
    }
  
    if((packet->Command & (1<<3)) != 0) //return all three parameters to default
    {
     //Serial.println("Set Default");
    }
  

    EEPROM.put(DEFAULTIP, DefaultIp);
  
    send_reply(UNICAST, (uint8_t *)&ArtIpprogReply, sizeof(ArtIpprogReply));
  
    fill_art_poll_reply    (&ArtPollReply);
    fill_art_ipprog_reply  (&ArtIpprogReply, &ArtPollReply);
  
    Udp.stop();
    Ethernet.begin(factory_mac, DefaultIp.ip, factory_gateway, factory_gateway, DefaultIp.mask); 
    Udp.begin(DefaultIp.port);
  }
}

int handle_address(artnet_address_t *packet) //not implemented yet
{
  if((packet->net & (1<<7)) != 0) //programming net
  {
    DefaultAddress.net = packet->net & (~(1<<7));
  }
  
  if((packet->subnet & (1<<7)) != 0) //programming subnet
  {
    DefaultAddress.subnet = packet->subnet & (~(1<<7));
  }
  for (int i = 0; i < 4; i++){
    if((packet->swin [i] & (1<<7)) != 0) //enable programming swin
  {
    DefaultAddress.swin [i] = packet->swin [i] & (~(1<<7));
  }
  if((packet->swout [0] & (1<<7)) != 0) //enable programming swout
  {
    DefaultAddress.swout [i] = packet->swout [i] & (~(1<<7));
  }
  }
  EEPROM.put(DEFAULT_ADDRESS, DefaultAddress);
  fill_art_poll_reply    (&ArtPollReply);
    
  send_reply(UNICAST, (uint8_t *)&ArtPollReply, sizeof(ArtPollReply));
}

void send_reply(uint8_t mode_broadcast, uint8_t *packet, uint16_t size)
{
  if(mode_broadcast == 1) // send broadcast packet
  {
  //Serial.println("Send reply. Mode BROADCAST");
  Udp.beginPacket(factory_broadcastIp, DefaultIp.port);
  Udp.write(packet, size);
  Udp.endPacket();
  }
  else // send unicast packet to controller
  {
    //Serial.println("Send reply. Mode UNICAST");
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(packet, size);
  Udp.endPacket();
  }
}


void fill_art_poll_reply(artnet_reply_t *poll_reply)
{
  //fill to 0's
  memset (poll_reply, 0, sizeof(poll_reply));
  
  //copy data from node
  sprintf((char *)poll_reply->id, "Art-Net\0"); // *** don't change never ***
  memcpy (poll_reply->ip, DefaultIp.ip, 4);           // the IP address of node
  memcpy (poll_reply->mac, factory_mac, 6);                   // the mac address of node
  sprintf((char *)poll_reply->shortname, "geneg node\0");
  sprintf((char *)poll_reply->longname, "Art-net Node v0.3 (c) 2015\0");
  sprintf((char *)poll_reply->nodereport, "%i DMX universes active.\0", NodeStatus.numports);

  memcpy (poll_reply->porttypes, NodeStatus.porttypes, 4); 
  memcpy (poll_reply->goodinput, NodeStatus.goodinput, 4);
  memcpy (poll_reply->goodoutput, NodeStatus.goodoutput, 4);
  memcpy (poll_reply->swin, DefaultAddress.swin, 4);
  memcpy (poll_reply->swout, DefaultAddress.swout, 4);

  poll_reply->etsaman[0] = 0;        // The ESTA manufacturer code.
  poll_reply->etsaman[1] = 0;        // The ESTA manufacturer code.
  
  poll_reply->opCode          = 0x2100;  // ARTNET_REPLY
  poll_reply->port            = DefaultIp.port;
  poll_reply->verH            = 0;
  poll_reply->ver             = 3;
  
  poll_reply->net             = DefaultAddress.net;
  poll_reply->sub             = DefaultAddress.subnet;
  poll_reply->oemH            = 0;
  poll_reply->oem             = 0xFF;
  poll_reply->status          = 0; //CHECK
  poll_reply->numbportsH      = 0;
  poll_reply->numbports       = NodeStatus.numports;
  poll_reply->swvideo         = 0;
  poll_reply->swmacro         = 0;
  poll_reply->swremote        = 0;
  poll_reply->style           = 0;
}

void fill_art_ipprog_reply(artnet_ipprog_reply_t *ipprog_reply, artnet_reply_t *poll_reply)
{
  //fill to 0's
  memset (ipprog_reply, 0, sizeof(ipprog_reply));
  
  //copy data from node
  memcpy (ipprog_reply->id, poll_reply->id, sizeof(ipprog_reply->id));

  ipprog_reply->ProgIpHi  = DefaultIp.ip[0];
  ipprog_reply->ProgIp2   = DefaultIp.ip[1];
  ipprog_reply->ProgIp1   = DefaultIp.ip[2];
  ipprog_reply->ProgIpLo  = DefaultIp.ip[3];
  
  ipprog_reply->ProgSmHi  = DefaultIp.mask[0];
  ipprog_reply->ProgSm2   = DefaultIp.mask[1];
  ipprog_reply->ProgSm1   = DefaultIp.mask[2];
  ipprog_reply->ProgSmLo  = DefaultIp.mask[3];
    
  ipprog_reply->OpCode        = 0xF900; //ARTNET_IPREPLY
  ipprog_reply->ProVerH       = 0;
  ipprog_reply->ProVer        = 14;
  ipprog_reply->ProgPortHi    = DefaultIp.port >> 8;
  ipprog_reply->ProgPortLo    =(DefaultIp.port & 0xFF);
}

void fill_dmx(artnet_dmx_t *dmx)
{
  //fill to 0's
  memset (dmx, 0, sizeof(dmx));
  
  sprintf((char *)dmx->id, "Art-Net\0"); // *** don't change never ***
  dmx->opCode          = 0x5000;  // ARTNET_DMX
  dmx->verH            = 0;        // high byte of Node firmware revision number.
  dmx->ver             = 14;
  dmx->sequence        = 0;
  dmx->physical        = 0;
  dmx->universe        = DefaultAddress.swin [1];
  dmx->lengthHi        = 02;
  dmx->length          = 0;
}

void read_dmx()
{
  #if defined(USE_UNIVERSE_1_TX)
  int n = memcmp ((uint8_t *)ArtDmx.data, (uint8_t *)ArduinoDmx1.RxBuffer, 512);
  if (n != 0)
  {
  memcpy ((uint8_t *)ArtDmx.data, (uint8_t *)ArduinoDmx1.RxBuffer, 512);
  send_reply(BROADCAST, (uint8_t *)&ArtDmx, sizeof(ArtDmx));
  }
  else
  {
  unsigned long currentMillis = millis();
   if(currentMillis - previousMillis > interval) {
     previousMillis = currentMillis;
     send_reply(BROADCAST, (uint8_t *)&ArtDmx, sizeof(ArtDmx));
   }
  }
  #endif
}

void frame_received(uint8_t universe) // Custom ISR: fired when all channels in one universe are received
{
  if (universe == 1) // USART1
  {
    //memcpy ((uint8_t *)ArtDmx.data, (uint8_t *)ArduinoDmx1.RxBuffer, 512);
  }
}  // end of ISR

/*
void serial_print4 (char _text[], uint8_t *param) {
  Serial.print(_text);
  for (int i = 0; i < 4; i++)
  {
    Serial.print(param[i], DEC);
    if (i < 3) {
      Serial.print(".");
    }
  }
  Serial.print("\n");
}*/
