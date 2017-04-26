/**
 * Example code for using a microchip mrf24j40 module to receive only
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */

/////////////////////////
//ROUTER
/////////////////////////

#include <SPI.h>
#include <mrf.h>
#include <OneWire.h> 
#include <DallasTemperature.h>

#define PANID 0xcafe
#define SERVER_ADDRESS 0x4201
#define SOURCE_ADDRESS 0x4202
#define ROUTER_ADDRESS 0x4204
#define Pin 3 //pin sensor temperatura agua

int reenviar = 0;
const int pin_reset = 6;
const int pin_cs = 7; // default CS pin on ATmega8/168/328
const int pin_int = 2; // default interrupt pin on ATmega8/168/328

Mrf24j mrf(pin_reset, pin_cs, pin_int);
OneWire ourWire(Pin); //Se establece el pin declarado como bus para la comunicación OneWire
DallasTemperature sensors(&ourWire); //Se instancia la librería DallasTemperature

long last_time;
long tx_interval = 10000;

void setup() {
  Serial.begin(9600);
  
  mrf.reset();
  mrf.init();
  mrf.set_pan(PANID);
  // This is _our_ address
  mrf.address16_write(ROUTER_ADDRESS); 

  // uncomment if you want to receive any packet on this channel
  mrf.set_promiscuous(true);
  //mrf.set_coordinador(true);
  
  // uncomment if you want to enable PA/LNA external control
  mrf.set_palna(true);
  
  // uncomment if you want to buffer all PHY Payload
  //mrf.set_bufferPHY(true);

  attachInterrupt(digitalPinToInterrupt(pin_int), interrupt_routine, FALLING); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();

  Serial.print("La pan leida es: ");
  Serial.println(mrf.get_pan());
  Serial.print("El numero del disp es: ");
  Serial.println(mrf.address16_read());
  sensors.begin(); //Se inician los sensores
  Serial.println("Sensor begin");
  
  Serial.println("Finalizo el SetUp.");
}

void interrupt_routine() {
  mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
    sensors.requestTemperatures(); //Prepara el sensor para la lectura
    float temperatura = sensors.getTempCByIndex(0); 
    mrf.check_flags(&handle_rx, &handle_tx);
    unsigned long current_time = millis();
    //int nro = 123;
    char str[25];
    if (current_time - last_time > tx_interval) {
      last_time = current_time;
      Serial.println("enviando al server datos sensados");
      sprintf(str, "H%dV%d!", ROUTER_ADDRESS,temperatura);
      mrf.send16(SERVER_ADDRESS, str);
      if (reenviar){
        Serial.println("reenviando al server");
        mrf.send16(SERVER_ADDRESS,mrf.get_rxinfo()->rx_data );
        reenviar=0;
      
    }
    }
}

void handle_rx() {
    Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    Serial.println("\r\nASCII data (relevant data):");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    Serial.print("\r\n Source");
    Serial.print(mrf.get_rxinfo()->src_addr16, DEC);
    Serial.print("\r\n Destino");
    Serial.print(mrf.get_rxinfo()->dst_addr16, DEC);
    if (mrf.get_rxinfo()->dst_addr16==ROUTER_ADDRESS)
      reenviar = 1;
    else
      reenviar = 0;
     
    Serial.print("\r\nLQI/RSSI=");
    Serial.print(mrf.get_rxinfo()->lqi, DEC);
    Serial.print("/");
    Serial.println(mrf.get_rxinfo()->rssi, DEC);

    
      
    
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }    
}
