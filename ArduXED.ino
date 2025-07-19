/*    ArduXED -- Arduino Xpressnet Einfacher Drehregler -- F. Cañada 2024 --  https://usuaris.tinet.cat/fmco/

      This software and associated files are a DIY project that is not intended for commercial use.
      This software uses libraries with different licenses, follow all their different terms included.

      THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.

      Sources are only provided for building and uploading to the device.
      You are not allowed to modify the source code or fork/publish this project.
      Commercial use is forbidden.

       v0.1     10jan16   Start writting code for PIC12F1822
       v0.2     01feb17   First PCB for PIC12F1822
       v0.3     28oct24   Arduino version (PacoMouse PCB)
       v0.4     03nov24   Working version tested with DR5000
*/


////////////////////////////////////////////////////////////
// ***** USER OPTIONS *****
////////////////////////////////////////////////////////////

// Direccion Xpressnet por defecto de ArduXED - Default Xpressnet address for ArduXED (1..31)
#define DEFAULT_XNET_ADR          9


////////////////////////////////////////////////////////////
// ***** PINS *****
////////////////////////////////////////////////////////////

const int pinLED        = 13;                                 // LED
const int pinSwitch     = A3;                                 // Direction switch
const int pinButton     = A4;                                 // Function button
const int pinTXRX       = A5;                                 // Xpressnet pin
const int pinPot        = A2;                                 // potentiometer

////////////////////////////////////////////////////////////
// ***** END OF USER OPTIONS *****
////////////////////////////////////////////////////////////

/*
    Arduino Nano

    D0  RXD   Xpressnet RXD. (NO CAMBIAR) Serial interface
    D1  TXD   Xpressnet TXD. (NO CAMBIAR) Serial interface
    D2  INT0
    D3
    D4
    D5
    D6
    D7
    D8  ICP
    D9
    D10
    D11
    D12
    D13 LED   LED
    A0
    A1
    A2        Potentiometer   - Potenciometro
    A3        Switch          - Interruptor
    A4  SDA   Pushbutton      - Pulsador
    A5  SCL   Xpressnet TXRX
    A6
    A7
*/

//#define DEBUG                                               // Descomentar para mensajes de depuracion

#include <EEPROM.h>


#ifdef DEBUG
const int pinDEBUG      = A0;
#endif

byte statusButton;
byte statusSwitch;
bool buttonPressed;
bool switchChanged;
const unsigned long timeoutButtons = 50;                      // temporizador antirebote
unsigned long timeButtons;
unsigned int potValue;                                        // current pot value
unsigned int potTotal;                                        // accumulator for filtering
unsigned int lastPot;                                         // last pot value
volatile bool discardPot;
bool waitPotStop;

#define FILTER_LNG  16                                        // must be power of 2 (4, 8, 16, 32,..)
unsigned int potFilter[FILTER_LNG];
byte potFilterIndex;

////////////////////////////////////////////////////////////
// ***** XED - Xpressnet Einfacher Drehregler *****
////////////////////////////////////////////////////////////

enum Settings {
  EE_ADRH, EE_ADRL, EE_XBUS, EE_FUNC,
};                                                            // EEPROM settings

union locomotora {                                            // direccion locomotora
  byte adr[2];
  unsigned int address;
} myLoco;

volatile byte mySteps, myDir, mySpeed, myFuncGrp1;            // locomotora actual
byte myButtonFunction;                                        // funcion asignada a pulsador
byte phaseXED;

enum phase {BOOT_XED, WAIT_SWITCH, WAIT_LOCO, GET_LOCO, RUN_XED};
enum xAnswer {HEADER, DATA1, DATA2, DATA3, DATA4, DATA5};
#define WAIT_FOR_XMIT_COMPLETE {while (!(UCSR0A & (1<<TXC0))); UCSR0A = (1<<TXC0); UCSR0A = 0;}

#define csNormalOps                   0x00                    // estado de la central
#define csEmergencyStop               0x01
#define csEmergencyOff                0x02
#define csStartMode                   0x04
#define csShortCircuit                0x04                    // Z21
#define csServiceMode                 0x08
#define csReserved                    0x10
#define csProgrammingModeActive       0x20                    // Z21
#define csPowerUp                     0x40
#define csErrorRAM                    0x80

volatile byte rxBufferXN[20];                                 // Comunicacion Xpressnet
volatile byte txBuffer[10];
volatile byte txBytes;
volatile byte rxBytes;
volatile byte miCallByte;
volatile bool enviaMensaje;                                   // Envia nuevo mensaje Xpressnet
volatile bool leerDatoXN;
volatile bool searchF0;                                       // buscando cambio en F0
volatile bool checkF0;                                        // recibir posible paquete con F0
volatile bool foundF0;                                        // paquete con F0 activo recibido
volatile byte foundFunctions;                                 // paquete con Funciones recibido
byte rxXOR, rxIndice, txXOR, rxData;
byte csStatus, xnetVersion, xnetCS;

byte miDireccionXpressnet;                                     // Xpressnet bus address
bool getInfoLoco;
unsigned long infoTimer;
unsigned int flashLED;


////////////////////////////////////////////////////////////
// ***** MAIN PROGRAM *****
////////////////////////////////////////////////////////////

void setup() {
  beginHID();                                                 // initialize HID (Human Interface Device) LED, switch, button and potentiometer
  initVariables();                                            // Estado inicial
  beginXpressNet(miDireccionXpressnet);                       // Initialize the Xpressnet interface
  getStatus();                                                // pide estado de la central
}


void loop() {
  byte n;
  switch (phaseXED) {
    case BOOT_XED:
      for (n = 0; n < (FILTER_LNG + 4); n++) {                // initial pot read filtering
        delay(50);
        readPot();
      }
      phaseXED = GET_LOCO;
      if (digitalRead (pinButton) == LOW) {                   // button pressed on power up
        if (statusSwitch == LOW) {
          if (potValue == 0) {                                // dir reverse & stop -> change Xbus addres according pot position
            flashLED = 0xFFFC;
            phaseXED = WAIT_SWITCH;
          }
        }
        else {
          searchF0 = true;                                    // dir forward -> adquire new loco
          flashLED = 0xFFFF;
          phaseXED = WAIT_LOCO;
        }
      }
      break;
    case WAIT_SWITCH:
      if (switchChanged == true) {
        setXbusAddress(potValue & 0x1F);                      // 1..31
        phaseXED = GET_LOCO;
      }
      hidProcess();
      break;
    case WAIT_LOCO:
      if (foundF0) {                                          // wait for F0 activation
        searchF0 = false;
        foundF0 = false;
        myButtonFunction = getSelectedFunction(foundFunctions);
        EEPROM.update(EE_FUNC, myButtonFunction);
        EEPROM.update(EE_ADRH, myLoco.adr[1]);
        EEPROM.update(EE_ADRL, myLoco.adr[0]);
        phaseXED = GET_LOCO;
      }
      hidProcess();
      break;
    case GET_LOCO:
      infoLocomotora(myLoco.address);                         // busca informacion de la locomotora
      setFlash();
      buttonPressed = false;
      switchChanged = false;
      lastPot = potValue;
      //discardPot = true;
      waitPotStop = true;
      phaseXED = RUN_XED;
      break;
    case RUN_XED:
      xpressnetProcess();
      if (buttonPressed)                                      // se ha pulsado el boton
        controlButton();
      if (switchChanged)                                      // se ha movido el interruptor
        controlSwitch();
      controlPot();                                           // control velocidad con potenciometro
      hidProcess();                                           // HID: Human Interface Device
      break;
  }
}

////////////////////////////////////////////////////////////
// ***** SOPORTE *****
////////////////////////////////////////////////////////////

void initVariables() {
  setXbusAddress(EEPROM.read(EE_XBUS));
  myLoco.adr[1] = EEPROM.read(EE_ADRH);                       // Ultima locomotora controlada
  myLoco.adr[0] = EEPROM.read(EE_ADRL);
  checkLocoAddress();
  myButtonFunction = EEPROM.read(EE_FUNC);                    // Funcion para el boton
  if (myButtonFunction > 4)
    myButtonFunction = 0;
  mySteps = bit(1);                                           // default 28 steps
  mySpeed = 0;
  potTotal = 0;
  csStatus = csNormalOps;                                     // Power on
  infoTimer = millis();
  phaseXED = BOOT_XED;
  potFilterIndex = 0;
}


void beginHID() {
  pinMode (pinSwitch, INPUT_PULLUP);                          // HID (Human Interface Device)
  pinMode (pinButton, INPUT_PULLUP);
  statusButton = HIGH;
  statusSwitch = digitalRead(pinSwitch);
  myDir = (statusSwitch == LOW) ? 0x00 : 0x80;
  pinMode (pinLED, OUTPUT);
  digitalWrite (pinLED, LOW);
  analogRead(pinPot);
#ifdef  DEBUG
  pinMode (pinDEBUG, OUTPUT);
  digitalWrite (pinDEBUG, LOW);
#endif
}


void setXbusAddress(byte adr) {
  miDireccionXpressnet = adr;
  if ((miDireccionXpressnet > 31) || (miDireccionXpressnet == 0)) {   // Si no es valido inicializa direccion bus en EEPROM
    miDireccionXpressnet = DEFAULT_XNET_ADR;
    EEPROM.update (EE_XBUS, miDireccionXpressnet);
  }
}

void checkLocoAddress() {
  myLoco.address &= 0x3FFF;
  if ((myLoco.address > 9999) || (myLoco.address == 0))       // Comprueba que este entre 1 y 9999
    myLoco.address = 3;
  if (myLoco.address > 99)                                    // Comprueba si es direccion larga
    myLoco.address |= 0xC000;
}

byte getSelectedFunction (byte funcData) {
  if (funcData & 0x08)
    return 4;
  if (funcData & 0x04)
    return 3;
  if (funcData & 0x02)
    return 2;
  if (funcData & 0x01)
    return 1;
  return 0;
}


////////////////////////////////////////////////////////////
// ***** HID *****
////////////////////////////////////////////////////////////

void hidProcess() {
  if (millis() - timeButtons > timeoutButtons) {              // lectura de boton
    timeButtons = millis();                                   // lee cada cierto tiempo
    readButtons();
    readPot();
    showLED();
  }
}


void readButtons () {
  byte inputButton, inputSwitch;

  inputButton = digitalRead (pinButton);                      // comprueba cambio en boton
  if (statusButton != inputButton) {
    statusButton = inputButton;
    if (statusButton == LOW)
      buttonPressed = true;
  }
  inputSwitch = digitalRead (pinSwitch);                      // comprueba cambio en interruptor sentido
  if (statusSwitch != inputSwitch) {
    statusSwitch = inputSwitch;
    switchChanged = true;
  }
}


void readPot() {
  unsigned int rawPot;
  byte n;
  rawPot = analogRead(pinPot) >> 5;                           // 0..1023 -> 0..31
  potFilter[potFilterIndex++] = rawPot;
  potFilterIndex &= (FILTER_LNG - 1);
  potTotal = 0;
  for (n = 0; n < FILTER_LNG; n++)
    potTotal += potFilter[n];
  potValue = potTotal / FILTER_LNG;
}


void controlPot() {
  if (waitPotStop) {
    if (potValue == 0) {
      waitPotStop = false;
      lastPot = potValue;
    }
  }
  else {
    if (potValue != lastPot) {
      lastPot = potValue;
      if (!discardPot) {                                        // send new speed
        if (bitRead(mySteps, 2)) {                              // 128 steps
          mySpeed = (potValue << 2) & 0x7C;
        }
        else {
          if (bitRead(mySteps, 1)) {                            // 28 steps
            if (potValue < 4)
              mySpeed = 0;
            else {
              mySpeed = (potValue & 0x01) ? ((potValue >> 1) | 0x10) : potValue >> 1;
            }
          }
          else {                                                // 14 steps
            mySpeed = potValue >> 1;
            if (mySpeed < 2)
              mySpeed = 0;
          }
        }
        locoOperationSpeed();
      }
      discardPot = false;
    }
  }
}


void showLED() {                                              // set LED flash sequence
  unsigned int flash;
  flash = flashLED & 0x8000;
  flashLED <<= 1;
  if (flash) {
    flashLED |= 0x0001;
    digitalWrite (pinLED, HIGH);
  }
  else {
    digitalWrite (pinLED, LOW) ;
  }
}


void controlButton() {                                        // pushbutton
  buttonPressed = false;
  funcOperations(myButtonFunction);
}


void controlSwitch() {                                        // switch changes position
  switchChanged = false;
  if (mySpeed > 1)
    mySpeed = 1;                                              // if running, do emergency stop
  myDir = (statusSwitch == LOW) ? 0x00 : 0x80;                // set new direction
  waitPotStop = true;
  locoOperationSpeed();
}


void setFlash() {
  flashLED = 0x0000;
  if (bitRead(mySteps, 3))                                    // Loco controlada por otro mando
    flashLED = 0xF000;
  if (csStatus & csEmergencyOff)
    flashLED = 0xCCCC;
  if (csStatus & csServiceMode)
    flashLED = 0xCC00;
  if (csStatus & csEmergencyStop)
    flashLED = 0xFF00;
}


////////////////////////////////////////////////////////////
// ***** XPRESSNET DECODE *****
////////////////////////////////////////////////////////////

void beginXpressNet (byte xnetAddr) {
  pinMode (pinTXRX, OUTPUT);
  digitalWrite (pinTXRX, LOW) ;                               // recibir datos
  miCallByte = paridadCallByte (xnetAddr);
  enviaMensaje = false;
  leerDatoXN = false;
  rxIndice = 1;
  searchF0 = false;
  checkF0 = false;
  foundF0 = false;
  cli();                                                      // deshabilitar interrupciones para acceder a registros
  UBRR0H = 0;                                                 // UBRR = (FXTAL / (16 *  baud)) - 1 si U2X = 0
  UBRR0L = 0x0F;                                              // Set 62500 baud
  UCSR0A = 0;                                                 // U2X = 0
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UCSZ02); // Enable reception (RXEN) transmission (TXEN0) Receive Interrupt (RXCIE = 1)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                     // UCSZ = b111 = 9 bits
  sei();                                                      // habilitar interrupciones
  getVersion();                                               // pide la version del Xpressnet
}


byte paridadCallByte (byte direccionXN) {
  bool paridad = false;
  byte bits;
  direccionXN &= 0x1F;                                        // Borra bit 7 de la direccion
  bits = direccionXN;
  while (bits) {                                              // mientras haya bits cambia paridad
    paridad = !paridad;
    bits &= (bits - 1);
  }
  if (paridad)                                                // coloca paridad en bit 7
    direccionXN |= 0x80;
  return (direccionXN);                                       // P00AAAAA
}


void enviaUSART (byte data) {
  while (!(UCSR0A & (1 << UDRE0)));                           // esperar a que se pueda enviar
  UDR0 = data;
}


ISR(USART_RX_vect)  {                                         // Interrupcion recepcion datos por Serial
  if (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0))) {    // error en la recepcion?
    rxData = UDR0;
    return;
  }
  if (UCSR0B & (1 << RXB80)) {                                // leer primero 9 bit. Activado: Call Byte
    rxData = UDR0;
    rxIndice = HEADER;
    leerDatoXN = false;
    checkF0 = false;                                          // stop F0 checking
    if (rxData == (miCallByte ^ 0xC0)) {                      // Call Byte: P10AAAAA. Normal Inquiry
      if (enviaMensaje) {                                     // Hay mensaje para enviar?
        delayMicroseconds(48);                                // esperamos el tiempo de tres bit antes de enviar
        digitalWrite (pinTXRX, HIGH);                         // enviamos mensaje
        delayMicroseconds(8);                                 // esperamos el tiempo de medio bit antes de enviar
        for (rxIndice = HEADER; rxIndice < txBytes; rxIndice++)
          enviaUSART (txBuffer[rxIndice]);
        WAIT_FOR_XMIT_COMPLETE
        digitalWrite (pinTXRX, LOW);
        enviaMensaje = false;
      }
    }
    else {
      // Call Byte:   P11AAAAA Message,    P0100000 Feedback BC o P1100000 Broadcast
      if ((rxData == (miCallByte | 0x60)) || (rxData == 0xA0) || (rxData == 0x60)) {
        leerDatoXN = true;
        rxXOR = 0;
      }
      else {
        if (rxData == miCallByte) {                           // Call Byte: P00AAAAA. Request ACK
          delayMicroseconds(32);                              // esperamos el tiempo de dos bit antes de enviar
          digitalWrite (pinTXRX, HIGH);                       // respuesta inmediata si ha habido error de transmision
          delayMicroseconds(8);                               // esperamos el tiempo de medio bit antes de enviar
          enviaUSART (0x20);
          enviaUSART (0x20);
          WAIT_FOR_XMIT_COMPLETE
          digitalWrite (pinTXRX, LOW);
        }
        else {
          if ((rxData & 0x60) == 0x40) {                       // 'P10AAAAA' anything to transmit to other?
            if (searchF0) {                                     // searching F0 change?
              checkF0 = true;                                 // receive possible F0 packet
              leerDatoXN = true;
              rxXOR = 0;
            }
          }
        }
      }
    }
  }
  else {                                                      // 9 bit desactivado: Datos
    rxData = UDR0;
    if (leerDatoXN) {                                         // leer paquete Xpressnet
      rxBufferXN[rxIndice++] = rxData;
      rxXOR ^= rxData;
      if (((rxBufferXN[HEADER] & 0x0F) + 2) == rxIndice) {    // si se han recibido todos los datos indicados en el paquete
        leerDatoXN = false;
        if (rxXOR == 0) {                                     // si el paquete es correcto
          rxBytes = rxIndice;
          if (checkF0) {                                      // packet for other, check F0 packet
            checkF0 = false;
#ifdef DEBUG
            digitalWrite (pinDEBUG, HIGH);
#endif
            if ((rxBufferXN[HEADER] == 0xE4) && (rxBufferXN[DATA1] == 0x20)) { // Function group 1 (F0..F4)
              if (rxBufferXN[DATA4] & 0x10) {                 // F0 on
                myLoco.adr[1] = rxBufferXN[DATA2];            // save loco address
                myLoco.adr[0] = rxBufferXN[DATA3];
                foundFunctions = rxBufferXN[DATA4];           // save loco functions for pusbutton
                //checkLocoAddress();
                foundF0 = true;
                searchF0 = false;
              }
            }
          }
          else {
            procesaXN();                                      // nuevo paquete recibido, procesarlo
          }
        }
      }
    }
  }
}


void procesaXN () {
  switch (rxBufferXN[HEADER]) {                               // segun el header byte
    case 0x61:
      switch (rxBufferXN[DATA1]) {
        case 0x01:                                            // Normal operation resumed                   (0x61,0x01,0x60)
          csStatus = csNormalOps;
          setFlash();
          break;
        case 0x08:                                            // Z21 LAN_X_BC_TRACK_SHORT_CIRCUIT           (0x61,0x08,XOR)
        case 0x00:                                            // Track power off                            (0x61,0x00,0x61)
          csStatus |= csEmergencyOff;
          setFlash();
          break;
        case 0x02:                                            // Service mode entry                         (0x61,0x02,0x63)
          csStatus |= csServiceMode;
          setFlash();
          break;
      }
      break;
    case 0x81:
      if (rxBufferXN[DATA1] == 0) {                           // Emergency Stop                             (0x81,0x00,0x81)
        csStatus |= csEmergencyStop;
        setFlash();
      }
      break;
    case 0x62:
      if (rxBufferXN[DATA1] == 0x22)                          // Command station status indication response (0x62,0x22,DATA,XOR)
        csStatus = rxBufferXN[DATA2] & (csEmergencyStop | csEmergencyOff | csServiceMode) ;
      if ((xnetCS >= 0x10) && (rxBufferXN[DATA2] & csProgrammingModeActive))   // Multimaus/Z21 Service Mode
        csStatus |= csServiceMode;
      setFlash();
      break;
    case 0x63:
      switch (rxBufferXN[DATA1]) {
        case 0x21:                                            // Command station software version           (0x63,0x21,VER,ID,XOR)
          xnetVersion = rxBufferXN[DATA2];
          xnetCS = rxBufferXN[DATA3];
          break;
      }
      break;
    case 0xE3:
      if (rxBufferXN[DATA1] == 0x40) {                        // Locomotive is being operated by another device response  (0xE3,0x40,ADRH,ADRL,XOR)
        if ((rxBufferXN[DATA3] == myLoco.adr[0]) && (rxBufferXN[DATA2] == myLoco.adr[1])) {
          bitSet(mySteps, 3);
          discardPot = true;
          setFlash();
        }
      }
      break;
    case 0xE4:
      if ((rxBufferXN[DATA1] & 0xF0) == 0x00) {               // Locomotive information normal locomotive   (0xE4,ID,SPD,FKTA,FKTB,XOR)
        setLocoInfoData();
      }
      break;
    case 0xE7:
      if ((rxBufferXN[DATA1] & 0xF0) == 0x00) {               // Locomotive function info F13..F20 MM       (0xE7,STP,SPD,FNC,FNC,FNC,0x00,0x00,XOR)
        setLocoInfoData();
      }
      break;
  }
}


void setLocoInfoData() {
  mySteps = rxBufferXN[DATA1];                                // '0000BFFF'
  myDir = rxBufferXN[DATA2] & 0x80;                           // 'RVVVVVVV'
  mySpeed = rxBufferXN[DATA2] & 0x7F;
  myFuncGrp1 = rxBufferXN[DATA3] & 0x1F;                      // '000FFFFF'
  if (bitRead(mySteps, 3))
    discardPot = true;
  setFlash();
}


void headerXN (byte header) {
  while (enviaMensaje);                                       // espera a que se envie el ultimo mensaje
  //enviaMensaje = false;                                     // ahora podemos modificar el buffer
  txBytes = HEADER;                                           // coloca header en el buffer
  txXOR = header;
  txBuffer[txBytes++] = header;
}


void dataXN (byte dato) {
  txBuffer[txBytes++] = dato;                                 // coloca dato en el buffer
  txXOR ^= dato;
}


void sendXN () {
  txBuffer[txBytes++] = txXOR;                                // coloca XOR byte en el buffer
  enviaMensaje = true;
}


void getStatus () {
  headerXN (0x21);                                            // Command station status request         (0x21,0x24,0x05)
  dataXN (0x24);
  sendXN();
}


void getVersion () {
  headerXN (0x21);                                            // Command station software version       (0x21,0x21,0x00)
  dataXN (0x21);
  sendXN();
}


void infoLocomotora (unsigned int loco) {                     // Locomotive information request         (0xE3,0x00,ADRH,ADRL,XOR)
  headerXN (0xE3);
  dataXN (0x00);
  dataXN (highByte(loco));
  dataXN (lowByte (loco));
  sendXN();
  getInfoLoco = false;
}


void locoOperationSpeed() {                                   // Locomotive speed and direction operations (0xE4,ID,ADRH,ADRL,SPD,XOR)
  headerXN (0xE4);
  if (bitRead(mySteps, 2)) {                                  // 128 steps
    dataXN (0x13);
  }
  else {
    if (bitRead(mySteps, 1)) {                                // 28 steps
      dataXN (0x12);
    }
    else {
      dataXN (0x10);                                          // 14 steps
    }
  }
  dataXN (myLoco.adr[1]);
  dataXN (myLoco.adr[0]);
  dataXN (mySpeed | myDir);
  sendXN();
  bitClear(mySteps, 3);                                       // currently operated by me
  setFlash();
}


void funcOperations (byte fnc) {                              // Function operation instructions        (0xE4,ID,ADRH,ADRL,GRP,XOR)
  byte grp;

  grp = (fnc > 0) ? bit(fnc - 1) : 0x10;
  myFuncGrp1 ^= grp;
  headerXN (0xE4);
  dataXN (0x20);                                              // F0..F4
  dataXN (myLoco.adr[1]);
  dataXN (myLoco.adr[0]);
  dataXN (myFuncGrp1 & 0x1F);
  sendXN();
  bitClear(mySteps, 3);                                       // currently operated by me
  setFlash();
}


void xpressnetProcess () {                                    // procesa Xpressnet
  if (getInfoLoco && (csStatus == csNormalOps))
    infoLocomotora(myLoco.address);
  if (millis() - infoTimer > 1000UL) {                        // Cada segundo
    infoTimer = millis();
    if (bitRead(mySteps, 3))                                  // Loco controlada por otro mando
      getInfoLoco = true;                                     // pide info locomotora
  }
}
