/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_Binary
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithCallbacks,
 *  muestra cómo es posible comunicar los parámetros de 
 *  configuración del transceiver entre nodos LoRa en
 *  formato binario *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>

#define TX_LAPSE_MS         10000

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0x70;     // Dirección de este dispositivo
uint8_t destination = 0x71;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;
volatile bool flag_ack_wait = false;
volatile bool flag_cambio = false;
volatile bool sendImprovedConfig = true;
volatile uint16_t lastMsgID;
volatile int thresholdRSSI = -80;   // Umbral para el RSSI [0, -127]
volatile float thresholdSNR = -10;  // Umbral para el SNR [20, -148]
volatile uint32_t lastReceivedTime_ms = 0;
volatile bool firstConnection = false;

int tiempo_max = 50000;
int dummyCounter = 0;
int dummyTotalReceivedRSSI = 0;
float dummyTotalReceivedSNR = 0;



// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 2, 10, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
LoRaConfig_t initialNodeConf = { 2, 10, 5, 2};
int remoteRSSI = 0;
float remoteSNR = 0;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);  
  while (!Serial); 

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN); 
                                  // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor
  LoRa.setSyncWord(0x72);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  
  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);
  
  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  // Nótese que la recepción está activada a partir de este punto
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() 
{
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 1;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;
   
  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms) && !flag_ack_wait) {

    uint8_t payload[50];
    uint8_t payloadLength = 0;
    //actualizar cambios, pero no aplicar
    sendImprovedConfig ? improvethisNodeConfiguration() : worsenthisNodeConfiguration();

    payload[payloadLength]    = (thisNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength]    = ((thisNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

    // Incluimos el RSSI y el SNR del último paquete recibido
    // RSSI puede estar en un rango de [0, -127] dBm
    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    // SNR puede estar en un rango de [20, -148] dBm
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());
    
    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    lastMsgID = msgCount;
    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);
  }                  
  
  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");
    
    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms; 
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;
    
    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 10;
    }

    transmitting = false;
    flag_ack_wait = true;
    // Reactivamos la recepción de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    if(flag_cambio){
      //aplicar_cambios();
      changeMasterConfiguration();
      flag_cambio = false;
    }
    LoRa.receive(); 
  }
  /*if((millis() - lastSendTime_ms) > tiempo_max){
    depuration();
    delay(100000);
  }*/


  if(((millis() - lastReceivedTime_ms) > tiempo_max) && !firstConnection) {
    goBackToInitialConf();
  }

}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(payloadLength);              // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // Añadimos el mensaje/payload 
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  firstConnection = false;

  if (transmitting && !txDoneFlag) txDoneFlag = true;
  
  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                   // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // Dirección del destinatario
  uint8_t sender = LoRa.read();         // Dirección del remitente
                                        // msg ID (High Byte first)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | 
                            (uint16_t)LoRa.read();
  
  uint8_t incomingLength = LoRa.read(); // Longitud en bytes del mensaje
  
  uint8_t receivedBytes = 0;            // Leemos el mensaje byte a byte

  lastReceivedTime_ms = millis();
  Serial.print("LAST RECEIVED:");
  Serial.println(lastReceivedTime_ms);
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer)-1))) {            
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  
  if (incomingLength != receivedBytes) {// Verificamos la longitud del mensaje
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;                             
  }
  

  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido específicamente a este dispositivo.
  // Nótese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay más de dos receptores activos
  // compartiendo la misma palabra de sincronización
  if ((recipient & localAddress) != localAddress ) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }
  if (sender == localAddress) {
    Serial.println("Receiving error: This message was sent from me.");
    LoRa.receive();
    return;
  }

  if (incomingMsgId == 0) {
    Serial.print("RECEIVED DUMMY PACKET N");
    Serial.println(dummyCounter + 1);
  }

  // Imprimimos los detalles del mensaje recibido
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("ACK ID: " + String(incomingMsgId));
  Serial.println("Payload length: " + String(incomingLength));
  Serial.print("Payload: ");
  printBinaryPayload(buffer, receivedBytes);
  int receivedRSSI = LoRa.packetRssi();
  float receivedSNR = LoRa.packetSnr();
  Serial.print("\nRSSI: " + String(receivedRSSI));
  Serial.print(" dBm\nSNR: " + String(receivedSNR));
  Serial.println(" dB");
  Serial.println();

  

  if (incomingMsgId == lastMsgID) { // Recibiendo ACK del esclavo
    flag_ack_wait = false;
    int meanRSSI = dummyTotalReceivedRSSI/dummyCounter;
    float meanSNR = dummyTotalReceivedSNR/dummyCounter;
    Serial.print("Media del RSSI de dummy packages: ");
    Serial.println(meanRSSI);
    Serial.print("Media del SNR de dummy packages: ");
    Serial.println(meanSNR);
    if (checkReceivedRSSIandSNR(meanRSSI, meanSNR)) {
      sendImprovedConfig = true;
    } else {
      sendImprovedConfig = false;
    }
    dummyCounter = 0;
    dummyTotalReceivedRSSI = 0;
    dummyTotalReceivedSNR = 0;
  } else if (incomingMsgId == 0) {  // ID = 0 significa que recibe un paquete dummy
    dummyTotalReceivedRSSI += receivedRSSI;
    dummyTotalReceivedSNR += receivedSNR;
    dummyCounter++;
    
  }
}

void TxFinished()
{
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t * payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}
void depuration(){
  Serial.print("ack_wait: ");
  Serial.println(flag_ack_wait);

  Serial.print("txDoneFlag: ");
  Serial.println(txDoneFlag);

  Serial.print("transmitting: ");
  Serial.println(transmitting);
}

void aplicar_cambios(){
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
}

bool checkReceivedRSSIandSNR(int rssi, float snr) {
  flag_cambio = true;
  if (rssi > thresholdRSSI && snr > thresholdSNR) {
    return true;
  }
  return false;
  
}

void improvethisNodeConfiguration() {
  Serial.println("aumento prestaciones");
  flag_cambio = true;
  thisNodeConf = {increaseBandwidthIndex(thisNodeConf.bandwidth_index),
                 decreaseSpreadingFactor(thisNodeConf.spreadingFactor),
                 decreaseCodingRate(thisNodeConf.codingRate),
                 decreaseTxPower(thisNodeConf.txPower)};
}

void worsenthisNodeConfiguration() {
  Serial.println("disminuyo prestaciones");
  flag_cambio = true;
  thisNodeConf = {decreaseBandwidthIndex(thisNodeConf.bandwidth_index),
                 increaseSpreadingFactor(thisNodeConf.spreadingFactor),
                 increaseCodingRate(thisNodeConf.codingRate),
                 increaseTxPower(thisNodeConf.txPower)};
}

int increaseBandwidthIndex(int index) {
  return index == 9 ? index : ++index;
}

int decreaseBandwidthIndex(int index) {
  return index == 0 ? index : --index;
}

int increaseSpreadingFactor(int spf) {
  return spf == 12 ? spf : ++spf;
}

int decreaseSpreadingFactor(int spf) {
  return spf == 7 ? spf : --spf;
}

int increaseCodingRate(int rate) {
  return rate == 8 ? rate : ++rate;
}

int decreaseCodingRate(int rate) {
  return rate == 5 ? rate : --rate;
}

int increaseTxPower(int power) {
  return power == 20 ? power : ++power;
}

int decreaseTxPower(int power) {
  return power == 2 ? power : --power;
}
void changeMasterConfiguration() {
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index]));
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
  LoRa.setCodingRate4(thisNodeConf.codingRate);
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

void goBackToInitialConf() {
  Serial.println("Going back to initial conf");
  thisNodeConf = initialNodeConf;
  transmitting = false;
  flag_ack_wait = false;
  txDoneFlag = true;
  firstConnection = true;
  changeMasterConfiguration();
}