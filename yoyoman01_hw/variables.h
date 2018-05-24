#ifndef _MODULE_VAR_H
#define _MODULE_VAR_H

#include <stdint.h>

//Comment to desactive
#define ACTIV_CODEURS // Direct read of the 4 encremental endoders using STM32 timers

////dans .c: #include "variables.h"
//------------------------IMU---------------------
//Adresses registres
#define X_RATE_adress 0x0400
#define Y_RATE_adress 0x0600
#define Z_RATE_adress 0x0800
#define X_ACC_adress 0x0A00
#define Y_ACC_adress 0x0C00
#define Z_ACC_adress 0x0E00
#define X_MAG_adress 0x1000
#define Y_MAG_adress 0x1200
#define Z_MAG_adress 0x1400
#define PRODUCT_ID_adress 0x5600
#define STDR_REGISTER_adress 0x3400
#define DIAGNOSTIC_STATUS_adress 0x3C00

#define FACTOR_RATE 200
#define FACTOR_ACC 4000
#define FACTOR_MAG 16384

//Variables
#define ZERO 0x0000 //Following all commands in order to allow response (NSS)

#define CS_PIN PA4
//------------------Definition Baudrate --------------------
#define BAUD_AX 1000000
#define BAUD_XM 1000000
#define BAUD_ODRIVE 115200
//------------------SPI --------------------
#ifdef ACTIV_CODEURS
#define SIZE_BUFFER 44
#else
#define SIZE_BUFFER 30
#endif
//------------------Definition des Pins --------------------
#define PIN_DATA_CTRL_AX PG2
#define PIN_DATA_CTRL_XM PG3
//------------------Definition des Moteurs--------------------
#define OD_0 0  //RIGHT
#define OD_1 1  //LEFT
#define AX_1 1  //RIGHT
#define AX_2 2  //LEFT
#define XM_1 1  //RIGHT
#define XM_2 2  //LEFT
#define ALL 254 // Broadcast ID
//------------------Definition des Variables--------------------

//IMU
struct rate_scaled
{ //rad/s
  double rXrate_scaled;
  double rYrate_scaled;
  double rZrate_scaled;
} ratescaled;
struct acc_scaled
{ //g
  double rXacc_scaled;
  double rYacc_scaled;
  double rZacc_scaled;
} accscaled;
struct mag_scaled
{ //gauss
  double rXmag_scaled;
  double rYmag_scaled;
  double rZmag_scaled;
} magscaled;

uint8_t DIAGNOSTIC_STATUS;

//------------------structures--------------------

struct TrameWrite
{ // Variables "write" to send over SPI from the master
  //Odrive
  uint16_t wOd0_pos;
  uint16_t wOd1_pos;
  //AX
  uint16_t wAx1_pos;
  uint16_t wAx2_pos;
  //XM
  uint16_t wXm1_pos;
  uint16_t wXm2_pos;

  uint16_t wspi_test;

  //Inutilisé
  uint16_t a;
  uint16_t b;
  uint16_t c;
  uint16_t d;
  uint16_t e;
  uint16_t f;
  uint16_t g;
  uint16_t h;
  uint16_t i;
  uint16_t k;
  uint16_t l;

  int16_t m;
  int16_t n;
  int16_t o;
  int16_t p;
} wbuffer;

struct TrameRead
{ // Variables "read" to send over SPI from the slave
  //Odrive
  uint16_t rOd0_pos; //read Odrive Right
  uint16_t rOd1_pos;
  //AX
  uint16_t rAx1_pos;
  uint16_t rAx2_pos;
  //XM
  uint16_t rXm1_pos;
  uint16_t rXm2_pos;
  uint16_t rXm1_cur;
  uint16_t rXm2_cur;
  //IMU
  int16_t rate[3];
  int16_t acc[3];
  int16_t mag[3];

  uint16_t rspi_test;

  //  int16_t rXrate;
  //  int16_t rYrate;
  //  int16_t rZrate;
  //  int16_t rXacc;
  //  int16_t rYacc;
  //  int16_t rZacc;
  //  int16_t rXmag;
  //  int16_t rYmag;
  //  int16_t rZmag;
  //Codeurs
  int16_t rCodRMot;
  int16_t rCodRHip;
  int16_t rCodLMot;
  int16_t rCodLHip;

//attention la deniere variable peut ne pas être stable


} rbuffer;

#endif
