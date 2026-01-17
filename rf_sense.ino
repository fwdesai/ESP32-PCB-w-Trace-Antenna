#include <SPI.h>
#include "soc/soc.h"

#define FSPI_CLK 6
#define FSPI_MISO 2
#define FSPI_MOSI 7
#define FSPI_ACCEL_SS 10
#define FSPI_GYRO_SS 8

#define ACCEL_INT 5
#define GYRO_INT 9

#define IMU_BUF_SIZE 128

#define COMP_FILTER_ALPHA 0.01
#define PRINT_PERIOD 200 // ms

typedef struct {
  uint64_t t_us;      // timestamp in microseconds
  //int16_t  gyro[3];  
  int16_t  accel[3];  
} ImuSample;

volatile ImuSample imu_buf[IMU_BUF_SIZE];
volatile uint16_t imu_head = 0;
volatile uint16_t imu_tail = 0;

static const int spiClk = 8000000;

SPIClass *fspi_accel = NULL;


hw_timer_t *timer = NULL;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1);
  pinMode(3, OUTPUT);

  //Set up the data interrupts
  attachInterrupt(digitalPinToInterrupt(ACCEL_INT), imu_drdy_isr, RISING);


  fspi_accel = new SPIClass(FSPI);
  fspi_accel->begin(FSPI_CLK, FSPI_MISO, FSPI_MOSI, FSPI_ACCEL_SS);

  //fspi_gyro = new SPIClass(FSPI);
  //fspi_gyro->begin(FSPI_CLK, FSPI_MISO, FSPI_MOSI, FSPI_GYRO_SS);

  pinMode(FSPI_ACCEL_SS, OUTPUT);
  pinMode(FSPI_GYRO_SS, OUTPUT);

  // turn led on
  digitalWrite(3, HIGH);

  digitalWrite(FSPI_ACCEL_SS, HIGH);
  digitalWrite(FSPI_GYRO_SS, HIGH);

  setUpIMU_accel(fspi_accel);

  // set up the hardware timer
  timer = timerBegin(1000000);    // 1 Mhz timer
  timerStart(timer);

}

void loop() {
  
  while (imu_tail != imu_head) {
    ImuSample s;
    memcpy(&s, (const void *)&imu_buf[imu_tail], sizeof(ImuSample));

    imu_tail = (imu_tail + 1) % IMU_BUF_SIZE;

    static uint64_t t_prev = 0;
      if (t_prev != 0) {
        double dt = (s.t_us - t_prev) * 1e-6;
        // use dt in filter / control loop
      }
      t_prev = s.t_us;

      process_imu(s);
  }


}

void process_imu(const ImuSample &s) {
  static uint64_t prev_print_time = 0;
  if ((timerReadMillis(timer) - prev_print_time) >= PRINT_PERIOD) {
    prev_print_time = timerReadMillis(timer);
    Serial.printf("A_x = %d, A_y = %d, A_z = %d\n", s.accel[0], s.accel[1], s.accel[2]);
  }
  
}

void setUpIMU_accel (SPIClass *spi) {
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  

  // g range for accel
  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer((0x00 | 0x0F));
  spi->transfer(0x03);
  digitalWrite(FSPI_ACCEL_SS, HIGH);

  // set up filtered / unfiltered data out
  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer((0x00 | 0x13));   
  spi->transfer(0x00);            // 0x00 for filtered, 0x80 for unfiltered
  digitalWrite(FSPI_ACCEL_SS, HIGH);

  // filter bandwith
  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer((0x00 | 0x10));
  spi->transfer(0x0B);                  // 0x0C for 125 Hz bandwidth, 0B for 62.5 Hz bandwidth
  digitalWrite(FSPI_ACCEL_SS, HIGH);

  //set up interrupt for data ready
  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer((0x00 | 0x17));
  spi->transfer(0x10);
  digitalWrite(FSPI_ACCEL_SS, HIGH);

  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer((0x00 | 0x1A));
  spi->transfer(0x01);
  digitalWrite(FSPI_ACCEL_SS, HIGH);

  spi->endTransaction();

}

uint8_t readReg(SPIClass *spi, uint8_t reg) {
  digitalWrite(FSPI_ACCEL_SS, LOW);
  spi->transfer(0x80 | reg);
  uint8_t v = spi->transfer(0x00);
  digitalWrite(FSPI_ACCEL_SS, HIGH);
  return v;
}

void imu_read(SPIClass *spi, int16_t *out) {

  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

  uint8_t a_xl = readReg(spi, 0x02);
  uint8_t a_xh = readReg(spi, 0x03);
  uint8_t a_yl = readReg(spi, 0x04);
  uint8_t a_yh = readReg(spi, 0x05);
  uint8_t a_zl = readReg(spi, 0x06);
  uint8_t a_zh = readReg(spi, 0x07);

  spi->endTransaction();

  out[0] = (int16_t)((a_xh << 8) | a_xl);
  out[1] = (int16_t)((a_yh << 8) | a_yl);
  out[2] = (int16_t)((a_zh << 8) | a_zl);

}

void ARDUINO_ISR_ATTR imu_drdy_isr() {
    uint16_t next = (imu_head + 1) % IMU_BUF_SIZE;
    if (next == imu_tail) {
        Serial.println("Buffer full");
        return;
    }

    ImuSample s;
    s.t_us = timerReadMicros(timer); 

    imu_read(fspi_accel, s.accel);

    //Push to buffer
    memcpy((void *)&imu_buf[imu_head], (const void *)&s, sizeof(ImuSample));
    imu_head = next;
}

