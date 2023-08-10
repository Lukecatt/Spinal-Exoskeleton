unsigned char sendData[17];
unsigned char rcvData[16];
float tCur, spdCur, posCur;
char temp, error;
uint16_t forceCur;
const float normalTorque = 0.15; // torque that the motor exerts on default
const float liftTorque = 0.4; // torque that the motor exerts when user is bending back up
float posUpright;


uint16_t const crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static inline uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}
/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

void createSend(unsigned char *data, unsigned char mode, float t, float spd, float pos, float kp, float kspd) {
  int32_t m;
  unsigned char *ptr;
  data[0] = 0xFE;
  data[1] = 0xEE;

  data[2] = mode;
  data[2] = data[2] << 4;

  m = t * 256;
  ptr = (unsigned char*)&m;
  data[3] = ptr[0];
  data[4] = ptr[1];

  m = (spd / (2*3.1415926535)) * 256;
  ptr = (unsigned char*)&m;
  data[5] = ptr[0];
  data[6] = ptr[1];

  m = (pos / (2 * 3.1415926535)) * 32768;
  ptr = (unsigned char*)&m;
  data[7] = ptr[0];
  data[8] = ptr[1];
  data[9] = ptr[2];
  data[10] = ptr[3];

  m = kp * 1280;
  ptr = (unsigned char*)&m;
  data[11] = ptr[0];
  data[12] = ptr[1];

  m = kspd * 1280;
  ptr = (unsigned char*)&m;
  data[13] = ptr[0];
  data[14] = ptr[1];

  m = crc_ccitt(0, data, 15);
  ptr = (unsigned char*)&m;
  data[15] = ptr[0];
  data[16] = ptr[1];
}

void send(unsigned char mode, float t, float spd, float pos, float kp, float kspd) {
  digitalWrite(14, HIGH);
  createSend(sendData, mode, t, spd, pos, kp, kspd);
  Serial1.write(sendData, 17);
  ets_delay_us(43); // baud rate 4000000 => 1/4000000=2.5*10^-10 = 250ns; 250ns * 10 bits/byte * 17 byte = 42.5 microseconds, approx. 43
}

int32_t tmp;
bool rcv() {
  digitalWrite(14, LOW);
  if (Serial1.available() >= 16) {
    Serial1.readBytes(rcvData, 16);
    if (rcvData[0] != 0xFD || rcvData[1] != 0xEE) {
    return false;
    }
    uint16_t crc = crc_ccitt(0, rcvData, 14);
    uint16_t crcRcv = (rcvData[15] << 8) + rcvData[14];
    if (crc != crcRcv) {
      return false;
    }
    // Serial.write(rcvData, 16);
    tmp = rcvData[4];
    tmp = (tmp << 8) + rcvData[3];
    tCur = (float)tmp / 256.0;

    tmp = rcvData[6];
    tmp = (tmp << 8) + rcvData[5];
    spdCur = ((float)tmp / 256.0) * 60.0; // rpm

    tmp = rcvData[10];
    tmp = tmp << 24;
    int32_t tmp2 = rcvData[9];
    tmp2 = tmp2 << 16;
    int16_t tmp3 = rcvData[8];
    tmp3 = tmp3 << 8;
    posCur = (float)(tmp + tmp2 + tmp3 + rcvData[7]) / 32768.0;
    posCur *= 6.2829;
    posCur += 50;

    temp = rcvData[11];
    error = rcvData[12] & 0b111;
    rcvData[12] = rcvData[12] >> 3;
    forceCur = rcvData[13] & 0b1111111 << 5 + rcvData[12];
  }
  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(4000000);
  pinMode(14, OUTPUT); // turn on output pin to transfer data to motor
  digitalWrite(14, HIGH);
  pinMode(11, OUTPUT); // turn on light to indicate ESP32 is running
  digitalWrite(11, HIGH);
  float sum = 0;
  send(1, normalTorque, 0, 0, 0, 0); // initialize motor
  delay(1000); // wait for motor to move to its position
  for (int i=0; i<5; i++) {
    send(1, normalTorque, 0, 0, 0, 0); // calculate average position
    rcv();
    sum += posCur;
    delay(50);
  }
  posUpright = sum / 5; // position of motor when user is upright

}

bool lock = false;
float posPrev1 = 10000, posPrev2 = 10000;
float tSend = 0.2;
unsigned char cnt = 0;
void loop() {
  Serial.print(posCur);
  Serial.print(",");
  Serial.print(posUpright);
  Serial.print(",");
  Serial.println(tSend);
  if (!lock) {  // prevents motor from changing its torque too often when it is pulling the user upright
    if (posPrev1 < posCur-5 && posPrev2 < posCur-10 && posCur < posUpright - 2) { // position of motor must be upwards trend and not close to posUpright (not when the user is close to upright)
      lock = true;
      tSend = liftTorque;
    } else {
      tSend = normalTorque;
    }
  } else { // delay 50 ms * 10 = 500ms lock time
    cnt ++;
    if (cnt == 10) {
      lock = false;
      cnt = 0;
    }
  }
  posPrev2 = posPrev1;
  posPrev1 = posCur;
  send(1, tSend, 0, 0, 0, 0);
  rcv();
  delay(50);
}
