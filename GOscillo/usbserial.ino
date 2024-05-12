extern uint8_t capture_buf[];
extern uint32_t trigger_index = 0;

// capture parameters
//uint8_t triglev;         // trigger level 0..255
bool trigger_pos_slope;  // trigger slope false = neg, true = pos
uint8_t trigger_channel; // trigger channel, 0=ch1, 1=ch2
uint8_t dma_multiplier;  // dma_count multiplier, multiple of SAMPLES in hex2 == max acquiring time
uint8_t round_robin;     // 1 = ch1, 2=ch2, 3=ch1+ch2
uint32_t samp_speed_divider;
uint32_t hor_trig_pct;   // horizontal trigger % (10..90)
uint32_t extra1;
uint32_t extra2;

int32_t txcount = 0;
uint8_t hexdigits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
bool blockdone = true;

uint32_t decodeHex(uint8_t * p, uint8_t len) { // convert hex to int
  uint32_t result = 0;
  for (int i = 0; i < len; i++) {
    uint8_t x = *p++ - 48;
    if (x > 9) x -= 7;
    result = result * 16 + x;
  }
  return result;
}

void write_hex(uint8_t x) {
  Serial.write(((byte)hexdigits[x >> 4]));
  Serial.write(((byte)hexdigits[x & 0x0f]));
}

void write_hex4(uint32_t x) {
  Serial.write((byte)(hexdigits[x >> 12 & 0x0f]));
  Serial.write((byte)(hexdigits[x >> 8  & 0x0f]));
  Serial.write((byte)(hexdigits[x >> 4  & 0x0f]));
  Serial.write((byte)(hexdigits[x & 0x0f]));
}

void write_block() {
  int samples;
  samples = round_robin > 2 ? 2 * SAMPLES : SAMPLES;
  if (txcount >= samples) {
    Serial.write("*/");
    blockdone = true;
  }
  int32_t len = samples - txcount;
  if (len > 64) len = 64;
  if (round_robin == 1) {
    for (int i = 0; i < len; i++) write_hex(data[0][txcount + i]);
  } else if (round_robin == 2) {
    for (int i = 0; i < len; i++) write_hex(data[1][txcount + i]);
  } else {
    int txcount2 = txcount / 2;
    for (int i = 0; i < len/2; i++) {
      write_hex(data[0][txcount2 + i]);
      write_hex(data[1][txcount2 + i]);
    }
  }
  txcount += len;
  delay(1);
}

void tx_complete(void) {
  if (!blockdone) write_block();
}

static void cdc_task(void)
{
  if ( Serial.available() ) {
    uint8_t buf[256];
    uint32_t count = 0;
    while (Serial.available()) {
      buf[count++] = Serial.read();
      if (count >= sizeof(buf)) break;
      delay(1);
    }

    // request format: '*',command,data,'/'

    // command 'm'
    // byte 0 = '*'
    // byte 1 = 'm'  start measuring cycle
    // byte 2 = '0'..'9', sequence code
    // byte 3 = '/'

    // command 'p'
    // byte 0 = '*
    // byte 1 = 'p' // parameters for capturing analog data
    // byte 2,3 = trigger level in hex2
    // byte 4,5,6,7 = sample speed divider in hex4 (96=500kS/s, 47999 = 1kS/s)
    // byte 8 = trigger slope '0' = neg, '1' = pos
    // byte 9 = trigger channel '0' = ch1, '1' = ch2
    // byte 10,11 = dma_count multiplier, multiple of SAMPLES in hex2 == max acquiring time
    // byte 12 = channelselect : '1' = ch1, '2' = ch2, '3' = ch1 + ch2
    // byte 13,14 = horTrigPct, horizontal trigger position, 10..90% from sample range, value 10..90 in hex2
    // byte 15 = '/'

    // command 'f'  
    // byte 0 = '*'
    // byte 1 = 'f'  // pwm setting
    // byte 2,3,4,5 = frequency in Hz
    // byte 6,7 = duty cycle hex 2 0..100 % 
    // byte 8 = '/'

    if (count >= 4 && buf[0] == '*' && buf[count - 1] == '/') {
      uint8_t command = buf[1];
      if (command == 'm') {
//        capture();
        blockdone = false;
        txcount = 0;
        Serial.write("/*m");
        Serial.write(buf[2]);
        write_hex4(trigger_index);
        write_hex4(extra1);  // extra data 1
        write_hex4(extra2);  // extra data 2
      } else if (command == 'p') {
        trig_lv = (uint8_t)decodeHex(&buf[2], 2);
        samp_speed_divider = decodeHex(&buf[4], 4);
        rate_value = samp_speed_divider;
        if (rate_value > RATE_MAX) rate_value = RATE_MAX;
        rate_flag = true;
        trigger_pos_slope = buf[8] == '1';
        trig_edge = trigger_pos_slope ? TRIG_E_UP: TRIG_E_DN;
        trigger_channel = buf[9] == '1';
        trig_ch = trigger_channel ? ad_ch0 : ad_ch1;
        dma_multiplier = decodeHex(&buf[10], 2);
        round_robin = buf[12] - 48; // 1,2, or 3
        hor_trig_pct = decodeHex(&buf[13], 2) ;
        if (round_robin == 1) {
          ch0_mode = MODE_ON;
          ch1_mode = MODE_OFF;
        } else if (round_robin == 2) {
          ch0_mode = MODE_OFF;
          ch1_mode = MODE_ON;
        } else {
          ch0_mode = MODE_ON;
          ch1_mode = MODE_ON;
        }
      } else if (command =='f') {
        uint16_t freq = decodeHex(&buf[2],4);
        uint16_t iduty = decodeHex(&buf[6],2);
        iduty = map(iduty, 0, 100, 0, 255);
        duty = constrain(iduty, 0, 255);
        set_pulse_frq((float) freq);
      }
    }
  }
  tx_complete();
}

void send_data() {
  blockdone = false;
  txcount = 0;
  Serial.write("/*m");
  Serial.write('0');
  write_hex4(trigger_index);
  write_hex4(extra1);  // extra data 1
  write_hex4(extra2);  // extra data 2
  while (blockdone == false) {
    tx_complete();
  }
}
