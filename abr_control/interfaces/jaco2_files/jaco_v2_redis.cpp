const long baudrate = 115200;
const int bufLen = 7;
byte inBuf[bufLen];  // 1B header (1B header already read), 2B motor1, 2B motor2, 2B motorUD
byte hdr1, hdr2;
volatile int wait_to_receive = 0;
boolean received = false;

void setup() {
    Serial.begin(baudrate);
}

void loop() {
    while (Serial.available()) {
        digitalWrite(DEBUG_PINS[0], LOW);
        hdr1 = Serial.read();
        hdr2 = Serial.peek();
        // Check header. If 0xFFFF, it's okay!
        if (hdr1 == 0xFF && hdr2 == 0xFF) {
            Serial.readBytes(inBuf, bufLen);

            // Read in joint angle targets
            temp_F0 = float(((inBuf[1] << 8) + inBuf[2])) / 100.0 - 50.0;
            temp_F1 = float(((inBuf[3] << 8) + inBuf[4])) / 100.0 - 50.0;
            temp_F2 = float(((inBuf[5] << 8) + inBuf[6])) / 100.0 - 50.0;

            // Add velocity term
            // convert velocities from enc/2ms to rad/sec
            // negative for motors 1 and 3 because velocity seemed to be in the wrong direction when plotting against angles
            #ifdef USE_VELOCITY
                force[0] = temp_F0 - m3->kv * (m3->vel * -2000/m3->enc_per_radian);
                force[1] = temp_F1 - m1->kv * (m1->vel * 2000/m1->enc_per_radian);
                force[2] = temp_F2 - m2->kv * (m2->vel * -2000/m2->enc_per_radian);
            #else
                force[0] = temp_F0;
                force[1] = temp_F1;
                force[2] = temp_F2;
            #endif
            #ifdef USE_GRAVITY
                // Compensate for gravity on the mass of the hand
                Fg[0] = 0;
                Fg[1] = -9.81 * m * L[0] * cos(m1->q);
                Fg[2] = -9.81 * m * L[1] * cos(m2->q);
    
                force[0] = force[0] - Fg[0];
                force[1] = force[1] - Fg[1];
                force[2] = force[2] - Fg[2];
            #endif

        }
        else {
            digitalWrite(DEBUG_PINS[0], HIGH);
        }
    }
}
