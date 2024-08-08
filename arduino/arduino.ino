

const unsigned long freq = 400;
const unsigned long period = 1000000 / freq;
const unsigned long idle_time = 1;
unsigned long start;
unsigned long i;
unsigned long frame = 0;
unsigned long max = 0;
unsigned long idle_counter;
bool idle = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(12, OUTPUT);
  Serial.begin(115200);
  start = micros();
  i = 0;
  idle_counter = 0;
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(100);
  // digitalWrite(LED_BUILTIN, LOW);
}

char buf[256];
void loop() {
  // put your main code here, to run repeatedly:
  while (micros() - start < period)
    ;
  if (frame < max) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(12, HIGH);
    unsigned long ts = micros();
    snprintf(buf, sizeof buf, "frame %lu %lu\n", frame, ts);
    Serial.print(buf);
    // snprintf(buf, sizeof buf, "compare %lu-%lu=%lu>=%lu\n", micros(), start,
    //          micros() - start, period);
    // Serial.print(buf);
    frame++;
    while (micros() - start < period / 2 + period)
      ;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(12, LOW);
  } else if (idle_counter % (freq * idle_time) == freq * idle_time - 1) {
    idle = true;
    Serial.print("ping\n");
    // } else if (!idle) {
    //   snprintf(buf, sizeof buf, "skip %lu", start + i * period);
    //   Serial.println(buf);
  }
  while (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (1 == sscanf(data.c_str(), "pong %lu", &max)) {
      if (idle) {
        frame = 0;
        Serial.write("reset\n");
        start = micros() - period;
        snprintf(buf, sizeof buf, "start %lu\n", start);
        Serial.write(buf);
      }
      idle_counter = 0;
      idle = false;
      // Serial.println(data);
    }
  }
  idle_counter++;
  start += period;
}
