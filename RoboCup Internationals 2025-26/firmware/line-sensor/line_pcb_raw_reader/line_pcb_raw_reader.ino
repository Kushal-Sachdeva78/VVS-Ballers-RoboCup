/*
  Line PCB — QTR-MD-05A raw reflectance reader (Teensy 4.1)
  ---------------------------------------------------------
  Streams the raw ADC value of all 18 connected sensors so you can
  watch the numbers and learn which reading is "white" on your surface.

  Wiring (from your PCB doc):
    Emitter control CTRL_ODD (shared by all 4 boards) .... Teensy D6
    QTR1 OUT0..OUT4 .................................. A0  A1  A2  A3  A4
    QTR2 OUT0..OUT4 .................................. A5  A6  A7  A8  A9
    QTR3 OUT0..OUT4 .................................. A10 A11 A12 A13 A14
    QTR4 OUT1 OUT2 OUT3  (OUT0 & OUT4 = NC) .......... A15 A16 A17
    Sensors on 3.3V, Teensy on 5V (VIN), common ground.

  ADC: 12-bit -> values range 0..4095.
  No library needed — this is just analogRead().

  Note: analogReadAveraging() and analogReadResolution() are Teensy
  (Teensyduino) functions. Keep this on the Teensy 4.1.
*/

const int EMITTER_PIN = 6;                 // CTRL_ODD shared line
const int NUM_SENSORS = 18;

const int ADC_BITS = 12;                   // 12-bit -> 0..4095
const int ADC_MAX  = (1 << ADC_BITS) - 1;
const unsigned long PRINT_INTERVAL_MS = 150;
const int HEADER_EVERY = 10;               // reprint column header every N updates

// Analog pins in wiring order (board 1 -> board 4)
const int sensorPins[NUM_SENSORS] = {
  A0,  A1,  A2,  A3,  A4,     // QTR1 OUT0..4
  A5,  A6,  A7,  A8,  A9,     // QTR2 OUT0..4
  A10, A11, A12, A13, A14,    // QTR3 OUT0..4
  A15, A16, A17               // QTR4 OUT1, OUT2, OUT3  (OUT0 & OUT4 not connected)
};

// Labels: Q<board>.<output channel>
const char* sensorNames[NUM_SENSORS] = {
  "Q1.0","Q1.1","Q1.2","Q1.3","Q1.4",
  "Q2.0","Q2.1","Q2.2","Q2.3","Q2.4",
  "Q3.0","Q3.1","Q3.2","Q3.3","Q3.4",
  "Q4.1","Q4.2","Q4.3"
};

int values[NUM_SENSORS];
unsigned long lastPrint = 0;
int updateCount = 0;

// right-justify an integer in a field of the given width
void printPadded(int v, int width) {
  int digits = 1, t = (v < 0) ? -v : v;
  while (t >= 10) { t /= 10; digits++; }
  if (v < 0) digits++;
  for (int i = digits; i < width; i++) Serial.print(' ');
  Serial.print(v);
}

// right-justify a string in a field of the given width
void printStrPadded(const char* s, int width) {
  int len = 0;
  while (s[len] != '\0') len++;
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(s);
}

void printHeader() {
  Serial.println();
  for (int i = 0; i < 6; i++) Serial.print(' ');     // align with row label width
  const char* cols[5] = {"OUT0","OUT1","OUT2","OUT3","OUT4"};
  for (int i = 0; i < 5; i++) { Serial.print("  "); printStrPadded(cols[i], 5); }
  Serial.println();
}

// print one board row: label + 5 columns (idx start..start+4)
void printBoardRow(const char* label, int start) {
  printStrPadded(label, 6);
  for (int i = start; i < start + 5; i++) { Serial.print("  "); printPadded(values[i], 5); }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait briefly for USB serial monitor */ }

  analogReadResolution(ADC_BITS);
  analogReadAveraging(16);            // smooth out noise for cleaner readings

  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);   // emitters ON

  Serial.println("QTR-MD-05A raw reader ready.");
  Serial.print("ADC range: 0 to "); Serial.println(ADC_MAX);
  Serial.println("Slide a white surface and a dark surface under each sensor and watch the numbers.");
}

void loop() {
  if (millis() - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = millis();

  // read every sensor
  for (int i = 0; i < NUM_SENSORS; i++) values[i] = analogRead(sensorPins[i]);

  if (updateCount % HEADER_EVERY == 0) printHeader();
  updateCount++;

  printBoardRow("QTR1:", 0);
  printBoardRow("QTR2:", 5);
  printBoardRow("QTR3:", 10);

  // QTR4: OUT0 and OUT4 are NC -> print "--" so the columns stay aligned
  printStrPadded("QTR4:", 6);
  Serial.print("  "); printStrPadded("--", 5);     // OUT0 (NC)
  Serial.print("  "); printPadded(values[15], 5);  // OUT1
  Serial.print("  "); printPadded(values[16], 5);  // OUT2
  Serial.print("  "); printPadded(values[17], 5);  // OUT3
  Serial.print("  "); printStrPadded("--", 5);     // OUT4 (NC)
  Serial.println();

  // quick min/max across all connected sensors (handy for spotting the bright one)
  int minI = 0, maxI = 0;
  for (int i = 1; i < NUM_SENSORS; i++) {
    if (values[i] < values[minI]) minI = i;
    if (values[i] > values[maxI]) maxI = i;
  }
  Serial.print("  min="); printPadded(values[minI], 4);
  Serial.print(" ("); Serial.print(sensorNames[minI]); Serial.print(")");
  Serial.print("   max="); printPadded(values[maxI], 4);
  Serial.print(" ("); Serial.print(sensorNames[maxI]); Serial.println(")");
  Serial.println();
}
