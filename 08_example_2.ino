// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float dist_ema;                     // EMA distance

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started.
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = _DIST_MAX + 10.0;    // Set Higher Value
    analogWrite(PIN_LED, 0);        // LED OFF (maximum brightness, active low)
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = _DIST_MIN - 10.0;    // Set Lower Value
    analogWrite(PIN_LED, 255);      // LED ON (minimum brightness, active low)
  } else if (dist_raw >= 150 && dist_raw <= 250) {
    analogWrite(PIN_LED, 128);      // LED at 50% brightness (active low)
  } else {
    // LED brightness proportional to distance
    int brightness = map(dist_raw, 200, _DIST_MIN, 255, 0);
    analogWrite(PIN_LED, brightness);
  }

  if (dist_raw > _DIST_MAX || dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;
  }
  dist_prev = dist_raw;
  // Modify the below line to implement the EMA equation
  dist_ema = dist_raw;

  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // do something here

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
