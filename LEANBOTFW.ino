#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if !defined(__AVR_ATmega328P__)
#error "LEANBOTFW requires an ATmega328P-based Arduino Nano"
#endif

// Leanbot drive pin map (Arduino Nano / ATmega328P).
constexpr uint8_t LEFT_DIR_PIN = 7;
constexpr uint8_t LEFT_STEP_PIN = 6;
constexpr uint8_t RIGHT_DIR_PIN = 4;
constexpr uint8_t RIGHT_STEP_PIN = 5;

// Current Leanbot firmware uses DIR LOW for positive speed on both motors.
// Change either value after the one-wheel test if your hardware is different.
constexpr bool LEFT_DIRECTION_INVERTED = true;
constexpr bool RIGHT_DIRECTION_INVERTED = true;

constexpr uint32_t SERIAL_BAUD = 115200UL;
constexpr uint16_t MAX_STEP_RATE = 2000U;
constexpr uint16_t COMMAND_TIMEOUT_MS = 2000U;
constexpr uint8_t SERIAL_BYTES_PER_LOOP = 16U;

// Timer2 runs the two step generators at 20 kHz. A STEP pulse is one timer tick
// (50 us), which is deliberately conservative for initial driver bring-up.
constexpr uint16_t STEPPER_TICK_HZ = 20000U;
constexpr uint16_t TIMER2_PRESCALER = 8U;
constexpr uint8_t TIMER2_COMPARE =
    static_cast<uint8_t>((F_CPU / TIMER2_PRESCALER / STEPPER_TICK_HZ) - 1UL);

static_assert(F_CPU == 16000000UL, "This timer setup targets a 16 MHz Nano");
static_assert((F_CPU / TIMER2_PRESCALER / (TIMER2_COMPARE + 1UL)) == STEPPER_TICK_HZ,
              "Timer2 cannot generate the requested stepper tick exactly");

// D4-D7 are all on ATmega328P PORTD, so both STEP pins can change in one write.
constexpr uint8_t LEFT_DIR_MASK = _BV(PD7);
constexpr uint8_t LEFT_STEP_MASK = _BV(PD6);
constexpr uint8_t RIGHT_STEP_MASK = _BV(PD5);
constexpr uint8_t RIGHT_DIR_MASK = _BV(PD4);

struct MotorRuntime {
  volatile int16_t requestedSpeed;
  volatile uint16_t stepRate;
  volatile uint16_t phase;
  volatile int32_t commandedSteps;
  volatile int16_t commandedSpeed;
  volatile int8_t positionDelta;
  volatile uint8_t directionHoldoffTicks;
};

volatile MotorRuntime g_leftMotor = {0, 0, 0, 0, 0, 1, 0};
volatile MotorRuntime g_rightMotor = {0, 0, 0, 0, 0, 1, 0};
volatile uint8_t g_activeStepMask = 0;

char g_lineBuffer[48];
uint8_t g_lineLength = 0;
bool g_lineOverflow = false;
bool g_motionCommandActive = false;
uint32_t g_lastMotionCommandMs = 0;

inline void writeDirectionPin(uint8_t mask, bool high) {
  if (high) {
    PORTD |= mask;
  } else {
    PORTD &= static_cast<uint8_t>(~mask);
  }
}

inline void serviceMotorTick(volatile MotorRuntime &motor, uint8_t directionMask,
                            uint8_t stepMask, bool directionInverted,
                            uint8_t &nextStepMask) {
  // STEP and DIR are changed only here, after the previous pulse was lowered.
  const int16_t speed = motor.requestedSpeed;

  if (speed == 0) {
    motor.stepRate = 0;
    motor.phase = 0;
    motor.commandedSpeed = 0;
    motor.directionHoldoffTicks = 0;
    return;
  }

  const int8_t newDelta = (speed > 0) ? 1 : -1;
  const bool directionHigh = ((speed > 0) != directionInverted);

  if (newDelta != motor.positionDelta) {
    motor.phase = 0;
    // Guarantees two complete 50 us ticks after changing DIR.
    motor.directionHoldoffTicks = 2;
    writeDirectionPin(directionMask, directionHigh);
    motor.positionDelta = newDelta;
  }

  motor.stepRate = static_cast<uint16_t>(speed > 0 ? speed : -speed);
  motor.commandedSpeed = speed;

  if (motor.directionHoldoffTicks != 0) {
    --motor.directionHoldoffTicks;
    return;
  }

  motor.phase += motor.stepRate;
  if (motor.phase >= STEPPER_TICK_HZ) {
    motor.phase -= STEPPER_TICK_HZ;

    if (motor.positionDelta > 0) {
      if (motor.commandedSteps < INT32_MAX) {
        ++motor.commandedSteps;
      }
    } else if (motor.commandedSteps > INT32_MIN) {
      --motor.commandedSteps;
    }

    nextStepMask |= stepMask;
  }
}

void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
  // The ISR applies these requests on its next 50 us tick. It is the sole owner
  // of the STEP and DIR pins, so commands cannot truncate a pulse.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g_leftMotor.requestedSpeed = leftSpeed;
    g_rightMotor.requestedSpeed = rightSpeed;
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
  g_motionCommandActive = false;
}

void beginStepperTimer() {
  // Load known output levels before enabling the pins to avoid startup pulses.
  digitalWrite(LEFT_STEP_PIN, LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  digitalWrite(LEFT_DIR_PIN, LEFT_DIRECTION_INVERTED ? LOW : HIGH);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_DIRECTION_INVERTED ? LOW : HIGH);

  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;
    GTCCR |= _BV(PSRASY);  // Reset the asynchronous Timer2 prescaler.
    TCNT2 = 0;
    OCR2A = TIMER2_COMPARE;
    TIFR2 = _BV(OCF2A) | _BV(OCF2B) | _BV(TOV2);  // Clear stale flags.
    TCCR2A = _BV(WGM21);  // CTC mode.
    TCCR2B = _BV(CS21);   // Prescaler 8.
    TIMSK2 = _BV(OCIE2A);
  }
}

ISR(TIMER2_COMPA_vect) {
  // End the pulses started on the previous tick.
  PORTD &= static_cast<uint8_t>(~g_activeStepMask);
  g_activeStepMask = 0;

  uint8_t nextStepMask = 0;

  serviceMotorTick(g_leftMotor, LEFT_DIR_MASK, LEFT_STEP_MASK,
                   LEFT_DIRECTION_INVERTED, nextStepMask);
  serviceMotorTick(g_rightMotor, RIGHT_DIR_MASK, RIGHT_STEP_MASK,
                   RIGHT_DIRECTION_INVERTED, nextStepMask);

  if (nextStepMask != 0) {
    PORTD |= nextStepMask;
    g_activeStepMask = nextStepMask;
  }
}

bool parseSpeed(const char *text, int16_t &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }

  char *end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (*end != '\0' || parsed < -static_cast<long>(MAX_STEP_RATE) ||
      parsed > static_cast<long>(MAX_STEP_RATE)) {
    return false;
  }

  value = static_cast<int16_t>(parsed);
  return true;
}

void printHelp() {
  Serial.println(F("Leanbot motor test commands:"));
  Serial.println(F("  M <left> <right>  set speed in steps/s (-2000..2000)"));
  Serial.println(F("  S                 stop STEP pulses"));
  Serial.println(F("  P                 print speed and commanded-step counts"));
  Serial.println(F("  Z                 zero commanded-step counts"));
  Serial.println(F("  H                 show this help"));
  Serial.println(F("Motion stops automatically after 2 seconds; resend M to continue."));
}

void printStatus() {
  int16_t leftSpeed;
  int16_t rightSpeed;
  int32_t leftSteps;
  int32_t rightSteps;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftSpeed = g_leftMotor.commandedSpeed;
    rightSpeed = g_rightMotor.commandedSpeed;
    leftSteps = g_leftMotor.commandedSteps;
    rightSteps = g_rightMotor.commandedSteps;
  }

  Serial.print(F("speed L/R: "));
  Serial.print(leftSpeed);
  Serial.print('/');
  Serial.print(rightSpeed);
  Serial.print(F("  commanded steps L/R: "));
  Serial.print(leftSteps);
  Serial.print('/');
  Serial.println(rightSteps);
}

void rejectCommand(const __FlashStringHelper *message) {
  stopMotors();
  Serial.print(F("ERR: "));
  Serial.print(message);
  Serial.println(F(" (STEP pulses stopped)"));
}

void handleCommand(char *line) {
  char *save = nullptr;
  char *command = strtok_r(line, " \t", &save);
  if (command == nullptr) {
    return;
  }

  command[0] = static_cast<char>(toupper(static_cast<unsigned char>(command[0])));

  if (command[0] == 'M' && command[1] == '\0') {
    char *leftText = strtok_r(nullptr, " \t", &save);
    char *rightText = strtok_r(nullptr, " \t", &save);
    char *extra = strtok_r(nullptr, " \t", &save);
    int16_t leftSpeed;
    int16_t rightSpeed;

    if (extra != nullptr || !parseSpeed(leftText, leftSpeed) ||
        !parseSpeed(rightText, rightSpeed)) {
      rejectCommand(F("use M <left> <right>, each from -2000 to 2000"));
      return;
    }

    setMotorSpeeds(leftSpeed, rightSpeed);
    g_motionCommandActive = (leftSpeed != 0 || rightSpeed != 0);
    g_lastMotionCommandMs = millis();
    Serial.print(F("OK M "));
    Serial.print(leftSpeed);
    Serial.print(' ');
    Serial.println(rightSpeed);
    return;
  }

  if (command[1] != '\0' || strtok_r(nullptr, " \t", &save) != nullptr) {
    rejectCommand(F("unknown command or extra argument"));
    return;
  }

  switch (command[0]) {
    case 'S':
      stopMotors();
      Serial.println(F("OK STEP pulses stopped"));
      break;

    case 'P':
      printStatus();
      break;

    case 'Z':
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        g_leftMotor.commandedSteps = 0;
        g_rightMotor.commandedSteps = 0;
      }
      Serial.println(F("OK counters zeroed"));
      break;

    case 'H':
    case '?':
      printHelp();
      break;

    default:
      rejectCommand(F("unknown command; send H for help"));
      break;
  }
}

void serviceSerial() {
  uint8_t remainingBytes = SERIAL_BYTES_PER_LOOP;

  while (remainingBytes-- != 0 && Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r' || incoming == '\n') {
      if (g_lineOverflow) {
        rejectCommand(F("command is too long or contains an invalid byte"));
      } else {
        g_lineBuffer[g_lineLength] = '\0';
        handleCommand(g_lineBuffer);
      }

      g_lineLength = 0;
      g_lineOverflow = false;
      return;  // At most one command per loop, so the timeout cannot starve.
    }

    if (static_cast<uint8_t>(incoming) < 0x20U && incoming != '\t') {
      g_lineOverflow = true;
    } else if (g_lineLength < sizeof(g_lineBuffer) - 1U) {
      g_lineBuffer[g_lineLength++] = incoming;
    } else {
      g_lineOverflow = true;
    }
  }
}

void serviceMotionTimeout() {
  if (g_motionCommandActive &&
      static_cast<uint32_t>(millis() - g_lastMotionCommandMs) >=
          COMMAND_TIMEOUT_MS) {
    stopMotors();
    Serial.println(F("TIMEOUT: STEP pulses stopped"));
  }
}

void setup() {
  beginStepperTimer();
  Serial.begin(SERIAL_BAUD);

  Serial.println();
  Serial.println(F("Leanbot dual-stepper bring-up firmware"));
  Serial.println(F("No STEP pulses; ready. Send H for commands."));
}

void loop() {
  serviceMotionTimeout();
  serviceSerial();
  serviceMotionTimeout();
}
