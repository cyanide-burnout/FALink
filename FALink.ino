/****************************************
 * Copyright 2011 Artem Prilutskiy R3ABM
 ****************************************/

#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>

/****************************************
 * EEPROM
 ****************************************/

void ReadMemory(int address, void* buffer, size_t size)
{
  for (size_t index = 0; index < size; index ++)
    ((byte*)buffer)[index] = EEPROM.read(address + index);
}

void WriteMemory(int address, void* buffer, size_t size)
{
  for (size_t index = 0; index < size; index ++)
    EEPROM.write(address + index, ((byte*)buffer)[index]);
}

/****************************************
 * CI-V input routines
 ****************************************/

// CI-V Protocol Specification
// http://www.plicht.de/ekki/civ/civ-p0a.html

// http://www.seed-solutions.com/gregordy/Amateur%20Radio/Experimentation/CIVInterface.htm

#define CIV              Serial3
#define CIV_BAUD_RATE    9600
#define CIV_BUFFER_SIZE  16

#define CIV_START        0xfe
#define CIV_TAIL         0xfd

unsigned char message[CIV_BUFFER_SIZE];
int position = 0;

boolean ReadCIV()
{
  while (CIV.available())
  {
    int data = CIV.read();
    if ((data == CIV_START) && (position > 1))
      position = 0;
    if (position < CIV_BUFFER_SIZE)
      message[position] = data;
    if (data == CIV_TAIL)
      return true;
    position ++;
  }
  return false;
}

#define CIV_HEADER_SIZE               5
#define CIV_FREQUENCY_SIZE            5

#define CIV_ADDRESS_ALL_RIGS          0x00
#define CIV_ADDRESS_CONTROLLER        0xe0

#define CIV_COMMAND_SET_FREQUENCY_NC  0x00
#define CIV_COMMAND_SET_FREQUENCY     0x05
#define CIV_COMMAND_READ_FREQUENCY    0x03

#define COMMAND_NONE                  0
#define COMMAND_FREQUENCY_UPDATED     1

unsigned char store[CIV_FREQUENCY_SIZE] = { 0, 0, 0, 0, 0 };

int ReadCommand()
{
  if ((message[0] == CIV_START) && (message[1] == CIV_START))
  {
    switch (message[4])
    {
      case CIV_COMMAND_SET_FREQUENCY_NC:
      case CIV_COMMAND_SET_FREQUENCY:
      case CIV_COMMAND_READ_FREQUENCY:
        if ((position > (CIV_HEADER_SIZE + 1)) && (position < (CIV_HEADER_SIZE + CIV_FREQUENCY_SIZE + 1)))
        {
          // Frequency could be updated partialy
          // so we are useing buffer to store frequency values
          int length = position - CIV_HEADER_SIZE;
          memcpy(store, &message[5], length);
          return COMMAND_FREQUENCY_UPDATED;
        }
        break;
      default:  
        break;
    }
  }
  return COMMAND_NONE;
}

#define CIV_FREQUENCY_LOW_BYTE   0
#define CIV_FREQUENCY_HIGH_BYTE  (CIV_FREQUENCY_SIZE - 1)

long ReadFrequency()
{
  // Frequency stored in BCD format
  // 0x76 0x23 0x28 0x45 0x01 = 145.282.376
  long value = 0;
  for (int index = CIV_FREQUENCY_HIGH_BYTE; index >= CIV_FREQUENCY_LOW_BYTE; index --)
    value = value * (long)100 + (long)((store[index] >> 4) * 10 + (store[index] & 15));
  return value;
}

boolean InProgress()
{
  return (position > 0) && (position < CIV_BUFFER_SIZE) && (message[position] != CIV_TAIL);
}

/****************************************
 * GIO communication routines
 ****************************************/

// MCP23017

#define MCP_IODIRA        0x00
#define MCP_IOPOLA        0x01
#define MCP_GPINTENA      0x02
#define MCP_DEFVALA       0x03
#define MCP_INTCONA       0x04
#define MCP_IOCON         0x05
#define MCP_IOCONA        0x05
#define MCP_GPPUA         0x06
#define MCP_INTFA         0x07
#define MCP_INTCAPA       0x08
#define MCP_GPIOA         0x09
#define MCP_OLATA         0x0A
#define MCP_IODIRB        0x10
#define MCP_IOPOLB        0x11
#define MCP_GPINTENB      0x12
#define MCP_DEFVALB       0x13
#define MCP_INTCONB       0x14
#define MCP_GPPUB         0x16
#define MCP_INTFB         0x17
#define MCP_INTCAPB       0x18
#define MCP_GPIOB         0x19
#define MCP_OLATB         0x1A

#define MCP_IOCON_INTPOL  B00000010
#define MCP_IOCON_ODR     B00000100
#define MCP_IOCON_HAEN    B00001000
#define MCP_IOCON_DISSLW  B00010000
#define MCP_IOCON_SEQOP   B00100000
#define MCP_IOCON_MIRROR  B01000000
#define MCP_IOCON_BANK    B10000000

// PCA9555

#define PCA_IN_P0         0x00
#define PCA_IN_P1         0x01
#define PCA_OUT_P0        0x02
#define PCA_OUT_P1        0x03
#define PCA_INV_P0        0x04
#define PCA_INV_P1        0x05
#define PCA_CONFIG_P0     0x06
#define PCA_CONFIG_P1     0x07

// Generic I/O

// 0x27 is default address for the DFR0013 board with no jumpers.
// 0x20 is address for the DFR0013 board with all jumpers.
#define GIO_ADDRESS       0x27           // B0100000
#define GIO_DIRECTION_A   PCA_CONFIG_P0  // MCP_IODIRA
#define GIO_DIRECTION_B   PCA_CONFIG_P1  // MCP_IODIRB
#define GIO_OUTPUT_A      PCA_OUT_P0     // MCP_GPIOA
#define GIO_OUTPUT_B      PCA_OUT_P1     // MCP_GPIOB

void WriteGIO(int address, byte option, byte value)
{
  address |= GIO_ADDRESS;
  Wire.beginTransmission(address);
  Wire.write(option);
  Wire.write(value);
  Wire.endTransmission();
}

byte ReadGIO(int address, byte option)
{
  address |= GIO_ADDRESS;
  Wire.beginTransmission(address);
  Wire.write(option);
  Wire.endTransmission();

  Wire.requestFrom(address, 1);
  while (Wire.available() < 1);
  return Wire.read();
}

/****************************************
 * FA control routines
 ****************************************/
 
#define FA_LOWER_BOUND    0
#define FA_UPPER_BOUND    767

#define FA_POWER_OFF      0
#define FA_POWER_ON       0x800

#define FA_IIC_ADDRESS    0

void InitializeFA()
{
  Wire.begin();
  WriteGIO(FA_IIC_ADDRESS, GIO_DIRECTION_A, 0);
  WriteGIO(FA_IIC_ADDRESS, GIO_DIRECTION_B, 0);
  WriteGIO(FA_IIC_ADDRESS, GIO_OUTPUT_A, 0);
  WriteGIO(FA_IIC_ADDRESS, GIO_OUTPUT_B, 0);
}

void TuneFA(int value)
{
  // Additional binary operations to enable the control of directly connected FAP.
  value |= (value & 0x200) >> 1;
  WriteGIO(FA_IIC_ADDRESS, GIO_OUTPUT_A, lowByte(value));
  WriteGIO(FA_IIC_ADDRESS, GIO_OUTPUT_B, highByte(value));
}

/****************************************
 * FA math routines
 ****************************************/

extern "C" struct Band
{
  long lowFrequency;    // Hz
  long highFrequency;   // Hz
  double inductance;    // nH
  double capacitance;   // pF
};

#define L1   14500  // nH
#define L2   14000  // nH
#define L3    4000  // nH
#define L4    3800  // nH

#define LB1  36300  // (L1 + L2 + L3 + L4)
#define LB2   9072  // (1 / (1 / (L1 + L3) + 1 / (L2 + L4)))
#define LB3   1949  // (1 / (1 / L3 + 1 / L4))

Band bands[] =
{
  { 1500000,  2999999, LB1, 61 },
  { 3000000,  6499999, LB2, 57 },
  { 6500000, 15999999, LB3, 29 }
};

#define FA_BANDS_COUNT       (sizeof(bands) / sizeof(Band))

#define FA_LOWER_BAND        0
#define FA_HIGHER_BAND       (FA_BANDS_COUNT - 1)

int CalculateTuning(long frequency)
{
  // Thompson formula for resonace frequency:
  //   f = 1 / (2 * Pi * sqrt(L * C))
  for (int index = 0; index < FA_BANDS_COUNT; index ++)
  {
    if ((frequency >= bands[index].lowFrequency) && (frequency <= bands[index].highFrequency))
    {
      double F = (double)frequency / 1000000;  // MHz
      double L = bands[index].inductance;      // nH
      double C = 10 * sq(5000 / (PI * F)) / L; // pF
      int control = int(C - bands[index].capacitance);
      return (index << 8) | constrain(control, 0, 255);
    }
  }
  return -1; 
}

long CalculateFrequency(int tuning)
{
  if ((tuning >= FA_LOWER_BOUND) && (tuning <= FA_UPPER_BOUND) && (highByte(tuning) < FA_BANDS_COUNT))
  {
    int index = highByte(tuning);
    int control = lowByte(tuning);
    double L = bands[index].inductance;             // nH
    double C = bands[index].capacitance + control;  // pF
    double F = 5000 / (PI * sqrt(L * C / 10));      // MHz
    return F * 1000000;
  }
  return 0;
}

double CalculateInductance(long frequency1, long frequency2, int capacitance1, int capacitance2)
{
  double F1 = (double)frequency1 / 1000000;  // MHz
  double F2 = (double)frequency2 / 1000000;  // MHz
  double Cd = capacitance2 - capacitance1;   // pF
  double L = 25000 * (sq(F1) - sq(F2)) / (sq(PI) * Cd * sq(F1 * F2 / 100)); // nH
  return L;
}

double CalculateParasiteCapacitance(long frequency1, long frequency2, int capacitance1, int capacitance2)
{
  double F1 = (double)frequency1 / 1000000;  // MHz
  double F2 = (double)frequency2 / 1000000;  // MHz
  double Cd = capacitance2 - capacitance1;   // pF
  double Cp = sq(F2) * Cd / (sq(F1) - sq(F2)) - (double)capacitance1; // pF
  return Cp;
}

/****************************************
 * Business logic
 ****************************************/

extern "C" struct Point
{
  long frequency;
  int tuning;
};

int tuning = 0;
long frequency = 0;
Point points[2] = { { frequency: 0, tuning: 0 }, { frequency: 0, tuning: 0 } };

#define SYNC_STEP_SIZE  1000

#define SYNC_NONE       0x00
#define SYNC_FREQUENCY  0x01
#define SYNC_TUNING     0x02

#define LEARNING_POINT_A  0
#define LEARNING_POINT_B  1

#define MEMORY_ANTENNA_1  0
#define MEMORY_ANTENNA_2  1


int Synchronize()
{
  int state = SYNC_NONE;
  long arrival = ReadFrequency();
  if (abs(frequency - arrival) > SYNC_STEP_SIZE)
  {
    frequency = arrival;
    int candidate = CalculateTuning(frequency);
    if (candidate >= 0)
    {
      state |= SYNC_FREQUENCY;
      if (candidate != tuning)
      {
        state |= SYNC_TUNING;
        tuning = candidate;
        TuneFA(tuning | FA_POWER_ON);
      }
    }
  }
  return state;
}

boolean Adjust(int summand)
{
  int candidate = tuning + summand;
  if ((candidate >= FA_LOWER_BOUND) && (candidate <= FA_UPPER_BOUND))
  {
    tuning = candidate;
    frequency = CalculateFrequency(tuning);
    TuneFA(tuning | FA_POWER_ON);
    return true;
  }
  return false;
}

void TurnOn()
{
  TuneFA(tuning | FA_POWER_ON);
}

void TurnOff()
{
  TuneFA(FA_POWER_OFF);
}

void Recall(int memory)
{
  int address = memory * sizeof(Band) * FA_BANDS_COUNT;
  ReadMemory(address, bands, sizeof(Band) * FA_BANDS_COUNT);
  TuneFA(tuning | FA_POWER_ON);
}

boolean Store(int memory)
{
  int tuning1 = points[LEARNING_POINT_A].tuning;
  int tuning2 = points[LEARNING_POINT_B].tuning;
  long frequency1 = points[LEARNING_POINT_A].frequency;
  long frequency2 = points[LEARNING_POINT_B].frequency;
  int band1 = tuning1 >> 8;
  int band2 = tuning2 >> 8;
  
  if (((tuning1 & 0x300) == (tuning2 & 0x300)) &&
      ((frequency1 >= bands[band1].lowFrequency) && (frequency1 <= bands[band1].highFrequency)) &&
      ((frequency2 >= bands[band2].lowFrequency) && (frequency2 <= bands[band2].highFrequency)))
  {
    Band* preset = &bands[band1];
    int capacitance1 = tuning1 & 255;
    int capacitance2 = tuning2 & 255;
    preset->inductance = CalculateInductance(frequency1, frequency2, capacitance1, capacitance2);
    preset->capacitance = CalculateParasiteCapacitance(frequency1, frequency2, capacitance1, capacitance2);

    int address = sizeof(Band) * (band1 + memory * FA_BANDS_COUNT);
    WriteMemory(address, preset, sizeof(Band));

    TuneFA(tuning | FA_POWER_ON);
    return true;
  }
  return false;
}

/****************************************
 * Buttons
 ****************************************/

#define BUTTON_NONE    0x000

#define BUTTON_RIGHT   0x001
#define BUTTON_UP      0x002
#define BUTTON_DOWN    0x004
#define BUTTON_LEFT    0x008
#define BUTTON_SELECT  0x010

#define BUTTON_S1      0x020
#define BUTTON_S2      0x040
#define BUTTON_S3      0x080
#define BUTTON_S4      0x100
#define BUTTON_S5      0x200

/****************************************
 * DFRobot LCD Shield, ADKeyboard Module
 ****************************************/

const int buttons[] = { 50, 195, 380, 555, 790 };

#define BUTTON_COUNT     (sizeof(buttons) / sizeof(int))

#define KEYPAD_PIN       0
#define KEYBOARD_PIN     14

#define KEYPAD_MARK      BUTTON_RIGHT
#define KEYBOARD_MARK    BUTTON_S1

int GetButtons(int pin, int mark)
{
  int value = analogRead(pin);

  if (value < 1000)
    for (int index = 0; index < BUTTON_COUNT; index ++)
      if (value < buttons[index])
        return (mark << index);

  return BUTTON_NONE;
}

/****************************************
 * Arduino JoyStick Module
 ****************************************/

#define JOYSTICK_X_PIN  13
#define JOYSTICK_Y_PIN  12
#define JOYSTICK_Z_PIN  40

int GetJoystick()
{
  int state = BUTTON_NONE;

  int X = analogRead(JOYSTICK_X_PIN);
  int Y = analogRead(JOYSTICK_Y_PIN);
  int Z = digitalRead(JOYSTICK_Z_PIN);
  
  if (Z == LOW)
    state |= BUTTON_SELECT;
  if (X < 420)
    state |= BUTTON_LEFT;
  if (X > 570)
    state |= BUTTON_RIGHT;
  if (Y < 510)
    state |= BUTTON_DOWN;
  if (Y > 580)
    state |= BUTTON_UP;

  return state;
}

/****************************************
 * Input routinues
 ****************************************/

#define INPUT_METHOD_NONE      0
#define INPUT_METHOD_KEYPAD    1
#define INPUT_METHOD_KEYBOARD  2
#define INPUT_METHOD_JOYSTICK  4

#define KEY_MASK               (BUTTON_RIGHT | BUTTON_UP | BUTTON_DOWN | BUTTON_LEFT | BUTTON_SELECT)
#define KEY_SHORT_PRESS        0x1000
#define KEY_LONG_PRESS         0x3000

#define SHORT_PRESS_TIMEOUT    20
#define LONG_PRESS_TIMEOUT     1500

unsigned long stamp = 0;
int previous = BUTTON_NONE;
int methods = INPUT_METHOD_NONE;

void DetectKeys()
{
  if (GetButtons(KEYPAD_PIN, KEYPAD_MARK) == BUTTON_NONE)
    methods |= INPUT_METHOD_KEYPAD;
  if (GetButtons(KEYBOARD_PIN, KEYBOARD_MARK) == BUTTON_NONE)
    methods |= INPUT_METHOD_KEYBOARD;
  if (GetJoystick() == BUTTON_NONE)
    methods |= INPUT_METHOD_JOYSTICK;
}

int ReadKeys()
{
  int state = BUTTON_NONE;
  unsigned long time = millis();

  if (methods & INPUT_METHOD_KEYPAD)
    state |= GetButtons(KEYPAD_PIN, KEYPAD_MARK);
  if (methods & INPUT_METHOD_KEYBOARD)
    state |= GetButtons(KEYBOARD_PIN, KEYBOARD_MARK);
  if (methods & INPUT_METHOD_JOYSTICK)
    state |= GetJoystick();  
  if (state != (previous & KEY_MASK))
    stamp = time;
  if (time > (stamp + SHORT_PRESS_TIMEOUT))
    state |= KEY_SHORT_PRESS;
  if (time > (stamp + LONG_PRESS_TIMEOUT))
    state |= KEY_LONG_PRESS;
  if (state == previous)
    return BUTTON_NONE;

  previous = state;
  return state;
}

/****************************************
 * User interface
 ****************************************/

#define LIGHTING_TIMEOUT    5000

#define BACKLIGHT_PIN       10
#define LED_PIN             13

#define MODE_OFF            0x00
#define MODE_AUTOMATIC      0x01
#define MODE_MANUAL         0x02
#define MODE_MASK           0x03

#define SELECTION_BAND      0
#define SELECTION_COARSE    1
#define SELECTION_FINAL     2

#define SELECT_FORWARD      1
#define SELECT_BACKWARD     -1

#define STATE_POINT_A       (0x01 << LEARNING_POINT_A)
#define STATE_POINT_B       (0x01 << LEARNING_POINT_B)
#define STATE_ANTENNA_1     (0x04 << MEMORY_ANTENNA_1)
#define STATE_ANTENNA_2     (0x04 << MEMORY_ANTENNA_2)
#define STATE_STORING       0x10

#define STATE_POINT_ALL     (STATE_POINT_A | STATE_POINT_B)
#define STATE_POINT_MASK    (STATE_POINT_A | STATE_POINT_B | STATE_STORING)
#define STATE_ANTENNA_MASK  (STATE_ANTENNA_1 | STATE_ANTENNA_2 | STATE_STORING)

const int ordinals[] = { 0x100, 0x10, 0x01 };

#define AREA_NONE           SYNC_NONE
#define AREA_FREQUENCY      SYNC_FREQUENCY
#define AREA_TUNING         SYNC_TUNING
#define AREA_MODE           0x10
#define AREA_STATE          0x20
#define AREA_SELECTION      AREA_TUNING
#define AREA_ALL            (AREA_FREQUENCY | AREA_TUNING | AREA_MODE | AREA_STATE)

LiquidCrystal display(8, 9, 4, 5, 6, 7);

unsigned long lighting = 0;
int activity = LOW;

int mode = MODE_AUTOMATIC;
int selection = SELECTION_BAND;
int state = STATE_ANTENNA_1;

void UpdateDisplay(int area)
{
  char buffer[16];
  if (area & AREA_FREQUENCY)
  {
    sprintf(buffer, "%6d KHz", int(frequency / 1000));
    display.setCursor(0, 0);
    display.print(buffer);
  }
  if (area & AREA_MODE)
  {
    static const char* modes[] = { "PO", "AT", "MA" };
    display.setCursor(1, 1);
    display.print(modes[mode & MODE_MASK]);
  }
  if (area & AREA_TUNING)
  {
    int band = highByte(tuning) + 1;
    int coarse = lowByte(tuning) >> 4;
    int final = tuning & 0x0f;
    sprintf(buffer, " %2d  %2d  %2d ", band, coarse, final);
    if ((selection >= SELECTION_BAND) && (selection <= SELECTION_FINAL))
    {
      buffer[selection * 4] = '[';
      buffer[selection * 4 + 3] = ']';
    }
    display.setCursor(4, 1);
    display.print(buffer);
  }
  if (area & AREA_STATE)
  {
    display.setCursor(14, 0);
    if (state == STATE_ANTENNA_1)
      display.print("M1");
    if (state == STATE_ANTENNA_2)
      display.print("M2");
    if ((state & STATE_POINT_MASK) == STATE_POINT_A)
      display.print("A ");
    if ((state & STATE_POINT_MASK) == STATE_POINT_B)
      display.print(" B");
    if ((state & STATE_POINT_MASK) == STATE_POINT_ALL)
      display.print("AB");
    if ((state & STATE_ANTENNA_MASK) == (STATE_STORING | STATE_ANTENNA_1))
      display.print("S1");
    if ((state & STATE_ANTENNA_MASK) == (STATE_STORING | STATE_ANTENNA_2))
      display.print("S2");
  }
  if (area != AREA_NONE)
  {
    lighting = millis() + LIGHTING_TIMEOUT;
    digitalWrite(BACKLIGHT_PIN, HIGH);
  }
}

boolean Select(int summand)
{
  int candidate = selection + summand;
  if ((candidate >= SELECTION_BAND) && (candidate <= SELECTION_FINAL))
  {
    selection = candidate;
    return true;
  }
  return false;
}

void UpdateIndicators()
{
  if ((lighting > 0) && (lighting < millis()))
  {
    lighting = 0;
    digitalWrite(BACKLIGHT_PIN, LOW);
  }
  if (activity != InProgress())
  {
    activity ^= HIGH;
    digitalWrite(LED_PIN, activity);
  }
}

/****************************************
 * Main
 ****************************************/

void setup()
{
  // Initialize debug console
  Serial.begin(9600);
  // Initialize display
  pinMode(BACKLIGHT_PIN, OUTPUT);
  display.begin(16, 2);
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  // Initialize joystick
  pinMode(JOYSTICK_Z_PIN, INPUT);
  // Initialize CI-V port
  CIV.begin(CIV_BAUD_RATE);
  // Initialize FA port and antenna settings
  InitializeFA();
  // Recall(MEMORY_ANTENNA_1);
  // ..
  DetectKeys();
  UpdateDisplay(AREA_ALL);
}

void loop()
{
  if ((ReadCIV()) && (ReadCommand() == COMMAND_FREQUENCY_UPDATED) && (mode == MODE_AUTOMATIC))
    UpdateDisplay(Synchronize());
  
  switch (ReadKeys())
  {
    case BUTTON_SELECT | KEY_SHORT_PRESS:
      mode ^= MODE_MASK;
      if (mode == MODE_MASK)
        mode = MODE_AUTOMATIC;
      if (mode == MODE_AUTOMATIC)
        Synchronize();
      UpdateDisplay(AREA_ALL);
      break;
    case BUTTON_SELECT | KEY_LONG_PRESS:
      mode = MODE_OFF;
      TurnOff();
      UpdateDisplay(AREA_MODE);
      break;
    case BUTTON_UP | KEY_SHORT_PRESS:
      mode = MODE_MANUAL;
      if (Adjust(ordinals[selection]))
        UpdateDisplay(AREA_ALL);
      break;
    case BUTTON_DOWN | KEY_SHORT_PRESS:
      mode = MODE_MANUAL;
      if (Adjust(-ordinals[selection]))
        UpdateDisplay(AREA_ALL);
      break;
    case BUTTON_RIGHT | KEY_SHORT_PRESS:
      if (Select(SELECT_FORWARD))
        UpdateDisplay(AREA_SELECTION);
      break;
    case BUTTON_LEFT | KEY_SHORT_PRESS:
      if (Select(SELECT_BACKWARD))
        UpdateDisplay(AREA_SELECTION);
      break;
    case BUTTON_S3 | KEY_LONG_PRESS:
      state = STATE_ANTENNA_1;
      Recall(MEMORY_ANTENNA_1);
      UpdateDisplay(AREA_STATE);
      break;
    case BUTTON_S4 | KEY_LONG_PRESS:
      state = STATE_ANTENNA_2;
      Recall(MEMORY_ANTENNA_2);
      UpdateDisplay(AREA_STATE);
      break;
    case BUTTON_S1 | KEY_SHORT_PRESS:
      if ((state & STATE_POINT_ALL) == STATE_POINT_ALL)
      {
        state |= STATE_STORING;
        UpdateDisplay(AREA_STATE);
      }
      break;
    case BUTTON_S3 | KEY_SHORT_PRESS:
      if (state & STATE_STORING)
      {
        state &= STATE_ANTENNA_MASK;
        state |= STATE_ANTENNA_1 | STATE_STORING;
        UpdateDisplay(AREA_STATE);
      }
      break;
    case BUTTON_S4 | KEY_SHORT_PRESS:
      if (state & STATE_STORING)
      {
        state &= STATE_ANTENNA_MASK;
        state |= STATE_ANTENNA_2 | STATE_STORING;
        UpdateDisplay(AREA_STATE);
      }
      break;
    case BUTTON_S2 | KEY_LONG_PRESS:
      // A
      break;
    case BUTTON_S5 | KEY_LONG_PRESS:
      // B
      break;
    case BUTTON_S1 | KEY_LONG_PRESS:
      state &= STATE_ANTENNA_1 | STATE_ANTENNA_2;
      if (state & STATE_ANTENNA_1)
        Store(MEMORY_ANTENNA_1);
      if (state & STATE_ANTENNA_2)
        Store(MEMORY_ANTENNA_2);
      UpdateDisplay(AREA_STATE);
      break;
  }
  
  UpdateIndicators();
}
