#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------------
// LCD + Keypad Settings
// ---------------------
#define PCF_ADDR 0x20
LiquidCrystal_I2C lcd(0x27, 20, 4);

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

// ---------------------
// Buzzer Pin
// ---------------------
#define BUZZER_PIN 42

// Bella Chao notes and timing variables
int Si2 = 1975;
int LaS2 = 1864;
int La2 = 1760;
int SolS2 = 1661;
int Sol2 = 1567;
int FaS2 = 1479;
int Fa2 = 1396;
int Mi2 = 1318;
int ReS2 = 1244;
int Re2 = 1174;
int DoS2 = 1108;
int Do2 = 1046;

// Low Octave
int Si = 987;
int LaS = 932;
int La = 880;
int SolS = 830;
int Sol = 783;
int FaS = 739;
int Fa = 698;
int Mi = 659;
int ReS = 622;
int Re = 587;
int DoS = 554;
int Do = 523;

int rounda = 0;
int roundp = 0;
int white = 0;
int whitep = 0;
int black = 0;
int blackp = 0;
int quaver = 0;
int quaverp = 0;
int semiquaver = 0;
int semiquaverp = 0;
int bpm = 120;
#define NOTE_REST 0

// ---------------------
// NOTE DEFINITIONS
// ---------------------
#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0

// ---------------------
// Happy Birthday Song
// ---------------------
int melodyHB[] = {
  NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_F4, NOTE_E4,
  NOTE_C4, NOTE_C4, NOTE_D4, NOTE_C4, NOTE_G4, NOTE_F4,
  NOTE_C4, NOTE_C4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_E4, NOTE_D4,
  NOTE_AS4, NOTE_AS4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_F4
};
int noteDurationsHB[] = {
  4, 8, 4, 4, 4, 2,
  4, 8, 4, 4, 4, 2,
  4, 8, 4, 4, 4, 4, 2,
  4, 8, 4, 4, 4, 2
};

// ---------------------
// Harry Potter - Hedwig's Theme
// ---------------------
int tempoHP = 144;
int melodyHP[] = {
  REST,
  2,
  NOTE_D4,
  4,
  NOTE_G4,
  -4,
  NOTE_AS4,
  8,
  NOTE_A4,
  4,
  NOTE_G4,
  2,
  NOTE_D5,
  4,
  NOTE_C5,
  -2,
  NOTE_A4,
  -2,
  NOTE_G4,
  -4,
  NOTE_AS4,
  8,
  NOTE_A4,
  4,
  NOTE_F4,
  2,
  NOTE_GS4,
  4,
  NOTE_D4,
  -1,
  NOTE_D4,
  4,
  NOTE_G4,
  -4,
  NOTE_AS4,
  8,
  NOTE_A4,
  4,
  NOTE_G4,
  2,
  NOTE_D5,
  4,
  NOTE_F5,
  2,
  NOTE_E5,
  4,
  NOTE_DS5,
  2,
  NOTE_B4,
  4,
  NOTE_DS5,
  -4,
  NOTE_D5,
  8,
  NOTE_CS5,
  4,
  NOTE_CS4,
  2,
  NOTE_B4,
  4,
  NOTE_G4,
  -1,
  NOTE_AS4,
  4,
  NOTE_D5,
  2,
  NOTE_AS4,
  4,
  NOTE_D5,
  2,
  NOTE_AS4,
  4,
  NOTE_DS5,
  2,
  NOTE_D5,
  4,
  NOTE_CS5,
  2,
  NOTE_A4,
  4,
  NOTE_AS4,
  -4,
  NOTE_D5,
  8,
  NOTE_CS5,
  4,
  NOTE_CS4,
  2,
  NOTE_D4,
  4,
  NOTE_D5,
  -1,
  REST,
  4,
  NOTE_AS4,
  4,
  NOTE_D5,
  2,
  NOTE_AS4,
  4,
  NOTE_D5,
  2,
  NOTE_AS4,
  4,
  NOTE_F5,
  2,
  NOTE_E5,
  4,
  NOTE_DS5,
  2,
  NOTE_B4,
  4,
  NOTE_DS5,
  -4,
  NOTE_D5,
  8,
  NOTE_CS5,
  4,
  NOTE_CS4,
  2,
  NOTE_AS4,
  4,
  NOTE_G4,
  -1,
};
int notesHP = sizeof(melodyHP) / sizeof(melodyHP[0]) / 2;
int wholenoteHP = (60000 * 4) / tempoHP;

int melodyJB[] = {
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_E4,
  NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_G4,

  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4,
  NOTE_E4, NOTE_G4, NOTE_C4, NOTE_D4, NOTE_E4,
  NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4,
  NOTE_G4, NOTE_G4, NOTE_F4, NOTE_D4, NOTE_C4
};

int noteDurationsJB[] = {
  4, 4, 2, 4, 4, 2,
  4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 8,
  8, 4, 4, 4, 4, 2, 2,

  4, 4, 2, 4, 4, 2,
  4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 8,
  4, 4, 4, 4, 2
};

int totalNotesJB = sizeof(melodyJB) / sizeof(melodyJB[0]);

// ---------------------
// Globals
// ---------------------
volatile bool stopMusic = false;

// ---------------------
// Setup
// ---------------------
void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  pinMode(BUZZER_PIN, OUTPUT);
  black = 35000 / bpm;
  blackp = black * 1.5;
  white = black * 2;
  whitep = white * 1.5;
  rounda = black * 4;
  roundp = rounda * 1.5;
  quaver = black / 2;
  quaverp = quaver * 1.5;
  semiquaver = black / 4;
  semiquaverp = semiquaver * 1.5;


  lcd.setCursor(0, 0);
  lcd.print("Interfacing with");
  lcd.setCursor(0, 1);
  lcd.print("Buzzer");
  delay(2000);

  showMenu();
}

// ---------------------
// Main Loop
// ---------------------
void loop() {
  char key = scanKeypad();
  if (key == '1') {
    stopMusic = false;
    playHappyBirthday();
    showMenu();
  } else if (key == '2') {
    stopMusic = false;
    playHarryPotter();
    showMenu();
  } else if (key == '3') {
    stopMusic = false;
    playBellaChao();
    showMenu();
  } else if (key == '4') {
    stopMusic = false;
    playJingleBells();
    showMenu();
  } else if (key == 'A') {
    stopMusic = true;
    noTone(BUZZER_PIN);
  }
}

void playJingleBells() {
  for (int noteIndex = 0; noteIndex < totalNotesJB; noteIndex++) {

    // Calculate timing for the note and gap
    int baseNoteDuration = 1000 / noteDurationsJB[noteIndex];
    float notePlayFactor = 0.9;  // play 90% of time
    float gapFactor = 0.1;       // rest 10% of time

    unsigned long playTime = baseNoteDuration * notePlayFactor;
    unsigned long gapTime = baseNoteDuration * gapFactor;

    // Start playing the note
    if (melodyJB[noteIndex] != NOTE_REST) {
      tone(BUZZER_PIN, melodyJB[noteIndex]);
    } else {
      noTone(BUZZER_PIN);  // rest
    }

    // Play for playTime, checking for stop
    unsigned long noteStartTime = millis();
    while (millis() - noteStartTime < playTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        noTone(BUZZER_PIN);
        return;
      }
    }

    // Stop the note to create the gap
    noTone(BUZZER_PIN);
    unsigned long gapStartTime = millis();
    while (millis() - gapStartTime < gapTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        return;
      }
    }
  }
}



void playBellaChao() {
  struct Note {
    int freq;
    float mult;
  };
  Note notes[] = {
    { Mi, 1 }, { La, 1 }, { Si, 1 }, { Do2, 1 }, { La, 1 }, { -1, 2 },  // pause
    { Mi, 1 },
    { La, 1 },
    { Si, 1 },
    { Do2, 1 },
    { La, 1 },
    { -1, 2 },
    { Mi, 1 },
    { La, 1 },
    { Si, 1 },
    { Do2, 1.3 },
    { -1, 2 },
    { Si, 1 },
    { La, 1 },
    { Do2, 1.3 },
    { -1, 2 },
    { Si, 1 },
    { La, 1 },
    { Mi2, 1 },
    { -1, white },
    { Mi2, 1 },
    { -1, white },
    { Mi2, 1 },
    { Re2, 1 },
    { Mi2, 1 },
    { Fa2, 1 },
    { Fa2, 1.3 },
    { -1, rounda },
    { Fa2, 1 },
    { Mi2, 1 },
    { Re2, 1 },
    { Fa2, 1 },
    { Mi2, 1.3 },
    { -1, rounda },
    { Mi2, 1 },
    { Re2, 1 },
    { Do2, 1 },
    { Si, 1.3 },
    { Mi2, 1.3 },
    { Si, 1.3 },
    { Do2, 1.3 },
    { La, rounda * 1.3 }
  };

  for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    // Calculate full note duration
    unsigned long fullDuration = (unsigned long)(black * notes[i].mult) + 50;

    // Play only 90% of the time, leave 10% gap
    unsigned long playTime = fullDuration * 0.9;
    unsigned long gapTime = fullDuration - playTime;

    // Start playing note (if it's not a rest)
    if (notes[i].freq > 0) tone(BUZZER_PIN, notes[i].freq);
    else noTone(BUZZER_PIN);

    // Play phase
    unsigned long startTime = millis();
    while (millis() - startTime < playTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        noTone(BUZZER_PIN);
        return;
      }
    }

    // Gap phase (silence)
    noTone(BUZZER_PIN);
    unsigned long gapStart = millis();
    while (millis() - gapStart < gapTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        return;
      }
    }
  }
}



// ---------------------
// Menu Display
// ---------------------
void showMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1: Happy Birthday");
  lcd.setCursor(0, 1);
  lcd.print("2: Harry Potter");
  lcd.setCursor(0, 2);
  lcd.print("3: Bella Chao");
  lcd.setCursor(0, 3);
  lcd.print("4: Jingle Bells");
}

// ---------------------
// Music Functions
// ---------------------
void playHappyBirthday() {
  int size = sizeof(melodyHB) / sizeof(int);

  for (int noteIndex = 0; noteIndex < size; noteIndex++) {
    // Calculate note timing
    int baseNoteDuration = 1000 / noteDurationsHB[noteIndex];
    float playRatio = 1.2;  // 90% play
    float gapRatio = 0.1;   // 10% silence

    unsigned long playTime = baseNoteDuration * playRatio;
    unsigned long gapTime = baseNoteDuration * gapRatio;

    // Start playing note (or rest)
    if (melodyHB[noteIndex] != NOTE_REST) {
      tone(BUZZER_PIN, melodyHB[noteIndex]);
    } else {
      noTone(BUZZER_PIN);
    }

    // Play phase
    unsigned long startTime = millis();
    while (millis() - startTime < playTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        noTone(BUZZER_PIN);
        return;
      }
    }

    // Gap phase
    noTone(BUZZER_PIN);
    unsigned long gapStart = millis();
    while (millis() - gapStart < gapTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        return;
      }
    }
  }
}


void playHarryPotter() {
  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notesHP * 2; thisNote += 2) {
    int note = melodyHP[thisNote];
    int duration = melodyHP[thisNote + 1];

    // Calculate note duration
    divider = duration;
    if (divider > 0) {
      noteDuration = wholenoteHP / divider;
    } else if (divider < 0) {
      noteDuration = (wholenoteHP / abs(divider)) * 1.5;
    }

    // Split into play & gap times
    float playRatio = 0.9;  // 90% play
    unsigned long playTime = noteDuration * playRatio;
    unsigned long gapTime = noteDuration - playTime;

    // Play the note (or rest)
    if (note == REST) {
      noTone(BUZZER_PIN);
    } else {
      tone(BUZZER_PIN, note);
    }

    // Play phase
    unsigned long startTime = millis();
    while (millis() - startTime < playTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        noTone(BUZZER_PIN);
        return;
      }
    }

    // Gap phase (silence)
    noTone(BUZZER_PIN);
    unsigned long gapStart = millis();
    while (millis() - gapStart < gapTime) {
      char key = scanKeypad();
      if (key == 'A') {
        stopMusic = true;
        return;
      }
    }
  }
}

// ---------------------
// Keypad Functions
// ---------------------
char scanKeypad() {
  for (int row = 0; row < 4; row++) {
    byte out = 0xFF;
    out &= ~(1 << (row + 4));
    writePCF(out);
    delay(2);

    byte in = readPCF();
    for (int col = 0; col < 4; col++) {
      if ((in & (1 << col)) == 0) {
        return keys[row][col];
      }
    }
  }
  return 0;
}

void writePCF(byte val) {
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(val);
  Wire.endTransmission();
}

byte readPCF() {
  Wire.requestFrom(PCF_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}