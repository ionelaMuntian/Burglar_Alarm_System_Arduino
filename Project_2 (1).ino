#define ZONE1 2 // PIN2 - has interrupt
#define ZONE2 3 // PIN3 - has interrupt
#define ZONE3 4 
#define ZONE4 5 
#define BUZZER 7
#define POTENTIOMETER A0 // Analog pin for threshold checks
#define READ_PORT(val) val = PIND


enum zone_types {EXIT_ENTRY, DIGITAL_ALARM, ANALOG_ALARM, CONT_MONIT, UNUSED};

//There are 5 zone types
struct Zone {
  byte pin;
  zone_types type;
  int threshold;
  bool active;
};

// Constants
#define NUM_ZONES 4 
#define PASSWORD_LEN 6

// Global variables
Zone zones[NUM_ZONES];
volatile bool alarmArmed = false;
volatile bool alarmTriggered = false;
volatile bool isTimerOn = false;
volatile bool timerExpired = false;
volatile unsigned long timerSeconds = 0;

// Variables related to handling the passwords 
char setPasswordEntry[PASSWORD_LEN] = "1234";
char setPasswordExit[PASSWORD_LEN] = "5678";
char enteredPassword[PASSWORD_LEN];
volatile bool entryMode = false;
volatile bool exitMode = false;
bool lastAlarmStatusPrinted = false; 

//Init. the 4 zones
void initializeZones() {
  zones[0] = {ZONE1, UNUSED, 0, false};
  zones[1] = {ZONE2, UNUSED, 0, false};
  zones[2] = {ZONE3, UNUSED, 500, false}; // Example analog threshold
  zones[3] = {ZONE4, UNUSED, 0, false};
}

//Timer
void setupTimer(unsigned int seconds) {
  isTimerOn = true;
  cli(); 
  TCCR1A = 0; 
  TCCR1B = 0; 

  // The system clock on an Arduino is 16 MHz, so to count seconds, we need to set the timer to count at a frequency that is one count per second.
  // CS12 selects a prescaler of 256

  OCR1A = (16000000 / 256) - 1; 
  TCCR1B |= (1 << WGM12) | (1 << CS12); 
  TIMSK1 |= (1 << OCIE1A); 
  timerSeconds = seconds; 
  sei(); 
}

void stopTimer() {
  isTimerOn = false;
  cli(); 
  TCCR1B = 0; 
  TIMSK1 &= ~(1 << OCIE1A); 
  sei(); 
}

ISR(TIMER1_COMPA_vect) {
  static unsigned long elapsed = 0;
  elapsed++;
  if (elapsed >= timerSeconds) {
    timerExpired = true;
    elapsed = 0;
    stopTimer();
    Serial.println("Timer expired! Alarm triggered.");
    alarmTriggered = true;
    PORTD |= (1 << BUZZER); // Activate buzzer
  }
}

// Function that checks if the password that I enter matches the existing password. 
bool handlePassword(const char actualPassword[]) {
  if (Serial.available() > 0) {
    int len = Serial.readBytesUntil('\n', enteredPassword, PASSWORD_LEN - 1);
    enteredPassword[len] = '\0'; // Null-terminate the string

    if (strcmp(enteredPassword, actualPassword) == 0) {
      return true;  // Password correct
    } else {
      Serial.println("Incorrect password.");
      return false;  // Password incorrect
    }
  }
  return false; // No password entered
}

//Configure the types of the zones based on the user choices
void configureZones() {
  for (int i = 0; i < NUM_ZONES; i++) {
    Serial.print("Configure ZONE ");
    Serial.print(i + 1);
    Serial.println(":");
    
    if (i == 0 || i == 1) {
      Serial.print("\n"); 
      Serial.println("1: Digital Alarm");
      Serial.println("3: Continuous Monitoring");
    } else { 
      Serial.print("\n"); 
      Serial.println("0: Entry/Exit");
      Serial.println("2: Analog Alarm");
      Serial.println("4: Unused");
    }
    
    // Wait for valid input
    while (true) {
      if (Serial.available() > 0) {
        char choice = Serial.read();  

        Serial.print("\nReceived choice: ");
        Serial.println(choice);
       
       // We have added this condition because it was accidently reading the spaces or newline before we had time to write the input
        if (choice == '\n' || choice == '\r') {
          continue; 
        }

        // Clear the rest of the serial buffer if needed
        while (Serial.available() > 0) {
          Serial.read(); // Discard any other bytes in the buffer
        }

        // Validate input based on zone number 
        if ((i < 2 && (choice == '1' || choice == '3')) || 
            (i >= 2 && (choice == '0' || choice == '2' || choice == '4'))) {
          zones[i].type = static_cast<zone_types>(choice - '0');
          Serial.print("Zone ");
          Serial.print(i + 1);
          Serial.print(" set to type ");
          Serial.println(zones[i].type);
          break;  // Exit loop once valid input is received
        } else {
          Serial.println("Invalid choice. Please try again.");
        }
      }
    }
  }
}

// Function pointer array definition
void (*zoneActions[])() = {
  exit_entry_func,
  digital_alarm_func,
  analog_alarm_func,
  continuous_monit,
  unused_zone_func
};

//Based on which button (which corresponds to a zone) is pressed, call the appropriate zone action
void handleZonePress(int zoneIndex) {
  if (zones[zoneIndex].type != UNUSED) {
    zoneActions[zones[zoneIndex].type]();  
  }
}

//This function arms the alarm (when the user wants to exit)
void armAlarm() {
  alarmArmed = true;
  lastAlarmStatusPrinted = false;
  Serial.println("Alarm armed.");
}

//This function arms the alarm
void disarmAlarm() {
  alarmArmed = false;
  alarmTriggered = false;
  digitalWrite(BUZZER, LOW); 
  lastAlarmStatusPrinted = false; 
  Serial.println("Alarm disarmed.");
}

// Function that handles the action that must happen in entry OR exit zone
void exit_entry_func() {

  // If we are in entry mode
  if (alarmArmed) {
    Serial.println("Entry detected. Enter the entry password to disarm the alarm.");
    entryMode = true;
    exitMode = false;

    setupTimer(15);
    bool passwordCorrect = false;

    // Wait for the user to insert the correct password for 15 seconds
    while (isTimerOn) { 
      if (handlePassword(setPasswordEntry)) { // Check if password is correct
        passwordCorrect = true;
        stopTimer();
        break;
      }
    }

    if (passwordCorrect) {
      Serial.println("Correct password.");
      disarmAlarm();
    } else {
      Serial.println("Incorrect password OR timeout expired.");
      Serial.println("ALARM SOUND IS ON!");
      digitalWrite(BUZZER, HIGH); 
    }
  } else {
    if (!exitMode) {  
    Serial.println("Exit detected. Enter the exit password to arm the alarm.");
      exitMode = true;
      entryMode = false;

      setupTimer(15);
      bool passwordCorrect = false;

      while (isTimerOn) { 
        if (handlePassword(setPasswordExit)) { 
          passwordCorrect = true;
          stopTimer();
          break;
        }
      }

      // If the password was correct, arm the alarm
      if (passwordCorrect) {
        armAlarm();
      } else {
        Serial.println("Password not entered correctly or timeout expired.");
        Serial.println("Triggering alarm sound...");
        digitalWrite(BUZZER, HIGH); // Trigger buzzer sound
      }
    }
  }
}

void digital_alarm_func() {
  if (alarmArmed) {
    Serial.println("Digital alarm was triggered!");
    alarmTriggered = true;
  }
}

void analog_alarm_func() {
  int sensorValue = analogRead(POTENTIOMETER);
  if (alarmArmed && sensorValue > zones[2].threshold) {
    Serial.println("Analog alarm was triggered!");
    alarmTriggered = true;
  }
}

// Continous monitoring zone can start even if the alarm is not armed
void continuous_monit() {
  Serial.println("Continuous Monitoring Zone was triggered!");
  alarmTriggered = true;
}

void unused_zone_func() {
  Serial.println("Unused zone triggered. No action taken.");
}

// Interrupt handlers for Zones 1 and 2
void zone1ISR() { handleZonePress(0); }
void zone2ISR() { handleZonePress(1); }

void setup() {
  Serial.begin(9600);
  
  //  BUZZER is OFF initially
  // Configure buzzer pin
  DDRD |= (1 << BUZZER); 
  PORTD &= ~(1 << BUZZER); 

  DDRD &= ~((1 << ZONE1) | (1 << ZONE2) | (1 << ZONE3) | (1 << ZONE4)); 
  PORTD |= ((1 << ZONE1) | (1 << ZONE2) | (1 << ZONE3) | (1 << ZONE4)); 

  initializeZones();
  configureZones();

  // Attach interrupts for ZONE1 and ZONE2 (DIGITAL_ALARM and CONT_MONIT zones)
  attachInterrupt(digitalPinToInterrupt(ZONE1), zone1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ZONE2), zone2ISR, FALLING);
}

void loop() {

  //Zones 3 and 4 correspond to one zones that do not use interrupts
  uint8_t portState;
  READ_PORT(portState);  // Read the entire port

  if (portState & (1 << ZONE3)) {  // Check if ZONE3 is HIGH
    handleZonePress(2);
}

if (portState & (1 << ZONE4)) {  // Check if ZONE4 is HIGH
    handleZonePress(3);
}


  // Print the alarm status only once
  if (!lastAlarmStatusPrinted) {
    Serial.println(alarmArmed ? "Alarm Armed" : "Alarm Not Armed");
    lastAlarmStatusPrinted = true; // Flag that the message has been printed
  }

  // Activate the buzzer if the alarm is triggered
  if (alarmTriggered) {
    digitalWrite(BUZZER, HIGH); // Activate buzzer
  } else {
    digitalWrite(BUZZER, LOW); // Deactivate buzzer
  }

  delay(100); // Small delay for stability
}