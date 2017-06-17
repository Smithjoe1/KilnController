#include <Adafruit_MAX31855.h>

#include "U8glib.h"
#include "M2tk.h"
#include "utility/m2ghu8g.h"
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <TimerOne.h>

#define TESTING false
#define NumberOfProfileStages 10


const char * ver = "1.9";

// setup u8g object, please remove comment from one of the following constructor calls
// IMPORTANT NOTE: The complete list of supported devices is here: http://code.google.com/p/u8glib/wiki/device
#define FONT10 u8g_font_6x12
#define FONT12 u8g_font_6x13
#define FONT14 u8g_font_fur14
#define FONT20 u8g_font_fur20
#define FONT u8g_font_5x8
#define FONT3 u8g_font_04b_03r
#define FONT2 u8g_font_u8glib_4

//#define LCD_SCK  23 //RAMPS breakout
#define LCD_SCK  16 //Hand wired
#define LCD_MOSI  17
#define LCD_CS 24

//#define ROTARY_ENCODER_A 33
//#define ROTARY_ENCODER_B 31
//#define ROTARY_ENCODER_SELECT 35
//#define BUZZER 37

#define ROTARY_ENCODER_A 4
#define ROTARY_ENCODER_B 6
#define ROTARY_ENCODER_SELECT 2
#define BUZZER 27

#define RELAY_PIN 8
char buf[20]; // generic char buffer
byte cursorX;
byte cursorY;
char s[16]; //Short char buffer for LABELFN use
U8GLIB_ST7920_128X64_1X u8g(LCD_SCK, LCD_MOSI, LCD_CS);

M2_EXTERN_ALIGN(top_menu);
M2_EXTERN_ALIGN(el_profile_menu);
M2_EXTERN_ALIGN(el_preview_menu);

M2tk m2(&top_menu, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_u8g_bf);

bool running = false;
bool initialProcessDisplay = false;

const uint8_t h =  64;
const uint8_t graphHeight = 35;
const uint8_t w = 128;
const uint8_t yOffset =  30; // space not available for graph
uint16_t dx, dy;
uint8_t y = 2;
double tmp;
byte actualTempArray[w];
byte predictedTempArray[w];
unsigned long minuteCounter; 
unsigned long minuteCounterStart;  
int startTemp = 10;
int errorTimer = 0;

//========================================================== Timers
double lastTime;
double lastTime2;
double lastTime3;
double now;
double ns     = 1000 / 2; //How many times a second does the cycle run, 360fps currently.
double delta  = 0;
double ns2    = 1000 *5 ; //Tempearture read. 5 second interval otherwise returns 0 if read too often
double delta2 = 0;
double ns3    = 1000 / 1; //How many times a second does the cycle run, 1fps currently.
double delta3 = 0;

//=================================================
// Thermocouple via SPI

#define TC_DO   11
#define TC_CS   12
#define TC_CLK  13


Adafruit_MAX31855 thermocouple(TC_CLK, TC_CS, TC_DO);

float Temp = 10.0;
float T;

int topTempReached = 0;
double Setpoint;

// ----------------------------------------------------------------------------

#define WITH_SPLASH 1

// ----------------------------------------------------------------------------


int WindowSize = 255;
unsigned long windowTime;

bool displayUpdate = false;

// ----------------------------------------------------------------------------

// data type for the values used in the reflow profile
// Bitfields compress the data to 4 bytes per array point
// EEPROM space is precious with arrays
typedef struct profileValues_s {  
  uint16_t Temp : 12;
  uint16_t Duration : 10;
  uint16_t Mode:2; //This is a flag to be written when saving a profile, 0 = up, 1 = down, 2 = off
  uint8_t Speed;
  //  uint8_t checksum;
} Profile_t[NumberOfProfileStages];
const uint8_t maxProfiles = 25;

//Placeholder values for menu cancel possibility before writing
//Also assists wtih type conversion issues

uint32_t TempTMP[NumberOfProfileStages];
uint32_t DurationTMP[NumberOfProfileStages];
uint8_t SpeedTMP[NumberOfProfileStages];
uint8_t stageBeingModified;

uint32_t checksumTMP;
int maxProfileTemp[maxProfiles]; //Used to show the peak temp the profile will reach

Profile_t activeProfile; // the one and only instance
int activeProfileId = 0;
unsigned long startTime; //Millis when we started 
unsigned long minutesElapsed; //How long have we been running for

unsigned long ProfileReachedTime[NumberOfProfileStages];

uint8_t activeProfileCursor = 0;//A counter to hold the profile about to be loaded 
uint8_t activeProfileCursor1 = 0;//A counter to hold the profile about to be saved


const uint16_t isRunning   = maxProfiles * sizeof(Profile_t) + 1; // one byte
const uint16_t offsetProfileNum = maxProfiles * sizeof(Profile_t) + 2; // one byte
const uint16_t offsetPidConfig  = maxProfiles * sizeof(Profile_t) + 3; // sizeof(PID_t)

uint32_t lastUpdate = 0;
uint32_t lastDisplayUpdate = 0;
boolean spinner = false; //Container for display spinner to know kiln is working and hasnt crashed.
int highestTemp = 0; //Top temp our program wants to reach

// Current State
byte State = 0;
byte previousState = 0;
boolean isHolding = false;
int currentStage;
/*  None     = 0,
  Idle     = 1,
  Modes = 10 - 19
  Complete = 20,
  */

unsigned int   estimatedTotalTime;
//
// Copyright (c) 2002 Colin O'Flynn
// Minor changes by M.Thomas 9/2004 
// ----------------------------------------------------------------------------

#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0
uint8_t crc8(uint8_t *data, uint16_t data_length) {
  uint8_t  b;
  uint8_t  bit_counter;
  uint8_t  crc = CRC8INIT;
  uint8_t  feedback_bit;
  uint16_t loop_count;
  
  for (loop_count = 0; loop_count != data_length; loop_count++) {
    b = data[loop_count];
    bit_counter = 8;

    do {
      feedback_bit = (crc ^ b) & 0x01;

      if (feedback_bit == 0x01) {
        crc = crc ^ CRC8POLY;
      }

      crc = (crc >> 1) & 0x7F;

      if (feedback_bit == 0x01) {
        crc = crc | 0x80;
      }

      b = b >> 1;
      bit_counter--;

    } while (bit_counter > 0);
  }
  
  return crc;
}

//=================================================Load/Save
void memoryFeedbackScreen(uint8_t profileId, bool loading) {
  u8g.setPrintPos(10, 50);
  u8g.print(loading ? "Loading" : "Saving");
  u8g.print(" profile ");
  u8g.print(profileId);
}

void saveProfile(unsigned int targetProfile, bool quiet) {
  activeProfileId = targetProfile;
  memoryFeedbackScreen(activeProfileId, false);
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action
  Serial.println("Saving Profile");
  u8g.firstPage();
  do {
    u8g.print("Saving profile");
  } 
  while( u8g.nextPage() );
  delay(500);
}

void loadProfile(unsigned int targetProfile) {
  memoryFeedbackScreen(activeProfileId, true);
  bool ok = loadParameters(targetProfile);

#if 0
  if (!ok) {
    lcd.setPrintPos(0, 2);
    lcd.print("Checksum error!");
    lcd.setPrintPos(0, 3);
    lcd.print("Review profile.");
    delay(2500);
  }
#endif

  // save in any way, as we have no undo
  activeProfileId = targetProfile;
  saveLastUsedProfile();

  delay(500);
}

#define WITH_CHECKSUM 1

bool saveParameters(uint8_t profile) {
  
  uint16_t offset = profile * sizeof(Profile_t);
//  #ifdef WITH_CHECKSUM
//    activeProfile.checksum = crc8((uint8_t *)&activeProfile, sizeof(Profile_t) - sizeof(uint8_t));
//  #endif

  do {
  } 
  while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, (void *)offset, sizeof(Profile_t));
  Serial.print("EEPROM writing "); Serial.print(profile); Serial.print(" at "); Serial.println(offset);
  return true;
}

bool loadParameters(uint8_t profile) {
  Serial.println("Loading profile");
  uint16_t offset = profile * sizeof(Profile_t);

  do {
  } 
  while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, (void *)offset, sizeof(Profile_t));
  
//  Serial.print(F("Current profile ID ")); Serial.println(profile);
  Serial.println(F("Reading values"));
  calculateMode();
  maxProfileTemp[activeProfileId] = 0;
  for(int i = 0; i < NumberOfProfileStages; i++){
    if(activeProfile[i].Temp != 0){
      if(activeProfile[i].Temp > maxProfileTemp[activeProfileId]) maxProfileTemp[activeProfileId] = activeProfile[i].Temp;
      Serial.print(F("Temp#")); Serial.print(i); Serial.print(" "); Serial.println(activeProfile[i].Temp);
      Serial.print(F("Duration#")); Serial.print(i); Serial.print(" "); Serial.println(activeProfile[i].Duration);
      Serial.print(F("Speed#")); Serial.print(i); Serial.print(" "); Serial.println(activeProfile[i].Speed);
      Serial.print(F("Mode#")); Serial.print(i); Serial.print(" "); Serial.println(activeProfile[i].Mode);
    }
  }
  
  
//  #ifdef WITH_CHECKSUM
//    bool chksum = crc8((uint8_t *)&activeProfile, sizeof(Profile_t) - sizeof(uint8_t));
//    Serial.print("Checksum "); Serial.println(chksum);
//    return activeProfile.checksum == chksum;
//  #else
    return true;
//  #endif 
  
}

bool firstRun() {
  // if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
  unsigned int offset = 15 * sizeof(Profile_t);
  for (uint16_t i = offset; i < offset + sizeof(Profile_t); i++) {
    if (EEPROM.read(i) != 255) {
      Serial.println(F("Firstrun False"));
      return false;
    }
  }
  Serial.println(F("Firstrun = true"));
  return true;
}

void populateMaxProfileTemp(){
  //Go through eac profile, load the max temp into the maxProfileTemp array
  Serial.println("Populating max profile temp array");
  for(int i = 0; i < maxProfiles; i++){
    uint16_t offset = i * sizeof(Profile_t);
    do {
    } 
    while (!(eeprom_is_ready()));
    eeprom_read_block(&activeProfile, (void *)offset, sizeof(Profile_t));
    for(int j = 0; j < NumberOfProfileStages; j++){
      if(activeProfile[j].Temp > maxProfileTemp[i]) maxProfileTemp[i] = activeProfile[j].Temp;
    }
    Serial.print("Profile "); Serial.print(i); Serial.print(" max temp "); Serial.println(maxProfileTemp[i]);
  }
}

void makeDefaultProfile() {
  Serial.println("Making default profile");
  activeProfile[0].Temp     = 20;
  activeProfile[0].Duration = 2;
  activeProfile[0].Speed    = 250;
  activeProfile[1].Temp     = 30;
  activeProfile[1].Duration =  5;
  activeProfile[1].Speed    = 250;
  activeProfile[2].Temp     = 40;
  activeProfile[2].Duration =  5;
  activeProfile[2].Speed    = 250;
  activeProfile[3].Temp     = 20;
  activeProfile[3].Duration =  1;
  activeProfile[3].Speed    = 250;
}

void calculateMode(){
  //This is a flag to be written when saving a profile, 0 = up, 1 = down, 2 = off
  Serial.println("Calculating Mode");
  for(int i = 0; i < NumberOfProfileStages; i++){
    //Caluclate and record the direction of programmed temperature travel
    if(i == 0){ //First should always go up
      activeProfile[i].Mode == 0;
    } else { 
      if(activeProfile[i].Temp == 0){
//        Serial.print("Mode #"); Serial.print(i); Serial.println(" Null");
        activeProfile[i].Mode = 2;
      }else if(activeProfile[i].Temp < activeProfile[i-1].Temp){
        activeProfile[i].Mode = 1;
//        Serial.print("Mode #"); Serial.print(i); Serial.println(" down");
      } else {
        activeProfile[i].Mode = 0;
//        Serial.print("Mode #"); Serial.print(i); Serial.println(" up");
      }
    }
  }
}

void factoryReset() {
  Serial.println("Factory Reset");
  #ifndef PIDTUNE
  activeProfileId = 0;
  makeDefaultProfile();
/*  u8g.firstPage();  
  do {    
    u8g.setPrintPos(10, 50);
    u8g.print("Resetting...");
  } 
  while( u8g.nextPage() );
  */


  // then save the same profile settings into all slots
  for (uint8_t i = 0; i < maxProfiles; i++) {
    saveParameters(i);
  }

    Serial.println("Saving profile factory reset");
  
  saveLastUsedProfile();
  delay(500);
#endif
}

void saveLastUsedProfile() {
  EEPROM.write(offsetProfileNum, (uint8_t)activeProfileId & 0xff);
}

void loadLastUsedProfile() {
  activeProfileId = EEPROM.read(offsetProfileNum) & 0xff;
  if(activeProfileId > maxProfiles){
    activeProfileId = 0;
  }
  
  loadParameters(activeProfileId);
}

//=================================================
// Functions

void updateTemp(){  
  if(!TESTING){
    enableTC();
    T = thermocouple.readCelsius();
//    Serial.print("New temperature is "); Serial.println(T);
    Temp = round(T);
  }
}

void fn_CycleStart(m2_el_fnarg_p fnarg) {
  startFiringCycle();
}

void startFiringCycle(){
//  startCycleZeroCrossTicks = zeroCrossTicks;
  m2.setRoot(&top_menu);
  startTime = millis();
  minuteCounterStart = millis();

  running = true;
  //If the kiln was reset, go to skip cycles if they're ove tmeperature
  //Loop through each of the upwards profile stages (Mode == 0) to see if they're zero
  State = 10;
  currentStage = 0;
  for (uint16_t i = 0; i < NumberOfProfileStages; i++){
    if(Temp > activeProfile[i].Temp + 20 && activeProfile[i].Temp != 0 && activeProfile[i].Mode == 0) {
      currentStage =  i;
    }
  }

  Setpoint = Temp;
  updateRampSetpoint();
  startTemp = Temp;
  Serial.print(F("Start temp = ")); Serial.println(Temp);
  Timer1.start();
}

//================================================ Draw running screen
double pxPerM;
double pxPerC;
uint16_t xOffset; // used for wraparound on x axis

void updateProcessDisplay() {

  ///   Serial.println("Update display");

  // header & initial view
//  updateProcessDisplayStartTime = millis();
  initialProcessDisplay = true;
  enableLCD();

  u8g.setFont(FONT14);
  u8g.setPrintPos(2, 18);
  u8g.print((int)Temp); //TODO: Change to temp once debugging is complete
  //if(Input < 1000){
    u8g.print("c");
  //  }
  u8g.setFont(FONT);
  u8g.setPrintPos(60, 7);
  u8g.print("Profile ");
  u8g.print(activeProfileId);
  u8g.setPrintPos(60, 14);
  u8g.print("Stage ");
  u8g.print(currentStage);
  u8g.setPrintPos(60, 23);
  u8g.print("T: ");
  u8g.print((int)Setpoint);
  u8g.print("/");
  u8g.print(activeProfile[currentStage].Temp);
  u8g.print("c");
  u8g.setPrintPos(2, 41);

  u8g.setPrintPos(2, 32);
  u8g.print("Time ");
  u8g.print((int)minutesElapsed);
  u8g.setPrintPos(2, 25);
  u8g.print("PT:");
  u8g.print((int)topTempReached);

  // 200°C grid
//  int16_t t = (uint16_t)(highestTemp * 1.10);
//  for (uint16_t tg = 0; tg <= t; tg += 250) {
//    uint16_t l = h - (tg * pxPerC) - 1;// + yOffset;
//    u8g.drawLine(0, l, w, l);
    //    Serial.println(l);
//  }

  //Hour lines
  for (uint16_t tg = 0; tg <= estimatedTotalTime; tg += 60) {
    uint16_t l = (tg / pxPerM);// + yOffset
 //   Serial.print("Drawing hour line at "); Serial.println(l);
    u8g.drawLine(l, h, l, h-4);
    //    Serial.println(l);
  }
  //Update current temp graph
  int j = minutesElapsed/(int)pxPerM;
  if(j>w) j = w; //Make sure array is within limits, if the kiln takes too long and minutesElapsed formula goes over the array bounds.
  actualTempArray[j] = h - ((int)Temp / pxPerC);
  Serial.print("Graph position "); Serial.print(j); Serial.print(" value "); Serial.println(actualTempArray[j]);
  /*
  if((int)minutesElapsed%(int)pxPerM == 0 && State < 20){
    int val =  h - ((int)Temp * pxPerC);//h - ((int)Temp - (int)startTemp) * pxPerC;
    int j = minutesElapsed/(int)pxPerM; //Get position to write to in array
    if(j>w) j = w; //Make sure array is within limits, if the kiln takes too long and minutesElapsed formula goes over the array bounds.
      actualTempArray[j] =  h - ((int)Temp) * pxPerC; // h - ((int)Temp - (int)startTemp) * pxPerC;
      //Serial.print("Graph position "); Serial.print(j); Serial.print(" value "); Serial.println(actualTempArray[j]);
  }
  */
  
  //Draw graph
  for (int i = 0; i < w; i++){
    u8g.drawPixel(i, predictedTempArray[i]);
    if(actualTempArray[i] != 0){
      u8g.drawPixel(i, actualTempArray[i]);
    }
  }
 if(spinner){
    u8g.drawPixel(w-2,h-2);
    u8g.drawPixel(w-1,h-2);
    u8g.drawPixel(w,h-2);
    spinner = false;
  }else {
    u8g.drawPixel(w-2,h-2);
    u8g.drawPixel(w-2,h-1);
    u8g.drawPixel(w-2,h-0);
    spinner = true;
  }
}

//================================================



//=================================================
// Load Profile

void fn_loadProfile(m2_el_fnarg_p fnarg) {
  /* accept selection */
  loadProfile(activeProfileCursor);
  loadTempValues();
  activeProfileId = activeProfileCursor;
  m2.setRoot(&top_menu);
}

const char *showMaxTempLabelA(m2_rom_void_p element){ 
  strcpy(s, "Max Temp : ");
  itoa((int)maxProfileTemp[activeProfileCursor], s+strlen(s), 10);  
  //  Serial.println(s); 
  return s;}
M2_LABELFN(el_MaxTemperatureLabelA, NULL, showMaxTempLabelA);

uint8_t fn_activeProfileCursor(m2_rom_void_p element, uint8_t msg, uint8_t val){ 
  if ( msg == M2_U8_MSG_SET_VALUE ){
    activeProfileCursor = val;
  }
  
  return activeProfileCursor;
}

M2_LABEL(el_loadProfileL, NULL, "loadProfile");
M2_U8NUMFN(el_activeProfileCursor, "c2", 0, maxProfiles, fn_activeProfileCursor);
M2_BUTTON(el_loadProfileOK, NULL, "OK", fn_loadProfile);


M2_LIST(el_loadProfile) = {
  &el_loadProfileL,
  &el_MaxTemperatureLabelA,
  &el_activeProfileCursor,
  &el_loadProfileOK
};

M2_VLIST(el_loadProfileMenuVList, NULL, el_loadProfile);
M2_ALIGN(el_loadProfileMenu, "W64H64", &el_loadProfileMenuVList);    
//-------------------------------------------------

// Save Profile

void fn_saveProfile(m2_el_fnarg_p fnarg) {
  /* accept selection */
  Serial.print("Saving profile ");
  Serial.println(activeProfileCursor1);
  saveProfile(activeProfileCursor1,true);
  activeProfileId = activeProfileCursor1;
  m2.setRoot(&top_menu);
}
M2_LABEL(el_saveProfileL, NULL, "saveProfile");
M2_U8NUM(el_activeProfileCursorSave, "c2", 0, 30, &activeProfileCursor1);
M2_BUTTON(el_saveProfileOK, NULL, "OK", fn_saveProfile);


M2_LIST(el_saveProfile) = {
  &el_saveProfileL,
  &el_activeProfileCursorSave,
  &el_saveProfileOK
};

M2_VLIST(el_saveProfileMenuVList, NULL, el_saveProfile);
M2_ALIGN(el_saveProfileMenu, "W64H64", &el_saveProfileMenuVList);  //


//-------------------------------------------------

// Edit States

uint32_t TempBeingEdited;
uint32_t DurationBeingEdited;
uint8_t SpeedBeingEdited;

void fn_StateOK(m2_el_fnarg_p fnarg) {
  /* accept selection */
  for(int i = 0; i < NumberOfProfileStages; i++){
    activeProfile[i].Temp = TempTMP[i];
    activeProfile[i].Duration = DurationTMP[i];
    activeProfile[i].Speed = SpeedTMP[i];
  }
  drawGraph();
  m2.setRoot(&el_profile_menu);
}

void fn_StateCancel(m2_el_fnarg_p fnarg) {
  /* accept selection */
  for(int i = 0; i < NumberOfProfileStages; i++){
    TempTMP[i] = activeProfile[i].Temp;
    DurationTMP[i] = activeProfile[i].Duration;
    SpeedTMP[i] = activeProfile[i].Speed;
  }
  m2.setRoot(&el_profile_menu);
}
uint8_t updateStateView(m2_rom_void_p element, uint8_t msg, uint8_t val){ 
  if ( msg == M2_U8_MSG_SET_VALUE ){
    stageBeingModified = val;
  }
  return stageBeingModified;
}

uint32_t updateTemp(m2_rom_void_p element, uint8_t msg, uint32_t val){
  if ( msg == M2_U8_MSG_SET_VALUE ){
    TempTMP[stageBeingModified] = val;
  }
  return TempTMP[stageBeingModified];
}
uint32_t updateDuration(m2_rom_void_p element, uint8_t msg, uint32_t val){
  if ( msg == M2_U8_MSG_SET_VALUE ){
    DurationTMP[stageBeingModified] = val;
  }
  return DurationTMP[stageBeingModified];
}
uint8_t updateSpeed(m2_rom_void_p element, uint8_t msg, uint8_t val){
  if (SpeedTMP[stageBeingModified] < 50){
    SpeedTMP[stageBeingModified] = 50;
  }
  if ( msg == M2_U8_MSG_SET_VALUE ){
    SpeedTMP[stageBeingModified] = val + 9;
  }
  return SpeedTMP[stageBeingModified];
}

M2_LABEL(el_StateAboutL, NULL, "Stage Control");
M2_LABEL(el_StateAbout2L, NULL, "");
//Which stage are we changing
M2_LABEL(el_StageCursorL, NULL, "Stage #");
M2_U8NUMFN(el_StagenCursor, "c2" ,0, NumberOfProfileStages, updateStateView);

M2_LABEL(el_StateTempL, NULL, "Temp");
M2_U32NUMFN(el_StateTemp, "c4", updateTemp);
M2_LABEL(el_StateHoldL, NULL, "Hold Time");
M2_U32NUMFN(el_StateHold, "c3", updateDuration);
M2_LABEL(el_StateSpeedL, NULL, "Speed C");
M2_U8NUMFN(el_StateSpeed, "c3" , 0, 255, updateSpeed);

M2_BUTTON(el_StateOK, NULL, "OK", fn_StateOK);
M2_BUTTON(el_StateCancel, NULL, "Cancel", fn_StateCancel);

M2_LIST(el_State) = {
  &el_StateAboutL, &el_StateAbout2L,
  &el_StageCursorL, &el_StagenCursor,
  &el_StateTempL, &el_StateTemp,
  &el_StateHoldL,&el_StateHold,
  &el_StateSpeedL,&el_StateSpeed,
  &el_StateOK, &el_StateCancel
};

M2_GRIDLIST(el_StatekMenuVList, "c2", el_State);
M2_ALIGN(el_StateMenu, "W64H64", &el_StatekMenuVList);  

//========================================================

void fn_EPok(m2_el_fnarg_p fnarg) {
  /* accept selection */
  for(int i = 0; i < NumberOfProfileStages; i++){
    activeProfile[i].Temp = TempTMP[i];
    activeProfile[i].Duration = DurationTMP[i];
    activeProfile[i].Speed = SpeedTMP[i];
  }
  calculateMode();
  m2.setRoot(&top_menu);
}


void fn_EPcancel(m2_el_fnarg_p fnarg) {
  /* accept selection */
  for(int i = 0; i < NumberOfProfileStages; i++){
    TempTMP[i] = activeProfile[i].Temp;
    DurationTMP[i] = activeProfile[i].Duration;
    SpeedTMP[i] = activeProfile[i].Speed;
  }
  m2.setRoot(&top_menu);
}

void fn_StateMenu(m2_el_fnarg_p fnarg) {
  //Goto state menu
 for(int i = 0; i < NumberOfProfileStages; i++){
    TempTMP[i] = activeProfile[i].Temp;
    DurationTMP[i] = activeProfile[i].Duration;
    SpeedTMP[i] = activeProfile[i].Speed;
  }
  m2.setRoot(&el_StateMenu);
}

//M2_LABEL(el_profile_menu_Label, NULL, "Edit Profile");

const char *showCurrentProfile(m2_rom_void_p element){ 
  strcpy(s, "Current profile is ");
  itoa((int)activeProfileId, s+strlen(s), 10);  
//  Serial.println(s); 
  return s;
}
M2_LABELFN(el_current_profile, NULL, showCurrentProfile);

M2_ROOT(el_goto_LoadProfile, NULL, "Load Profile", &el_loadProfileMenu);
M2_ROOT(el_goto_SaveProfile, NULL, "Save Profile", &el_saveProfileMenu);
M2_BUTTON(el_EPcancel, NULL, "cancel", fn_EPcancel);
M2_BUTTON(el_EPok, NULL, " ok ", fn_EPok);
//M2_ROOT(el_goto_StateMenu, NULL, "States", &el_StateMenu);
M2_BUTTON(el_goto_StateMenu, NULL, "States", fn_StateMenu);
M2_LIST(el_profileMenuList) = {
//  &el_profile_menu_Label, 
  &el_current_profile,
  &el_goto_StateMenu, 
  &el_goto_LoadProfile,
  &el_goto_SaveProfile,
  &el_EPok, &el_EPcancel
};

M2_VLIST(el_profilegridlist, NULL, el_profileMenuList);

M2_ALIGN(el_profile_menu, "-1|2W64H64", &el_profilegridlist);

//========================================================
void fn_PreviewOk(m2_el_fnarg_p fnarg) {
  m2.setFont(0, FONT);
  m2.setRoot(&top_menu);
}

M2_BUTTON(el_PreviewOk, NULL, " ok ", fn_PreviewOk);

M2_LABEL(el_nullL, NULL, "");

const char *showTempLabel0(m2_rom_void_p element){ 
  strcpy(s, "T0: ");
  itoa((int)activeProfile[0].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp0_label, NULL, showTempLabel0);

const char *showSpeedLabel0(m2_rom_void_p element){ 
  strcpy(s, " S0: ");
  itoa((int)activeProfile[0].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed0_label, NULL, showSpeedLabel0);

const char *showDurationLabel0(m2_rom_void_p element){ 
  strcpy(s, " D0: ");
  itoa((int)activeProfile[0].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration0_label, NULL, showDurationLabel0);

const char *showTempLabel1(m2_rom_void_p element){ 
  strcpy(s, "T1: ");
  itoa((int)activeProfile[1].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp1_label, NULL, showTempLabel1);

const char *showSpeedLabel1(m2_rom_void_p element){ 
  strcpy(s, " S1: ");
  itoa((int)activeProfile[1].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed1_label, NULL, showSpeedLabel1);

const char *showDurationLabel1(m2_rom_void_p element){ 
  strcpy(s, " D1: ");
  itoa((int)activeProfile[1].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration1_label, NULL, showDurationLabel1);

const char *showTempLabel2(m2_rom_void_p element){ 
  strcpy(s, "T2: ");
  itoa((int)activeProfile[2].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp2_label, NULL, showTempLabel2);

const char *showSpeedLabel2(m2_rom_void_p element){ 
  strcpy(s, " S2: ");
  itoa((int)activeProfile[2].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed2_label, NULL, showSpeedLabel2);

const char *showDurationLabel2(m2_rom_void_p element){ 
  strcpy(s, " D2: ");
  itoa((int)activeProfile[2].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration2_label, NULL, showDurationLabel2);

const char *showTempLabel3(m2_rom_void_p element){ 
  strcpy(s, "T3: ");
  itoa((int)activeProfile[3].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp3_label, NULL, showTempLabel3);

const char *showSpeedLabel3(m2_rom_void_p element){ 
  strcpy(s, " S3: ");
  itoa((int)activeProfile[3].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed3_label, NULL, showSpeedLabel3);

const char *showDurationLabel3(m2_rom_void_p element){ 
  strcpy(s, " D3: ");
  itoa((int)activeProfile[3].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration3_label, NULL, showDurationLabel3);

const char *showTempLabel4(m2_rom_void_p element){ 
  strcpy(s, "T4: ");
  itoa((int)activeProfile[4].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp4_label, NULL, showTempLabel4);

const char *showSpeedLabel4(m2_rom_void_p element){ 
  strcpy(s, " S4: ");
  itoa((int)activeProfile[4].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed4_label, NULL, showSpeedLabel4);

const char *showDurationLabel4(m2_rom_void_p element){ 
  strcpy(s, " D4: ");
  itoa((int)activeProfile[4].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration4_label, NULL, showDurationLabel4);

const char *showTempLabel5(m2_rom_void_p element){ 
  strcpy(s, "T5: ");
  itoa((int)activeProfile[5].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp5_label, NULL, showTempLabel5);

const char *showSpeedLabel5(m2_rom_void_p element){ 
  strcpy(s, " S5: ");
  itoa((int)activeProfile[5].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed5_label, NULL, showSpeedLabel5);

const char *showDurationLabel5(m2_rom_void_p element){ 
  strcpy(s, " D5: ");
  itoa((int)activeProfile[5].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration5_label, NULL, showDurationLabel5);

const char *showTempLabel6(m2_rom_void_p element){ 
  strcpy(s, "T6: ");
  itoa((int)activeProfile[6].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp6_label, NULL, showTempLabel6);

const char *showSpeedLabel6(m2_rom_void_p element){ 
  strcpy(s, " S6: ");
  itoa((int)activeProfile[6].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed6_label, NULL, showSpeedLabel6);

const char *showDurationLabel6(m2_rom_void_p element){ 
  strcpy(s, " D6: ");
  itoa((int)activeProfile[6].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration6_label, NULL, showDurationLabel6);

const char *showTempLabel7(m2_rom_void_p element){ 
  strcpy(s, "T7: ");
  itoa((int)activeProfile[7].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp7_label, NULL, showTempLabel7);

const char *showSpeedLabel7(m2_rom_void_p element){ 
  strcpy(s, " S7: ");
  itoa((int)activeProfile[7].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed7_label, NULL, showSpeedLabel7);

const char *showDurationLabel7(m2_rom_void_p element){ 
  strcpy(s, " D7: ");
  itoa((int)activeProfile[7].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration7_label, NULL, showDurationLabel7);

const char *showTempLabel8(m2_rom_void_p element){ 
  strcpy(s, "T8: ");
  itoa((int)activeProfile[8].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp8_label, NULL, showTempLabel8);

const char *showSpeedLabel8(m2_rom_void_p element){ 
  strcpy(s, " S8: ");
  itoa((int)activeProfile[8].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed8_label, NULL, showSpeedLabel8);

const char *showDurationLabel8(m2_rom_void_p element){ 
  strcpy(s, " D8: ");
  itoa((int)activeProfile[8].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration8_label, NULL, showDurationLabel8);

const char *showTempLabel9(m2_rom_void_p element){ 
  strcpy(s, "T9: ");
  itoa((int)activeProfile[9].Temp, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_temp9_label, NULL, showTempLabel9);

const char *showSpeedLabel9(m2_rom_void_p element){ 
  strcpy(s, " S9: ");
  itoa((int)activeProfile[9].Speed, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_speed9_label, NULL, showSpeedLabel9);

const char *showDurationLabel9(m2_rom_void_p element){ 
  strcpy(s, " D9: ");
  itoa((int)activeProfile[9].Duration, s+strlen(s), 10);   
  return s;}
M2_LABELFN(el_duration9_label, NULL, showDurationLabel9);

M2_LIST(el_previewMenuList) = {
// &el_profile_menu_Label, 
  &el_temp0_label, 
  &el_duration0_label,
  &el_speed0_label, 
  &el_nullL,
  
  &el_temp1_label, 
  &el_duration1_label,
  &el_speed1_label, 
  &el_nullL,
  
  &el_temp2_label, 
  &el_duration2_label,
  &el_speed2_label, 
  &el_nullL,
  
  &el_temp3_label, 
  &el_duration3_label,
  &el_speed3_label, 
  &el_nullL,
  
  &el_temp4_label, 
  &el_duration4_label,
  &el_speed4_label, 
  &el_nullL,
  
  &el_temp5_label, 
  &el_duration5_label,
  &el_speed5_label, 
  &el_nullL,
  
  &el_temp6_label, 
  &el_duration6_label,
  &el_speed6_label, 
  &el_nullL,
  
  &el_temp7_label, 
  &el_duration7_label,
  &el_speed7_label, 
  &el_nullL,
  
  &el_temp8_label, 
  &el_duration8_label,
  &el_speed8_label, 
  &el_nullL,
  
  &el_temp9_label, 
  &el_duration9_label,
  &el_speed9_label, 
  
  &el_PreviewOk,
};


M2_GRIDLIST(el_previewgridlist, "c4" , el_previewMenuList);
M2_ALIGN(el_preview_profile, "-1|2W64H64", &el_previewgridlist);
//========================================================
//el_goto_preview_profile

//========================================================
M2_LABEL(el_goto_title, NULL, "Kiln Controller");

M2_BUTTON(el_goto_miCycleStart, NULL, "Begin firing", fn_CycleStart);
M2_ROOT(el_goto_profile_menu, NULL, "Edit Profile", &el_profile_menu);
//M2_ROOT(el_goto_preview_profile, NULL, "Preview Profile", &el_preview_profile);
M2_BUTTON(fn_goto_preview_profile, NULL, "Preview Profile", fn_preview_profile);

void fn_preview_profile(m2_el_fnarg_p fnarg) {
  m2.setFont(0, FONT2);
  m2.setRoot(&el_preview_profile);
}

const char *showProfileLabel(m2_rom_void_p element){ 
  strcpy(s, "Profile : ");
  itoa((int)activeProfileId, s+strlen(s), 10);  
  //  Serial.println(s); 
  return s;
}
M2_LABELFN(el_ProfileLabel, NULL, showProfileLabel);

const char *showTempLabel(m2_rom_void_p element){ 
  strcpy(s, "Temp C : ");
  itoa((int)Temp, s+strlen(s), 10);  
  //  Serial.println(s); 
  return s;}
M2_LABELFN(el_TemperatureLabel, NULL, showTempLabel);

const char *showMaxTempLabel(m2_rom_void_p element){ 
  strcpy(s, "Max Temp : ");
  itoa((int)maxProfileTemp[activeProfileId], s+strlen(s), 10);  
  //  Serial.println(s); 
  return s;}
M2_LABELFN(el_MaxTemperatureLabel, NULL, showMaxTempLabel);

M2_LIST(list_menu) = {
  &el_goto_title, 
  &el_goto_miCycleStart,
  &el_goto_profile_menu,
  &fn_goto_preview_profile,
  &el_TemperatureLabel,
  &el_ProfileLabel,
  &el_MaxTemperatureLabel,
};
M2_VLIST(el_menu_vlist, NULL, list_menu);
M2_ALIGN(top_menu, "W64H64", &el_menu_vlist);


//========================================================


void loadTempValues() {
 for(int i = 0; i < NumberOfProfileStages; i++){
    TempTMP[i] = activeProfile[i].Temp;
    DurationTMP[i] = activeProfile[i].Duration;
    SpeedTMP[i] = activeProfile[i].Speed ;
  }
 drawGraph();
}

void drawGraph(){
  for(int i = 0; i < NumberOfProfileStages; i++){
    Serial.print("Stage "); Serial.println(i);
    Serial.print("Temp ");Serial.println(TempTMP[i]);
    Serial.print("Duration ");Serial.println(DurationTMP[i]);
    Serial.print("Speed ");Serial.println(SpeedTMP[i]);
  }
  float timeUpContainer[NumberOfProfileStages];
  estimatedTotalTime = 0;
  timeUpContainer[0] = (activeProfile[0].Temp) / (activeProfile[0].Speed / 60.0);
//  Serial.print("TimeupContainer0 "); Serial.println(timeUpContainer[0]);
  timeUpContainer[0] += activeProfile[0].Duration;
  estimatedTotalTime += timeUpContainer[0];
  Serial.print("Time container #0 "); Serial.println(timeUpContainer[0]);
  for(int i = 1; i < NumberOfProfileStages; i++){ //We calculate the first stage before this loop so i-1 doesnt reference a non existant profile
    //The difference between two temperatures over the speed
     if(activeProfile[i].Mode == 0){ //Temp going up
       timeUpContainer[i] = (activeProfile[i].Temp - activeProfile[i-1].Temp) / (activeProfile[i].Speed / 60.0);
       Serial.print(F("Time container #"));Serial.print(i); Serial.print(F(" Up ")); Serial.println(timeUpContainer[i]);
       timeUpContainer[i] += activeProfile[i].Duration;
       Serial.print(F("Time container w/ hold#"));Serial.print(i); Serial.print(F(" Up ")); Serial.println(timeUpContainer[i]);
     } else if (activeProfile[i].Mode == 1){ //Temp going down
       timeUpContainer[i] = (activeProfile[i-1].Temp - activeProfile[i].Temp) / (activeProfile[i].Speed / 60.0);
       timeUpContainer[i] += activeProfile[i].Duration;
       Serial.print(F("Time container #"));Serial.print(i); Serial.print(F(" Down ")); Serial.println(timeUpContainer[i]);
     } else {
      timeUpContainer[i] = 0;
      Serial.print(F("Time container #"));Serial.print(i); Serial.println(F(" null "));
     }
     estimatedTotalTime += timeUpContainer[i];
     //Lets also find what is the highest programmed temp
     if (activeProfile[i].Temp > highestTemp){
      highestTemp = activeProfile[i].Temp;
    }
    
  }
  Serial.print(F("Highest Temp ")); Serial.println(highestTemp);
  Serial.print(F("Estimated total time : "));
  Serial.println(estimatedTotalTime);
  double totTime = estimatedTotalTime; //Arduino is changing estimatedTotalTime during the for loop below, try to hold it in another container


  //Fill in the graph with estimated temperatures
  tmp = estimatedTotalTime / (float)w;

  Serial.print(F("Minutes per pixel : "));
  Serial.println(tmp);
  pxPerM = tmp; //pxPerM = round(tmp);
  if(estimatedTotalTime < w ){
    if(w/estimatedTotalTime >= 2) {
      pxPerM = w/estimatedTotalTime;
    } else {
      pxPerM = 1;
    }
  } 

  tmp = (highestTemp * 1.1) / graphHeight ;
  pxPerC = tmp;
  Serial.print(F("Deg C per pixel: "));
  Serial.println(pxPerC);

  float val;
  int totalTime = 0; //Value of all previous profiles time elapsed
  int currentStage = 0;
  int profileStartTime = 0; //the minute value at each profile change, use to caluclate if the mode is ramp or hold
  int stageRampDuration = timeUpContainer[0] - activeProfile[0].Duration;
  float rateOfChange = activeProfile[currentStage].Speed / pxPerM; //How many degrees per pixel do we move the graph
  int pxForRamp = stageRampDuration / pxPerM;
  int previousJ = 0; //Prevent the loop from changing value if the pixel is the same as the last calculated step.
//  Serial.print("Estimated total time "); Serial.println(estimatedTotalTime);
  for(int i = 0; i < totTime; i++){
//    Serial.print("i "); Serial.print(i); Serial.print(" limit "); Serial.println(totTime);
   // Serial.print(i);Serial.print(" i% "); Serial.print(i%(int)pxPerM); Serial.print(" ");
    if(i%(int)pxPerM == 0 ){ //Only calculate for each pixel required instead of each minute
      
      int j = i/(float)pxPerM; //Get position to write to in array
      
      //Serial.print(i); Serial.print(" pos "); Serial.println(j);//Appears to be working
      int currentVal = 0;
      
      int lastVal = 0;
      
//      Serial.print("minutes elapsed "); Serial.println(i);
      if(i >= timeUpContainer[currentStage] + totalTime || i == 0) {
        if(i != 0){
          totalTime +=  timeUpContainer[currentStage];
          Serial.print("Estimated profile time elapsed "); Serial.print(currentStage); Serial.print(" "); Serial.println(totalTime);
          if(i != 0)currentStage ++;
        }
        profileStartTime = j;
        Serial.print("Profile Start Time "); Serial.println(i);
        float degCperMin = activeProfile[currentStage].Speed/60.0;
        Serial.print("^c per min "); Serial.println(degCperMin);
        Serial.print("Time up container for current stage ");Serial.print(currentStage); Serial.print(" "); Serial.println(timeUpContainer[currentStage]);
        Serial.print("Hold duration for current stage ");Serial.print(currentStage); Serial.print(" "); Serial.println(activeProfile[currentStage].Duration);
        stageRampDuration = timeUpContainer[currentStage] - activeProfile[currentStage].Duration;        
        Serial.print("stageRampDuration "); Serial.println(stageRampDuration);
        rateOfChange = pxPerM * degCperMin;
        Serial.print("Rate of change "); Serial.println(rateOfChange);
        pxForRamp = stageRampDuration / pxPerM;
        Serial.print("Px For Ramp "); Serial.println(pxForRamp);
      }

      if(i == 0){
        val = Temp;
      } else if (previousJ  == j) {
      } else {
        if(j < pxForRamp + profileStartTime){ //This should be the time spent ramping from the start of the change of profile
          if(activeProfile[currentStage].Mode == 0){ //Temp is going up
            val += rateOfChange;
          } else { //Temp is going down
            val -= rateOfChange;
          }
        }else { 
          val = activeProfile[currentStage].Temp;
        }
      }
      previousJ = j;
      
      predictedTempArray[j] = h - (val / pxPerC);
      Serial.print("j "); 
      Serial.print(j); 
      Serial.print(" i "); 
      Serial.print(i); 
      Serial.print(" deg C "); 
      Serial.print(val); 
      Serial.print(" @px ");
      Serial.println(predictedTempArray[j]);
    }
  }
 // totalTime +=  timeUpContainer[currentProfile];
        Serial.print("Estimated total graph time elapsed "); Serial.println(totalTime);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Begin");
  if(TESTING)factoryReset();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  pinMode(LCD_CS, OUTPUT);
  pinMode(TC_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(TC_CS, HIGH);
  u8g.firstPage();
  Serial.println("Drawing Splash");
  do {
    u8g.setPrintPos(0, 13);
    u8g.setFont(FONT12);
    //tft.setTextSize(2);
    u8g.print("Kiln");
    u8g.setPrintPos(0, 26);
    u8g.print("Controller");
    u8g.setFont(FONT10);
    //tft.setTextSize(1);
    u8g.setPrintPos(0, 39);
    u8g.print("v"); 
    u8g.print(ver);
  } 
  while( u8g.nextPage() );
  delay(1500);
  Serial.println("Begin");
  
  wdt_enable(WDTO_8S); //Enable watchdog timer for 8 second intervals, if wdt_reset(); is not called within 8 seconds, arduino reboots.


  m2_SetU8g(u8g.getU8g(), m2_u8g_box_icon);
  // Assign u8g font to index 0
  m2.setFont(0, FONT);
  
  // define button for the select message
  m2.setPin(M2_KEY_SELECT, ROTARY_ENCODER_SELECT);          // dogm128 shield, 2nd from top
  // The incremental rotary encoder is conected to these two pins
  m2.setPin(M2_KEY_ROT_ENC_A, ROTARY_ENCODER_A);
  m2.setPin(M2_KEY_ROT_ENC_B, ROTARY_ENCODER_B);
  if (firstRun()) {
    Serial.println("Factory Reset");
    factoryReset();
    populateMaxProfileTemp();
    loadParameters(0);
  }
  else {
    Serial.println("Loading Last Profile");
    populateMaxProfileTemp();
    loadLastUsedProfile();
  }
  enableTC();
  loadTempValues();


//  temperature = new MAX31856(SDI, SDO, TC_CS, SCK);
  // Initializing the MAX31855's registers
//  temperature->writeRegister(REGISTER_CR0, CR0_INIT);
//  temperature->writeRegister(REGISTER_CR1, CR1_INIT);
//  temperature->writeRegister(REGISTER_MASK, MASK_INIT);


  updateTemp();
  delay(100);
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(updateRampSetpoint);
  Timer1.stop();
  if(TESTING)startFiringCycle();
}

void loop() {
  wdt_reset();
  now = millis();
  delta = delta + (now-lastTime) / ns;
  delta2 = delta2 + (now-lastTime2) / ns2;
  delta3 = delta3 + (now-lastTime3) / ns3;
  lastTime = now;
  lastTime2 = now;
  lastTime3 = now;
  m2.checkKey(); 

  if (!running){
    if ( m2.handleKey() != 0) {
      u8g.firstPage();  
      do {
        m2.draw();
        m2.checkKey(); 
      } 
      while( u8g.nextPage() );
      displayUpdate = false;
   } 
  }
  
  while (delta >= 1){
    minutesElapsed = (millis() - startTime) / 60000.0;   
    //Serial.println("Drawing menu");
    if(running){
      u8g.firstPage();
      do {
        updateProcessDisplay();
      } 
      while( u8g.nextPage() );     
    } 
    displayUpdate = true;
    delta--;
  }

  minuteCounter = millis();
  if(minuteCounter - minuteCounterStart >= 10000){
    minuteCounterStart = millis();
  }

  while (delta3 >=1){   
    runRelays();
    delta3--;
 }
 
  while (delta2 >=1){      
    updateTemp();
      if(Temp > topTempReached){
        topTempReached = Temp;
      }  
      delta2--;
    } 
  }


int soakTime;

void updateRampSetpoint() {
  Serial.print("Update Ramp Setpoint "); Serial.println(Setpoint);
//  Serial.print("Current state "); Serial.println(currentStage);
//  Serial.print("Current Mode "); Serial.println(activeProfile[currentStage].Mode);
//Serial.print("Temp change "); Serial.println(activeProfile[currentStage].Speed/(60.0*60.0));
  
  if(activeProfile[currentStage].Mode == 0){
    if(Setpoint < activeProfile[currentStage].Temp)  {
      Setpoint += activeProfile[currentStage].Speed/(60.0*60.0);
      if(TESTING)Temp += activeProfile[currentStage].Speed/(60.0*60.0);
//      Serial.print("New Setpoint = "); Serial.print(Setpoint); Serial.print(" at rate of "); Serial.print((float)activeProfile[currentState].Speed/(60.0*60.0)); 
    } else {
      if(!isHolding){
        isHolding = true;
        soakTime = minutesElapsed;
        Setpoint = activeProfile[currentStage].Temp;
      }
    }
  }
  
  if(activeProfile[currentStage].Mode == 1){
    if(Setpoint > activeProfile[currentStage].Temp)  {
      Setpoint -= activeProfile[currentStage].Speed/(60.0*60.0);
      if(TESTING)Temp -= activeProfile[currentStage].Speed/(60.0*60.0);
    } else {
      if(!isHolding){
        isHolding = true;
        soakTime = minutesElapsed;
        Setpoint = activeProfile[currentStage].Temp;
      }
    }
  }

  
  
  if(activeProfile[currentStage].Mode >1) { //Off
    Setpoint = 0;
    State = 0;
    digitalWrite(RELAY_PIN, LOW);
    Timer1.stop();
   }

  if(isHolding){
    if (minutesElapsed < soakTime + activeProfile[currentStage].Duration){
      Setpoint = activeProfile[currentStage].Temp;
      if(TESTING){
        Serial.print("Minutes Elapsed ");Serial.print(minutesElapsed); 
        Serial.print(" soak time "); Serial.print(soakTime); 
        Serial.print(" Duration "); Serial.println(activeProfile[currentStage].Duration);
        
      }
      //Soak setpoint stays the same 
    } else {
      currentStage ++;
      isHolding = false;
      Serial.print("Soak finished, starting next cycle ");Serial.println(currentStage);
      Serial.print("Current mode "); Serial.println(activeProfile[currentStage].Mode);
      Serial.print("Current Target "); Serial.println(activeProfile[currentStage].Temp);
      if(activeProfile[currentStage].Temp == 0){
        running = false;
        digitalWrite(RELAY_PIN, LOW);
        Setpoint = 0;
        Serial.println("Finished");
      }
    }
  }

  if(Temp > 1320 || Temp < -50){//Outer bounds of temperature. Force off
    errorTimer++;
    if(errorTimer > 60){
    State = 0;
     digitalWrite(RELAY_PIN, LOW);
     Setpoint = 0;
     Timer1.stop();
     running = false;
     enableLCD();
     u8g.firstPage();
     do{
      u8g.setFont(FONT14);
      u8g.setPrintPos(2, 18);
      u8g.print("ERROR: TEMP"); //TODO: Change to temp once debugging is complete
      }
    while( u8g.nextPage() );
    }
  }
//  Serial.println("Setpoint change complete");
  if(TESTING)Serial.print("Temp "); Serial.println(Temp);
}


void enableTC(){
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(TC_CS, LOW);
}
void enableLCD(){
  digitalWrite(LCD_CS, LOW);
  digitalWrite(TC_CS, HIGH);
}

void runRelays(){
  if(running){
if(Temp <= Setpoint ){
      digitalWrite(RELAY_PIN, LOW);
    } else {
      digitalWrite(RELAY_PIN, HIGH);
   }
  } else {
    digitalWrite(RELAY_PIN, HIGH);
  }
}
