
////////////////// User Settings /////////////////////////
  /* 
   *  Wheel angle sensor zero point...
   *  
   * 3320 minus 127 = 3193. 1288 counts per 1 volt
   * The 127 (half of 255) is so you can take WAS zero of 3320 
   * from 3193 to 3347. Zero from AOG is sent as a number from 0 to 255. 
   * 
   * Leave at 3193 - Only change if using a factory OEM wheel angle Sensor
   * Put your wheels straight forward, adjust WAS physical position
   * So it puts out 2.5v. Do a good installation!
   * 
   * Factory WAS - Wheels pointing forward, measure the voltage. 
   * Example 2.2v - So, 2.5 minus 2.2 = 0.3v times 
   * 1288 counts per volt = 386 counts. 3320 - 386 - 127 = 2706.
   * So your new WAS_ZERO is 2706.
   */
  #define WAS_ZERO 3193    

  //How many degrees before decreasing Max PWM
  #define LOW_HIGH_DEGREES 5.0

  //value for max step in roll noise 5 is slow, 30 is too fast (noisier)
  #define ROLL_DSP_STEP 5

  //PWM Frequency -> 490hz (default) = 0 and -> 122hz = 1  -> 3921hz = 2
  #define PWM_Frequency 0

  #define NUMPIXELS   13                 // Odd number dont use =0 
  #define Neopixel_Pin 5                 // Note this clashes with IBT2
  #define mmPerLightbarPixel  20         // 40 = 4cm

  //#define DEBUG  // uncomment this to add serial debug output
  //#define DEBUG_LOOP_TIME  // uncomment this to add serial debug output

  // Create DEBUG_PRINT command that only prints to serial if DEBUG is defined at the top of the program
  #ifdef DEBUG
   #define DEBUG_PRINT(x)  Serial.println (x)
  #else
   #define DEBUG_PRINT(x)
  #endif

  // Address of CMPS14 shifted right one bit for arduino wire library
  #define CMPS14_ADDRESS 0x60
  int Heading16x = 0;
  int Roll16x = 0;

  // BNO08x definitions
  #include "BNO08x_AOG.h"
  #define REPORT_INTERVAL 90 //Report interval in ms (same as the delay at the bottom)
  const byte bno08xAddresses[] = {0x4A,0x4B};
  const int nrBNO08xAdresses = sizeof(bno08xAddresses)/sizeof(bno08xAddresses[0]);
  byte bno08xAddress;
  BNO080 bno08x;
  float bno08xHeading = 0;
  float bno08xRoll = 0;
  float bno08xPitch = 0;

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;

/////////////////////////////////////////////

  #if NUMPIXELS > 0
    #include <Adafruit_NeoPixel.h>
    Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, Neopixel_Pin, NEO_GRB + NEO_KHZ800);
    const byte centerpixel = (NUMPIXELS-1) /2;
    byte levelcolor[NUMPIXELS][3];
  #endif

  // if not in eeprom, overwrite 
  #define EEP_Ident 4310  

  //version in AOG ex. v4.3.10 -> 4+3+10=17
  #define aogVersion 17

  //   ***********  Motor drive connections  **************888
  //Connect ground only for cytron, Connect Ground and +5v for IBT2
    
  //Dir1 for Cytron Dir, Both L and R enable for IBT2
  #define DIR1_RL_ENABLE  4  //PD4

  //PWM1 for Cytron PWM, Left PWM for IBT2
  #define PWM1_LPWM  3  //PD3

  //Not Connected for Cytron, Right PWM for IBT2
  #define PWM2_RPWM  9 //D9

  //--------------------------- Switch Input Pins ------------------------
  #define STEERSW_PIN 6 //PD6
  #define WORKSW_PIN 7  //PD7
  //#define REMOTE_PIN 8  //PB0
  #define WORKSW_ENABLE_PIN 8  //PB0
  #define WORK_LED_PIN 2  //PD2

  // MCP23017 I2C address is 0x20(32)
  #define Addr 0x20
  #define PortA 0x12
  #define PortB 0x13

  #include <Wire.h>
  #include <EEPROM.h> 
  #include "zADS1015.h"
  Adafruit_ADS1115 ads;     // Use this for the 16-bit version ADS1115

  #include "CMPS14_AOG.h"  // I have replaced inclinometer = 2 with the CMPS14 module instead of the GY MMA845x module
  CMPS14 CMPS14_IMU(CMPS14_ADDRESS);
  
  //loop time variables in microseconds  
  const unsigned int LOOP_TIME = 40;      
  unsigned long lastTime = LOOP_TIME;
  unsigned long currentTime = LOOP_TIME;
  byte watchdogTimer = 20;
  byte serialResetTimer = 100; //if serial buffer is getting full, empty it
  
  //inclinometer variables
  float roll, heading;
  
  //Program flow
  bool isDataFound = false, isSettingFound = false, isMachineFound=false, isAogSettingsFound = false; 
  bool CMPS14initialized = false, isRelayActiveHigh = true;
  int header = 0, tempHeader = 0, temp, EEread = 0;
  byte relay = 0, relayHi = 0, uTurn = 0;
  byte remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0, workSwitchEnabled = 0;
  float distanceFromLine = 0, gpsSpeed = 0;
  
  //steering variables
  float steerAngleActual = 0;
  float steerAngleSetPoint = 0; //the desired angle from AgOpen
  int steeringPosition = 0; //from steering sensor
  float steerAngleError = 0; //setpoint - actual
  
  //pwm variables
  int pwmDrive = 0, pwmDisplay = 0;
  float pValue = 0;
  float errorAbs = 0;
  float highLowPerDeg = 0; 

  //Steer switch button  ***********************************************************************************************************
  byte currentState = 1;
  byte reading;
  byte previous = 0;

   //Variables for settings  
   struct Storage {
      float Ko = 0.0f;  //overall gain
      float Kp = 0.0f;  //proportional gain
      float lowPWM = 0.0f;  //band of no action
      float Kd = 0.0f;  //derivative gain 
      float steeringPositionZero = 3320.0;
      byte minPWM=0;
      byte highPWM=100;//max PWM value
      float steerSensorCounts=10;        
  };  Storage steerSettings;  //27 bytes

    //Variables for settings - 0 is false  
   struct Setup {
      byte InvertWAS = 0;
      byte InvertRoll = 0;
      byte MotorDriveDirection = 0;
      byte SingleInputWAS = 1;
      byte CytronDriver = 0;
      byte SteerSwitch = 0;
      byte UseMMA_X_Axis = 0;
      byte ShaftEncoder = 0;
      
      byte BNOInstalled = 0;
      byte InclinometerInstalled = 2;   // set to 0 for none
                                        // set to 1 if BNO085 (DOGS2 in AOG)
                                        // set to 2 for CMPS14 (MMA8452 in AOG)
      byte maxSteerSpeed = 20;
      byte minSteerSpeed = 1;
      byte PulseCountMax = 5; 
      byte AckermanFix = 100;     //sent as percent
      byte isRelayActiveHigh = 0; //if zero, active low (default)

  };  Setup aogSettings;          //15 bytes

  //reset function
  void(* resetFunc) (void) = 0;

void setup()
{ 
  //set up serial communication
  Serial.begin(38400);
  DEBUG_PRINT("starting setup");
  
  //PWM rate settings. Set them both the same!!!!
  #if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))   // Abstract diferences between Arduino Nano and the Nucleo 32
    {
    DEBUG_PRINT("Setting up Nano PWM frequency...  ");
    // if PWM_Frequency == 0 use default nano PWM frequency (490 Hz)
    if (PWM_Frequency == 1) 
    { 
      TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 to 256 for PWM frequency of   122.55 Hz
      TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 to 256 for PWM frequency of   122.55 Hz
    }
  
    else if (PWM_Frequency == 2)
    {
      TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 to 8 for PWM frequency of  3921.16 Hz
      TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 to 8 for PWM frequency of  3921.16 Hx
    }
    DEBUG_PRINT("Done");
  }
  #else  // otherwise assume it is an STM32 based board
  {
    Serial.println("Setting up STM32 frequency...  ");
    TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PWM1_LPWM), PinMap_PWM);
    TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PWM2_RPWM), PinMap_PWM);
    uint32_t channel1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PWM1_LPWM), PinMap_PWM));
    uint32_t channel2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PWM2_RPWM), PinMap_PWM));
  
    HardwareTimer *pwmTimer1 = new HardwareTimer(Instance1);  // Instantiate HardwareTimer object.
    HardwareTimer *pwmTimer2 = new HardwareTimer(Instance2);
  
    // Configure and start PWM at 0%
    int16_t PWM_Freq = 490;  // Arduino nano frequency 490 Hz
    if (PWM_Frequency == 1) { PWM_Freq = 125 ;}
    if (PWM_Frequency == 2) { PWM_Freq = 4000 ;}
    pwmTimer1->setPWM(channel1, PWM1_LPWM, PWM_Freq, 0); // 490 Hertz, 0% dutycycle
    pwmTimer2->setPWM(channel2, PWM2_RPWM, PWM_Freq, 0); // 490 Hertz, 0% dutycycle
    DEBUG_PRINT("Done");
  }
  #endif
  
  //keep pulled high and drag low to activate, noise free safe   
  pinMode(WORKSW_PIN, INPUT_PULLUP); 
  pinMode(STEERSW_PIN, INPUT_PULLUP); 
  //pinMode(REMOTE_PIN, INPUT_PULLUP); 
  pinMode(WORKSW_ENABLE_PIN, INPUT_PULLUP); 
  pinMode(DIR1_RL_ENABLE, OUTPUT);
  
  if (aogSettings.CytronDriver) pinMode(PWM2_RPWM, OUTPUT); 
  
  //set up communication 
  Wire.begin();

  //50Khz I2C
  #if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
  {
    TWBR = 144;
  }
//  #else  // If STM32 leave at default 100KHz for now and see how we get on
//  {
//    STM32d
//  }
  #endif

  //PortB configured as output
  //MCP_Write(0x01,0x00);  
  delay(300);

  DEBUG_PRINT("Reading EEPROM/FLASH...  ");
  EEPROM.get(0, EEread);              // read identifier
    
  if (EEread != EEP_Ident)   // check on first start and write EEPROM
  {           
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(2, WAS_ZERO);
    EEPROM.put(10, steerSettings);   
    EEPROM.put(40, aogSettings);
  }
  else 
  { 
    //EEPROM.get(2, EEread);            // read SteerPosZero
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, aogSettings);
  }
  DEBUG_PRINT("Done");
  
  // for PWM High to Low interpolator
  highLowPerDeg = (steerSettings.highPWM - steerSettings.lowPWM) / LOW_HIGH_DEGREES;

  if (aogSettings.InclinometerInstalled == 1)
  {
    for(int i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];
      
      Serial.print("\r\nChecking for BNO08X on ");
      Serial.println(bno08xAddress, HEX);
      Wire.beginTransmission(bno08xAddress);
      int error = Wire.endTransmission();
  
      if (error == 0)
      {
        Serial.println("Error = 0");
        Serial.print("BNO08X ADDRESs: 0x");
        Serial.println(bno08xAddress, HEX);
        Serial.println("BNO08X Ok.");
        
        // Initialize BNO080 lib        
        if (bno08x.begin(bno08xAddress))
        {
          Wire.setClock(400000); //Increase I2C data rate to 400kHz

          // Use gameRotationVector
          bno08x.enableGameRotationVector(REPORT_INTERVAL); //Send data update every REPORT_INTERVAL in ms for BNO085

          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else 
          {
            Serial.println("BNO08x init fails!!");
          }
        }
        else
        {
          Serial.println("BNO080 not detected at given I2C address.");
        }
      }
      else 
      {
        Serial.println("Error = 4");
        Serial.println("BNO08X not Connected or Found"); 
      }
    }
  }

  if (aogSettings.InclinometerInstalled == 2)
  {
    DEBUG_PRINT("Initialising CMPS14");
    // CMPS14 IMU
    CMPS14initialized = CMPS14_IMU.init();
    if (CMPS14initialized)
    {
      DEBUG_PRINT("CMPS14 software version: "); // print software version
      DEBUG_PRINT(CMPS14_IMU.softwareVersion);
      useCMPS = true;
    }
    else
    {
      Serial.println("CMPS14 init fails!!");
    }
  }

  #if NUMPIXELS > 0
    pixels.begin();
    for (int i =0 ;i < centerpixel;i++){ //Right
      levelcolor[i][0]=0; levelcolor[i][1]=255; levelcolor[i][2]=0; //Green
    }
    for (int i = centerpixel+1;i < NUMPIXELS;i++){ //Left
      levelcolor[i][0]=255; levelcolor[i][1]=0; levelcolor[i][2]=0; //Red
    }
    levelcolor[centerpixel][0]=10;levelcolor[centerpixel][1]=10;levelcolor[centerpixel][2]=0; //Center Yellow
  
  Serial.println("Waiting for AgOpenGPS");
  #endif

}// End of Setup

void loop()
{
	// Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc	 
	currentTime = millis();

	if (currentTime - lastTime >= LOOP_TIME)
	{
		lastTime = currentTime;
  
    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = 12;

    //clean out serial buffer to prevent buffer overflow
    if (serialResetTimer++ > 20)
    {
      while (Serial.available() > 0) char t = Serial.read();
      serialResetTimer = 0;
    }

    if (aogSettings.InclinometerInstalled == 1)  // BNO085 IMU 
    {      
      if (bno08x.dataAvailable() == true)
      {
        bno08xHeading = (bno08x.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
        bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data
        
        if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180°;180°] to [0;360°]
        {
          bno08xHeading = bno08xHeading + 360;
        }
            
        bno08xRoll = (bno08x.getRoll()) * 180.0 / PI; //Convert roll to degrees
        bno08xPitch = (bno08x.getPitch())* 180.0 / PI; // Convert pitch to degrees

        //if not positive when rolling to the right
        if (aogSettings.InvertRoll) { roll *= -1.0; }

        Heading16x = (int)(bno08xHeading * 16);
        Roll16x = (int)(bno08xRoll * 16);
      }
    }
    if (aogSettings.InclinometerInstalled == 2)  // CMPS14 IMU 
    {          
      if (CMPS14initialized)
      {
        roll = CMPS14_IMU.getRoll();
        heading = CMPS14_IMU.getHeading();
  
        //if not positive when rolling to the right
        if (aogSettings.InvertRoll) { roll *= -1.0; }

        Heading16x = (int)(16*heading);
        Roll16x = (int)(16*roll);
      }
    }

    //read all the switches
    workSwitchEnabled = digitalRead(WORKSW_ENABLE_PIN);
    if (workSwitchEnabled)
    {
      workSwitch = digitalRead(WORKSW_PIN);  // read work switch
      digitalWrite(WORK_LED_PIN, HIGH);
    }
    else
    {
      workSwitch = 0;  // read work switch
      digitalWrite(WORK_LED_PIN, LOW);
    }
    
    if (aogSettings.SteerSwitch == 1) //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    }
    else   //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);      
      if (reading == LOW && previous == HIGH) 
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }      
      previous = reading;
    }
  
    //remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    remoteSwitch = 0;  // I am using the remote switch pin to enable / disable the workswitch - therefore set this value to 0 manually
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
    #if Relay_Type == 1
        SetRelays();       //turn on off section relays
    #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
    #endif
    */
    //MCP_Write(PortB,relay);   
  
    //get steering position       
    if (aogSettings.SingleInputWAS)   //Single Input ADS
    {
      steeringPosition = ads.readADC_SingleEnded(0);    //ADS1115 Single Mode 
      
       steeringPosition = (steeringPosition >> 2); //bit shift by 2  0 to 6640 is 0 to 5v
    }    
    else    //ADS1115 Differential Mode
    {
      steeringPosition = ads.readADC_Differential_0_1(); //ADS1115 Differential Mode
           
      steeringPosition = (steeringPosition >> 2); //bit shift by 2  0 to 6640 is 0 to 5v
    }
   
    //DETERMINE ACTUAL STEERING POSITION
    steeringPosition = (steeringPosition - steerSettings.steeringPositionZero);   //read the steering position sensor
          
      //convert position to steer angle. 32 counts per degree of steer pot position in my case
      //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (aogSettings.InvertWAS)
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    else
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts; 

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * aogSettings.AckermanFix)/100;
    
    if (watchdogTimer < 10)
      { 
       //Disable H Bridge for IBT2, hyd aux, etc for cytron
        if (aogSettings.CytronDriver) 
        {
          if (aogSettings.isRelayActiveHigh) 
          {
            digitalWrite(PWM2_RPWM, 0); 
          }
          else  
          {
            digitalWrite(PWM2_RPWM, 1);       
          }
        }
        else digitalWrite(DIR1_RL_ENABLE, 1);     
        
        steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
        //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;
        
        calcSteeringPID();  //do the pid
        motorDrive();       //out to motors the pwm value
      }
    else
      {
        //we've lost the comm to AgOpenGPS, or just stop request
        //Disable H Bridge for IBT2, hyd aux, etc for cytron
        if (aogSettings.CytronDriver) 
        {
          if (aogSettings.isRelayActiveHigh) 
          {
            digitalWrite(PWM2_RPWM, 1); 
          }
          else  
          {
            digitalWrite(PWM2_RPWM, 0);       
          }
        }
        else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2
                
        pwmDrive = 0; //turn off steering motor
        motorDrive(); //out to motors the pwm value
     }  

    #ifdef DEBUG_LOOP_TIME
      Serial.print("Loop time (ms): ");
      Serial.println((unsigned long)millis() - lastTime);
    #endif

  	} //end of timed loop

  //This runs continuously, not timed //// Serial Receive Data/Settings /////////////////
  delay (5);

  if (Serial.available() > 0 && !isDataFound && !isSettingFound && !isMachineFound && !isAogSettingsFound) 
  {
    int temp = Serial.read();
    header = tempHeader << 8 | temp;               //high,low bytes to make int
    tempHeader = temp;                             //save for next time
    if (header == 32766) isDataFound = true;        //Do we have a match?
    else if (header == 32764) isSettingFound = true;     //Do we have a match?
    else if (header == 32762) isMachineFound = true;     //Do we have a match?    
    else if (header == 32763) isAogSettingsFound = true;     //Do we have a match?    
  }

  //Data Header has been found, so the next 8 bytes are the usbData
  if (Serial.available() > 7 && isDataFound)
  {
    isDataFound = false;
    
    //was section control lo byte
    Serial.read();
    gpsSpeed = Serial.read() * 0.25;  //actual speed times 4, single byte

    //distance from the guidance line in mm
    distanceFromLine = (float)(Serial.read() << 8 | Serial.read());   //high,low bytes

    //set point steer angle * 100 is sent
    steerAngleSetPoint = ((float)(Serial.read() << 8 | Serial.read()))*0.01; //high low bytes

    //auto Steer is off if 32020,Speed is too slow, motor pos or footswitch open
    if (distanceFromLine == 32020 | distanceFromLine == 32000 
        | gpsSpeed < aogSettings.minSteerSpeed | gpsSpeed > aogSettings.maxSteerSpeed 
        | steerSwitch == 1 )
    {
     watchdogTimer = 12; //turn off steering motor
     serialResetTimer = 0; //if serial buffer is getting full, empty it
    }
    else          //valid conditions to turn on autosteer
    {
     watchdogTimer = 0;  //reset watchdog
     serialResetTimer = 0; //if serial buffer is getting full, empty it
    }      
    Serial.read();
    Serial.read();

    //////////////////////////////////////////////////////////////////////////////////////

      //Serial Send to agopenGPS **** you must send 10 numbers ****
      Serial.print("127,253,");
      Serial.print((int)(steerAngleActual * 100));  //The actual steering angle in degrees
      Serial.print(",");
      Serial.print((int)(steerAngleSetPoint * 100));  //the setpoint originally sent
      Serial.print(",");
      Serial.print(Heading16x);  // heading in degrees*16 from CMPS14/BNO085
      Serial.print(",");
      Serial.print(Roll16x);  // roll in degrees*16 from CMPS14/BNO085
      Serial.print(",");
      //the status of switch inputs
      Serial.print(switchByte);  //steering switch status 
      Serial.print(","); 
      Serial.print(pwmDisplay);  //steering switch status 
      Serial.println(",0,0");
      Serial.flush();   // flush out buffer
  }

  //Machine Header has been found, so the next 8 bytes are the usbData
  if (Serial.available() > 7 && isMachineFound)
  {
    isMachineFound = false;

    relayHi = Serial.read();
    relay = Serial.read();    
    gpsSpeed = Serial.read() * 0.25;  //actual speed times 4, single byte
    uTurn = Serial.read();    
    
    Serial.read();
    Serial.read();
    Serial.read();
    Serial.read();
  }

  //ArdSettings has been found, so the next 8 bytes are the usbData
  if (Serial.available() > 7 && isAogSettingsFound)
  {
    isAogSettingsFound = false;

    byte checksum = 0;
    byte reed = 0;

    reed = Serial.read();
    checksum += reed;
    byte sett = reed; //setting0
     
    if (bitRead(sett,0)) aogSettings.InvertWAS = 1; else aogSettings.InvertWAS = 0;
    if (bitRead(sett,1)) aogSettings.InvertRoll = 1; else aogSettings.InvertRoll = 0;
    if (bitRead(sett,2)) aogSettings.MotorDriveDirection = 1; else aogSettings.MotorDriveDirection = 0;
    if (bitRead(sett,3)) aogSettings.SingleInputWAS = 1; else aogSettings.SingleInputWAS = 0;
    if (bitRead(sett,4)) aogSettings.CytronDriver = 1; else aogSettings.CytronDriver = 0;
    if (bitRead(sett,5)) aogSettings.SteerSwitch = 1; else aogSettings.SteerSwitch = 0;
    if (bitRead(sett,6)) aogSettings.UseMMA_X_Axis = 1; else aogSettings.UseMMA_X_Axis = 0;
    if (bitRead(sett,7)) aogSettings.ShaftEncoder = 1; else aogSettings.ShaftEncoder = 0;

    //set1 
    reed = Serial.read();
    checksum += reed;
    sett = reed;  //setting1     
    if (bitRead(sett,0)) aogSettings.BNOInstalled = 1; else aogSettings.BNOInstalled = 0;
    if (bitRead(sett,1)) aogSettings.isRelayActiveHigh = 1; else aogSettings.isRelayActiveHigh = 0;

    reed = Serial.read();
    checksum += reed;
    aogSettings.maxSteerSpeed = reed;  //actual speed

    reed = Serial.read();
    checksum += reed;
    aogSettings.minSteerSpeed = reed;    

    reed = Serial.read();
    checksum += reed;
    byte inc = reed;
    aogSettings.InclinometerInstalled = inc & 192;
    aogSettings.InclinometerInstalled = aogSettings.InclinometerInstalled >> 6;
    aogSettings.PulseCountMax = inc & 63;

    reed = Serial.read();
    checksum += reed;
    aogSettings.AckermanFix = reed;  

    reed = Serial.read();
    checksum += reed;    
    
    //send usbData back - version number etc. 
    SendTwoThirty((byte)checksum);

    EEPROM.put(40, aogSettings);
    
    //reset the arduino
    resetFunc();
  }
    
  //Settings Header has been found, 8 bytes are the settings
  if (Serial.available() > 7 && isSettingFound)
  {
    byte checksum = 0;
    byte reed = 0;
    isSettingFound = false;  //reset the flag
    
    //change the factors as required for your own PID values
    reed = Serial.read();
    checksum += reed;
    steerSettings.Kp = ((float)reed);   // read Kp from AgOpenGPS
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.lowPWM = (float)reed;   // read lowPWM from AgOpenGPS
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.Kd = (float)reed;   // read Kd from AgOpenGPS
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.Ko = (float)reed;   // read  from AgOpenGPS
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.steeringPositionZero = WAS_ZERO + reed;  //read steering zero offset
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.minPWM = reed; //read the minimum amount of PWM for instant on
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.highPWM = reed; //
    
    reed = Serial.read();
    checksum += reed;
    steerSettings.steerSensorCounts = reed; //sent as 10 times the setting displayed in AOG

    //send usbData back - version number. 
    SendTwoThirty((byte)checksum);
    
    EEPROM.put(10, steerSettings);
    
    // for PWM High to Low interpolator
    highLowPerDeg = (steerSettings.highPWM - steerSettings.lowPWM) / LOW_HIGH_DEGREES;
  }

  #if NUMPIXELS >0
    lightbar(distanceFromLine);
  #endif

} // end of main loop

void SendTwoThirty(byte check)
{
    //Serial Send to agopenGPS **** you must send 10 numbers ****
    Serial.print("127,230,");
    Serial.print(check); 
    Serial.print(",");
    
    //version in AOG ex. v4.1.13 -> 4+1+13=18
    Serial.print(aogVersion);  
    Serial.println(",0,0,0,0,0,0");
    Serial.flush();   
}

void lightbar(float distanceFromLine ){
  int level = constrain (distanceFromLine / mmPerLightbarPixel , -centerpixel ,centerpixel);
  byte n = level + centerpixel;
    for (int i = 0 ;i < NUMPIXELS; i++){
      if ( (i == centerpixel && i == n)|| //Center
           (level < 0 && i >= n && i < centerpixel && distanceFromLine != 32020)|| //Right Bar
           (level > 0 && i <= n && i > centerpixel && distanceFromLine != 32020) ) //Left Bar
      {
        pixels.setPixelColor(i,levelcolor[i][0],levelcolor[i][1],levelcolor[i][2]);
      }else{
        pixels.setPixelColor(i,0,0,0);//Clear
      }
    }
  pixels.show();
}
