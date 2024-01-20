/** @file
 * Speeduino Initialisation (called at Arduino setup()).
 */
#include "globals.h"
#include "init.h"
#include "storage.h"
#include "updates.h"
#include "speeduino.h"
#include "timers.h"
#include "comms_secondary.h"
#include "comms_CAN.h"
#include "utilities.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "decoders.h"
#include "corrections.h"
#include "idle.h"
#include "table2d.h"
#include "acc_mc33810.h"
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 
#if defined(EEPROM_RESET_PIN)
  #include EEPROM_LIB_H
#endif
#ifdef SD_LOGGING
  #include "SD_logger.h"
  #include "rtc_common.h"
#endif

/** Initialise Speeduino for the main loop.
 * Top level init entry point for all initialisations:
 * - Initialise and set sizes of 3D tables
 * - Load config from EEPROM, update config structures to current version of SW if needed.
 * - Initialise board (The initBoard() is for board X implemented in board_X.ino file)
 * - Initialise timers (See timers.ino)
 * - Perform optional SD card and RTC battery inits
 * - Load calibration tables from EEPROM
 * - Perform pin mapping (calling @ref setPinMapping() based on @ref config2.pinMapping)
 * - Stop any coil charging and close injectors
 * - Initialise schedulers, Idle, Fan, auxPWM, Corrections, AD-conversions, Programmable I/O
 * - Initialise baro (ambient pressure) by reading MAP (before engine runs)
 * - Initialise triggers (by @ref initialiseTriggers() )
 * - Perform cyl. count based initialisations (@ref config2.nCylinders)
 * - Perform injection and spark mode based setup
 *   - Assign injector open/close and coil charge begin/end functions to their dedicated global vars
 * - Perform fuel pressure priming by turning fuel pump on
 * - Read CLT and TPS sensors to have cranking pulsewidths computed correctly
 * - Mark Initialisation completed (this flag-marking is used in code to prevent after-init changes)
 */

#undef LED_BUILTIN
#undef EEPROM_RESET_PIN
#define LED_BUILTIN PC13

void initialiseAll(void)
{   
    currentStatus.fpPrimed = false;
    currentStatus.injPrimed = false;

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    #if defined(CORE_STM32)
    configPage9.intcan_available = 1;   // device has internal canbus
    //STM32 can not currently enabled
    #endif

    /*
    ***********************************************************************************************************
    * EEPROM reset
    */
    #if defined(EEPROM_RESET_PIN)
    uint32_t start_time = millis();
    byte exit_erase_loop = false; 
    pinMode(EEPROM_RESET_PIN, INPUT_PULLUP);  

    //only start routine when this pin is low because it is pulled low
    while (digitalRead(EEPROM_RESET_PIN) != HIGH && (millis() - start_time)<1050)
    {
      //make sure the key is pressed for at least 0.5 second 
      if ((millis() - start_time)>500) {
        //if key is pressed afterboot for 0.5 second make led turn off
        digitalWrite(LED_BUILTIN, HIGH);

        //see if the user reacts to the led turned off with removing the keypress within 1 second
        while (((millis() - start_time)<1000) && (exit_erase_loop!=true)){

          //if user let go of key within 1 second erase eeprom
          if(digitalRead(EEPROM_RESET_PIN) != LOW){
            #if defined(FLASH_AS_EEPROM_h)
              EEPROM.read(0); //needed for SPI eeprom emulation.
              EEPROM.clear(); 
            #else 
              for (int i = 0 ; i < EEPROM.length() ; i++) { EEPROM.write(i, 255);}
            #endif
            //if erase done exit while loop.
            exit_erase_loop = true;
          }
        }
      } 
    }
    #endif
  
    // Unit tests should be independent of any stored configuration on the board!
#if !defined(UNIT_TEST)
    loadConfig();
    doUpdates(); //Check if any data items need updating (Occurs with firmware updates)
#endif


    //Always start with a clean slate on the bootloader capabilities level
    //This should be 0 until we hear otherwise from the 16u2
    configPage4.bootloaderCaps = 0;
    
    initBoard(); //This calls the current individual boards init function. See the board_xxx.ino files for these.
    initialiseTimers();
    
  #ifdef SD_LOGGING
    initRTC();
    initSD();
  #endif

    Serial.begin(115200);
    BIT_SET(currentStatus.status4, BIT_STATUS4_ALLOW_LEGACY_COMMS); //Flag legacy comms as being allowed on startup

    //Repoint the 2D table structs to the config pages that were just loaded
    taeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
    taeTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    taeTable.xSize = 4;
    taeTable.values = configPage4.taeValues;
    taeTable.axisX = configPage4.taeBins;
    maeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
    maeTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    maeTable.xSize = 4;
    maeTable.values = configPage4.maeRates;
    maeTable.axisX = configPage4.maeBins;
    WUETable.valueSize = SIZE_BYTE; //Set this table to use byte values
    WUETable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    WUETable.xSize = 10;
    WUETable.values = configPage2.wueValues;
    WUETable.axisX = configPage4.wueBins;
    ASETable.valueSize = SIZE_BYTE;
    ASETable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    ASETable.xSize = 4;
    ASETable.values = configPage2.asePct;
    ASETable.axisX = configPage2.aseBins;
    ASECountTable.valueSize = SIZE_BYTE;
    ASECountTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    ASECountTable.xSize = 4;
    ASECountTable.values = configPage2.aseCount;
    ASECountTable.axisX = configPage2.aseBins;
    PrimingPulseTable.valueSize = SIZE_BYTE;
    PrimingPulseTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    PrimingPulseTable.xSize = 4;
    PrimingPulseTable.values = configPage2.primePulse;
    PrimingPulseTable.axisX = configPage2.primeBins;
    crankingEnrichTable.valueSize = SIZE_BYTE;
    crankingEnrichTable.axisSize = SIZE_BYTE;
    crankingEnrichTable.xSize = 4;
    crankingEnrichTable.values = configPage10.crankingEnrichValues;
    crankingEnrichTable.axisX = configPage10.crankingEnrichBins;

    dwellVCorrectionTable.valueSize = SIZE_BYTE;
    dwellVCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    dwellVCorrectionTable.xSize = 6;
    dwellVCorrectionTable.values = configPage4.dwellCorrectionValues;
    dwellVCorrectionTable.axisX = configPage6.voltageCorrectionBins;
    injectorVCorrectionTable.valueSize = SIZE_BYTE;
    injectorVCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    injectorVCorrectionTable.xSize = 6;
    injectorVCorrectionTable.values = configPage6.injVoltageCorrectionValues;
    injectorVCorrectionTable.axisX = configPage6.voltageCorrectionBins;
    injectorAngleTable.valueSize = SIZE_INT;
    injectorAngleTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    injectorAngleTable.xSize = 4;
    injectorAngleTable.values = configPage2.injAng;
    injectorAngleTable.axisX = configPage2.injAngRPM;
    IATDensityCorrectionTable.valueSize = SIZE_BYTE;
    IATDensityCorrectionTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    IATDensityCorrectionTable.xSize = 9;
    IATDensityCorrectionTable.values = configPage6.airDenRates;
    IATDensityCorrectionTable.axisX = configPage6.airDenBins;
    baroFuelTable.valueSize = SIZE_BYTE;
    baroFuelTable.axisSize = SIZE_BYTE;
    baroFuelTable.xSize = 8;
    baroFuelTable.values = configPage4.baroFuelValues;
    baroFuelTable.axisX = configPage4.baroFuelBins;
    IATRetardTable.valueSize = SIZE_BYTE;
    IATRetardTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    IATRetardTable.xSize = 6;
    IATRetardTable.values = configPage4.iatRetValues;
    IATRetardTable.axisX = configPage4.iatRetBins;
    CLTAdvanceTable.valueSize = SIZE_BYTE;
    CLTAdvanceTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    CLTAdvanceTable.xSize = 6;
    CLTAdvanceTable.values = (byte*)configPage4.cltAdvValues;
    CLTAdvanceTable.axisX = configPage4.cltAdvBins;
    idleTargetTable.valueSize = SIZE_BYTE;
    idleTargetTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    idleTargetTable.xSize = 10;
    idleTargetTable.values = configPage6.iacCLValues;
    idleTargetTable.axisX = configPage6.iacBins;
    idleAdvanceTable.valueSize = SIZE_BYTE;
    idleAdvanceTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    idleAdvanceTable.xSize = 6;
    idleAdvanceTable.values = (byte*)configPage4.idleAdvValues;
    idleAdvanceTable.axisX = configPage4.idleAdvBins;
    rotarySplitTable.valueSize = SIZE_BYTE;
    rotarySplitTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    rotarySplitTable.xSize = 8;
    rotarySplitTable.values = configPage10.rotarySplitValues;
    rotarySplitTable.axisX = configPage10.rotarySplitBins;

    flexFuelTable.valueSize = SIZE_BYTE;
    flexFuelTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    flexFuelTable.xSize = 6;
    flexFuelTable.values = configPage10.flexFuelAdj;
    flexFuelTable.axisX = configPage10.flexFuelBins;
    flexAdvTable.valueSize = SIZE_BYTE;
    flexAdvTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    flexAdvTable.xSize = 6;
    flexAdvTable.values = configPage10.flexAdvAdj;
    flexAdvTable.axisX = configPage10.flexAdvBins;
    flexBoostTable.valueSize = SIZE_INT;
    flexBoostTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins (NOTE THIS IS DIFFERENT TO THE VALUES!!)
    flexBoostTable.xSize = 6;
    flexBoostTable.values = configPage10.flexBoostAdj;
    flexBoostTable.axisX = configPage10.flexBoostBins;
    fuelTempTable.valueSize = SIZE_BYTE;
    fuelTempTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    fuelTempTable.xSize = 6;
    fuelTempTable.values = configPage10.fuelTempValues;
    fuelTempTable.axisX = configPage10.fuelTempBins;

    knockWindowStartTable.valueSize = SIZE_BYTE;
    knockWindowStartTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    knockWindowStartTable.xSize = 6;
    knockWindowStartTable.values = configPage10.knock_window_angle;
    knockWindowStartTable.axisX = configPage10.knock_window_rpms;
    knockWindowDurationTable.valueSize = SIZE_BYTE;
    knockWindowDurationTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    knockWindowDurationTable.xSize = 6;
    knockWindowDurationTable.values = configPage10.knock_window_dur;
    knockWindowDurationTable.axisX = configPage10.knock_window_rpms;

    oilPressureProtectTable.valueSize = SIZE_BYTE;
    oilPressureProtectTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    oilPressureProtectTable.xSize = 4;
    oilPressureProtectTable.values = configPage10.oilPressureProtMins;
    oilPressureProtectTable.axisX = configPage10.oilPressureProtRPM;

    coolantProtectTable.valueSize = SIZE_BYTE;
    coolantProtectTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    coolantProtectTable.xSize = 6;
    coolantProtectTable.values = configPage9.coolantProtRPM;
    coolantProtectTable.axisX = configPage9.coolantProtTemp;


    fanPWMTable.valueSize = SIZE_BYTE;
    fanPWMTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    fanPWMTable.xSize = 4;
    fanPWMTable.values = configPage9.PWMFanDuty;
    fanPWMTable.axisX = configPage6.fanPWMBins;

    rollingCutTable.valueSize = SIZE_BYTE;
    rollingCutTable.axisSize = SIZE_SIGNED_BYTE; //X axis is SIGNED for this table. 
    rollingCutTable.xSize = 4;
    rollingCutTable.values = configPage15.rollingProtCutPercent;
    rollingCutTable.axisX = configPage15.rollingProtRPMDelta;

    wmiAdvTable.valueSize = SIZE_BYTE;
    wmiAdvTable.axisSize = SIZE_BYTE; //Set this table to use byte axis bins
    wmiAdvTable.xSize = 6;
    wmiAdvTable.values = configPage10.wmiAdvAdj;
    wmiAdvTable.axisX = configPage10.wmiAdvBins;

    cltCalibrationTable.valueSize = SIZE_INT;
    cltCalibrationTable.axisSize = SIZE_INT;
    cltCalibrationTable.xSize = 32;
    cltCalibrationTable.values = cltCalibration_values;
    cltCalibrationTable.axisX = cltCalibration_bins;

    iatCalibrationTable.valueSize = SIZE_INT;
    iatCalibrationTable.axisSize = SIZE_INT;
    iatCalibrationTable.xSize = 32;
    iatCalibrationTable.values = iatCalibration_values;
    iatCalibrationTable.axisX = iatCalibration_bins;

    o2CalibrationTable.valueSize = SIZE_BYTE;
    o2CalibrationTable.axisSize = SIZE_INT;
    o2CalibrationTable.xSize = 32;
    o2CalibrationTable.values = o2Calibration_values;
    o2CalibrationTable.axisX = o2Calibration_bins;
    
    //Setup the calibration tables
    loadCalibration();

    

    //Set the pin mappings
    if((configPage2.pinMapping == 255) || (configPage2.pinMapping == 0)) //255 = EEPROM value in a blank AVR; 0 = EEPROM value in new FRAM
    {
      //First time running on this board
      resetConfigPages();
      configPage4.triggerTeeth = 4; //Avoiddiv by 0 when start decoders
      setPinMapping(70); //Force board to v0.4
    }
    else { setPinMapping(70); }

    #if defined(NATIVE_CAN_AVAILABLE)
      initCAN();
    #endif

    //Must come after setPinMapping() as secondary serial can be changed on a per board basis
    #if defined(secondarySerial_AVAILABLE)
      if (configPage9.enable_secondarySerial == 1) { secondarySerial.begin(115200); }
    #endif

    //End all coil charges to ensure no stray sparks on startup
    endCoil1Charge();
    endCoil2Charge();
    endCoil3Charge();
    endCoil4Charge();
    endCoil5Charge();
    #if (IGN_CHANNELS >= 6)
    endCoil6Charge();
    #endif
    #if (IGN_CHANNELS >= 7)
    endCoil7Charge();
    #endif
    #if (IGN_CHANNELS >= 8)
    endCoil8Charge();
    #endif

    //Similar for injectors, make sure they're turned off
    closeInjector1();
    closeInjector2();
    closeInjector3();
    closeInjector4();
    closeInjector5();
    #if (INJ_CHANNELS >= 6)
    closeInjector6();
    #endif
    #if (INJ_CHANNELS >= 7)
    closeInjector7();
    #endif
    #if (INJ_CHANNELS >= 8)
    closeInjector8();
    #endif
    
    //Set the tacho output default state
    digitalWrite(pinTachOut, HIGH);
    //Perform all initialisations
    initialiseSchedulers();
    //initialiseDisplay();
    initialiseIdle(true);
    initialiseFan();
    initialiseAirCon();
    initialiseAuxPWM();
    initialiseCorrections();
    BIT_CLEAR(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Clear the I/O error bit. The bit will be set in initialiseADC() if there is problem in there.
    initialiseADC();
    initialiseProgrammableIO();

    //Check whether the flex sensor is enabled and if so, attach an interrupt for it
    if(configPage2.flexEnabled > 0)
    {
      attachInterrupt(digitalPinToInterrupt(pinFlex), flexPulse, CHANGE);
      currentStatus.ethanolPct = 0;
    }
    //Same as above, but for the VSS input
    if(configPage2.vssMode > 1) // VSS modes 2 and 3 are interrupt drive (Mode 1 is CAN)
    {
      attachInterrupt(digitalPinToInterrupt(pinVSS), vssPulse, RISING);
    }

    //Once the configs have been loaded, a number of one time calculations can be completed
    req_fuel_uS = configPage2.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
    inj_opentime_uS = configPage2.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).

    if(configPage10.stagingEnabled == true)
    {
    uint32_t totalInjector = configPage10.stagedInjSizePri + configPage10.stagedInjSizeSec;
    /*
        These values are a percentage of the req_fuel value that would be required for each injector channel to deliver that much fuel.
        Eg:
        Pri injectors are 250cc
        Sec injectors are 500cc
        Total injector capacity = 750cc

        staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
        staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
    */
    staged_req_fuel_mult_pri = (100 * totalInjector) / configPage10.stagedInjSizePri;
    staged_req_fuel_mult_sec = (100 * totalInjector) / configPage10.stagedInjSizeSec;
    }

    if (configPage4.trigPatternSec == SEC_TRIGGER_POLL && configPage4.TrigPattern == DECODER_MISSING_TOOTH)
    { configPage4.TrigEdgeSec = configPage4.PollLevelPolarity; } // set the secondary trigger edge automatically to correct working value with poll level mode to enable cam angle detection in closed loop vvt.
    //Explanation: currently cam trigger for VVT is only captured when revolution one == 1. So we need to make sure that the edge trigger happens on the first revolution. So now when we set the poll level to be low
    //on revolution one and it's checked at tooth #1. This means that the cam signal needs to go high during the first revolution to be high on next revolution at tooth #1. So poll level low = cam trigger edge rising.

    //Begin the main crank trigger interrupt pin setup
    //The interrupt numbering is a bit odd - See here for reference: arduino.cc/en/Reference/AttachInterrupt
    //These assignments are based on the Arduino Mega AND VARY BETWEEN BOARDS. Please confirm the board you are using and update accordingly.
    currentStatus.RPM = 0;
    currentStatus.hasSync = false;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
    currentStatus.runSecs = 0;
    currentStatus.secl = 0;
    //currentStatus.seclx10 = 0;
    currentStatus.startRevolutions = 0;
    currentStatus.syncLossCounter = 0;
    currentStatus.flatShiftingHard = false;
    currentStatus.launchingHard = false;
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10); //Crank RPM limit (Saves us calculating this over and over again. It's updated once per second in timers.ino)
    currentStatus.fuelPumpOn = false;
    currentStatus.engineProtectStatus = 0;
    triggerFilterTime = 0; //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be discarded as noise. This is simply a default value, the actual values are set in the setup() functions of each decoder
    dwellLimit_uS = (1000 * configPage4.dwellLimit);
    currentStatus.nChannels = ((uint8_t)INJ_CHANNELS << 4) + IGN_CHANNELS; //First 4 bits store the number of injection channels, 2nd 4 store the number of ignition channels
    fpPrimeTime = 0;
    ms_counter = 0;
    fixedCrankingOverride = 0;
    timer5_overflow_count = 0;
    toothHistoryIndex = 0;
    toothLastToothTime = 0;

    //Lookup the current MAP reading for barometric pressure
    instanteneousMAPReading();
    readBaro();
    
    noInterrupts();
    initialiseTriggers();

    //The secondary input can be used for VSS if nothing else requires it. Allows for the standard VR conditioner to be used for VSS. This MUST be run after the initialiseTriggers() function
    if( VSS_USES_RPM2() ) { attachInterrupt(digitalPinToInterrupt(pinVSS), vssPulse, RISING); } //Secondary trigger input can safely be used for VSS
    if( FLEX_USES_RPM2() ) { attachInterrupt(digitalPinToInterrupt(pinFlex), flexPulse, CHANGE); } //Secondary trigger input can safely be used for Flex sensor

    //End crank trigger interrupt attachment
    if(configPage2.strokes == FOUR_STROKE)
    {
      //Default is 1 squirt per revolution, so we halve the given req-fuel figure (Which would be over 2 revolutions)
      req_fuel_uS = req_fuel_uS / 2; //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle. If we're doing more than 1 squirt per cycle then we need to split the amount accordingly. (Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the stroke to make the single squirt on)
    }

    //Initial values for loop times
    currentLoopTime = micros_safe();
    mainLoopCount = 0;

    if(configPage2.divider == 0) { currentStatus.nSquirts = 2; } //Safety check.
    else { currentStatus.nSquirts = configPage2.nCylinders / configPage2.divider; } //The number of squirts being requested. This is manually overridden below for sequential setups (Due to TS req_fuel calc limitations)
    if(currentStatus.nSquirts == 0) { currentStatus.nSquirts = 1; } //Safety check. Should never happen as TS will give an error, but leave in case tune is manually altered etc. 

    //Calculate the number of degrees between cylinders
    //Set some default values. These will be updated below if required.
    CRANK_ANGLE_MAX_IGN = 360;
    CRANK_ANGLE_MAX_INJ = 360;

    maxInjOutputs = 1; // Disable all injectors expect channel 1

    ignition1EndAngle = 0;
    ignition2EndAngle = 0;
    ignition3EndAngle = 0;
    ignition4EndAngle = 0;
#if IGN_CHANNELS >= 5
    ignition5EndAngle = 0;
#endif
#if IGN_CHANNELS >= 6
    ignition6EndAngle = 0;
#endif
#if IGN_CHANNELS >= 7
    ignition7EndAngle = 0;
#endif
#if IGN_CHANNELS >= 8
    ignition8EndAngle = 0;
#endif

    if(configPage2.strokes == FOUR_STROKE) { CRANK_ANGLE_MAX_INJ = 720 / currentStatus.nSquirts; }
    else { CRANK_ANGLE_MAX_INJ = 360 / currentStatus.nSquirts; }

    switch (configPage2.nCylinders) {
    case 1:
        channel1IgnDegrees = 0;
        channel1InjDegrees = 0;
        maxIgnOutputs = 1;
        maxInjOutputs = 1;

        //Sequential ignition works identically on a 1 cylinder whether it's odd or even fire. 
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) ) { CRANK_ANGLE_MAX_IGN = 720; }

        if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) )
        {
          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }

        //Check if injector staging is enabled
        if(configPage10.stagingEnabled == true)
        {
          maxInjOutputs = 2;
          channel2InjDegrees = channel1InjDegrees;
        }
        break;

    case 2:
        channel1IgnDegrees = 0;
        channel1InjDegrees = 0;
        maxIgnOutputs = 2;
        maxInjOutputs = 2;
        if (configPage2.engineType == EVEN_FIRE ) { channel2IgnDegrees = 180; }
        else { channel2IgnDegrees = configPage2.oddfire2; }

        //Sequential ignition works identically on a 2 cylinder whether it's odd or even fire (With the default being a 180 degree second cylinder).
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) ) { CRANK_ANGLE_MAX_IGN = 720; }

        if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) )
        {
          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }
        //The below are true regardless of whether this is running sequential or not
        if (configPage2.engineType == EVEN_FIRE ) { channel2InjDegrees = 180; }
        else { channel2InjDegrees = configPage2.oddfire2; }
        if (!configPage2.injTiming) 
        { 
          //For simultaneous, all squirts happen at the same time
          channel1InjDegrees = 0;
          channel2InjDegrees = 0; 
        }

        //Check if injector staging is enabled
        if(configPage10.stagingEnabled == true)
        {
          maxInjOutputs = 4;

          channel3InjDegrees = channel1InjDegrees;
          channel4InjDegrees = channel2InjDegrees;
        }

        break;

    case 3:
        channel1IgnDegrees = 0;
        maxIgnOutputs = 3;
        maxInjOutputs = 3;
        if (configPage2.engineType == EVEN_FIRE )
        {
          //Sequential and Single channel modes both run over 720 crank degrees, but only on 4 stroke engines.
          if( ( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage4.sparkMode == IGN_MODE_SINGLE) ) && (configPage2.strokes == FOUR_STROKE) )
          {
            channel2IgnDegrees = 240;
            channel3IgnDegrees = 480;

            CRANK_ANGLE_MAX_IGN = 720;
          }
          else
          {
            channel2IgnDegrees = 120;
            channel3IgnDegrees = 240;
          }
        }
        else
        {
          channel2IgnDegrees = configPage2.oddfire2;
          channel3IgnDegrees = configPage2.oddfire3;
        }

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) )
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 120;
          channel3InjDegrees = 240;

          if(configPage2.injType == INJ_TYPE_PORT)
          { 
            //Force nSquirts to 2 for individual port injection. This prevents TunerStudio forcing the value to 3 even when this isn't wanted. 
            currentStatus.nSquirts = 2;
            if(configPage2.strokes == FOUR_STROKE) { CRANK_ANGLE_MAX_INJ = 360; }
            else { CRANK_ANGLE_MAX_INJ = 180; }
          }
          
          //Adjust the injection angles based on the number of squirts
          if (currentStatus.nSquirts > 2)
          {
            channel2InjDegrees = (channel2InjDegrees * 2) / currentStatus.nSquirts;
            channel3InjDegrees = (channel3InjDegrees * 2) / currentStatus.nSquirts;
          }

          if (!configPage2.injTiming) 
          { 
            //For simultaneous, all squirts happen at the same time
            channel1InjDegrees = 0;
            channel2InjDegrees = 0;
            channel3InjDegrees = 0; 
          } 
        }
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          currentStatus.nSquirts = 1;

          if(configPage2.strokes == TWO_STROKE)
          {
            channel1InjDegrees = 0;
            channel2InjDegrees = 120;
            channel3InjDegrees = 240;
            CRANK_ANGLE_MAX_INJ = 360;
          }
          else
          {
            req_fuel_uS = req_fuel_uS * 2;
            channel1InjDegrees = 0;
            channel2InjDegrees = 240;
            channel3InjDegrees = 480;
            CRANK_ANGLE_MAX_INJ = 720;
          }
        }
        else
        {
          //Should never happen, but default values
          channel1InjDegrees = 0;
          channel2InjDegrees = 120;
          channel3InjDegrees = 240;
        }

        //Check if injector staging is enabled
        if(configPage10.stagingEnabled == true)
        {
          #if INJ_CHANNELS >= 6
            maxInjOutputs = 6;

            channel4InjDegrees = channel1InjDegrees;
            channel5InjDegrees = channel2InjDegrees;
            channel6InjDegrees = channel3InjDegrees;
          #else
            //Staged output is on channel 4
            maxInjOutputs = 4;
            channel4InjDegrees = channel1InjDegrees;
          #endif
        }
        break;
    case 4:
        channel1IgnDegrees = 0;
        channel1InjDegrees = 0;
        maxIgnOutputs = 2; //Default value for 4 cylinder, may be changed below
        maxInjOutputs = 2;
        if (configPage2.engineType == EVEN_FIRE )
        {
          channel2IgnDegrees = 180;

          if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage2.strokes == FOUR_STROKE) )
          {
            channel3IgnDegrees = 360;
            channel4IgnDegrees = 540;

            CRANK_ANGLE_MAX_IGN = 720;
            maxIgnOutputs = 4;
          }
          if(configPage4.sparkMode == IGN_MODE_ROTARY)
          {
            //Rotary uses the ign 3 and 4 schedules for the trailing spark. They are offset from the ign 1 and 2 channels respectively and so use the same degrees as them
            channel3IgnDegrees = 0;
            channel4IgnDegrees = 180;
            maxIgnOutputs = 4;

            configPage4.IgInv = GOING_LOW; //Force Going Low ignition mode (Going high is never used for rotary)
          }
        }
        else
        {
          channel2IgnDegrees = configPage2.oddfire2;
          channel3IgnDegrees = configPage2.oddfire3;
          channel4IgnDegrees = configPage2.oddfire4;
          maxIgnOutputs = 4;
        }

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) || (configPage2.strokes == TWO_STROKE) )
        {
          channel2InjDegrees = 180;

          if (!configPage2.injTiming) 
          { 
            //For simultaneous, all squirts happen at the same time
            channel1InjDegrees = 0;
            channel2InjDegrees = 0; 
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            channel2InjDegrees = (channel2InjDegrees * 2) / currentStatus.nSquirts;
          }
          else { } //Do nothing, default values are correct
        }
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          channel2InjDegrees = 180;
          channel3InjDegrees = 360;
          channel4InjDegrees = 540;

          maxInjOutputs = 4;

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }
        else
        {
          //Should never happen, but default values
          maxInjOutputs = 2;
        }

        //Check if injector staging is enabled
        if(configPage10.stagingEnabled == true)
        {
          maxInjOutputs = 4;

          if( (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage2.injLayout == INJ_SEMISEQUENTIAL) )
          {
            //Staging with 4 cylinders semi/sequential requires 8 total channels
            #if INJ_CHANNELS >= 8
              maxInjOutputs = 8;

              channel5InjDegrees = channel1InjDegrees;
              channel6InjDegrees = channel2InjDegrees;
              channel7InjDegrees = channel3InjDegrees;
              channel8InjDegrees = channel4InjDegrees;
            #else
              //This is an invalid config as there are not enough outputs to support sequential + staging
              //Put the staging output to the non-existent channel 5
              #if (INJ_CHANNELS >= 5)
              maxInjOutputs = 5;
              channel5InjDegrees = channel1InjDegrees;
              #endif
            #endif
          }
          else
          {
            channel3InjDegrees = channel1InjDegrees;
            channel4InjDegrees = channel2InjDegrees;
          }
        }

        break;
    case 5:
        channel1IgnDegrees = 0;
        channel2IgnDegrees = 72;
        channel3IgnDegrees = 144;
        channel4IgnDegrees = 216;
#if (IGN_CHANNELS >= 5)
        channel5IgnDegrees = 288;
#endif
        maxIgnOutputs = 5; //Only 4 actual outputs, so that's all that can be cut
        maxInjOutputs = 4; //Is updated below to 5 if there are enough channels

        if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
        {
          channel2IgnDegrees = 144;
          channel3IgnDegrees = 288;
          channel4IgnDegrees = 432;
#if (IGN_CHANNELS >= 5)
          channel5IgnDegrees = 576;
#endif

          CRANK_ANGLE_MAX_IGN = 720;
        }

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) || (configPage2.strokes == TWO_STROKE) )
        {
          if (!configPage2.injTiming) 
          { 
            //For simultaneous, all squirts happen at the same time
            channel1InjDegrees = 0;
            channel2InjDegrees = 0;
            channel3InjDegrees = 0;
            channel4InjDegrees = 0;
#if (INJ_CHANNELS >= 5)
            channel5InjDegrees = 0; 
#endif
          }
          else
          {
            channel1InjDegrees = 0;
            channel2InjDegrees = 72;
            channel3InjDegrees = 144;
            channel4InjDegrees = 216;
#if (INJ_CHANNELS >= 5)
            channel5InjDegrees = 288;
#endif

            //Divide by currentStatus.nSquirts ?
          }
        }
    #if INJ_CHANNELS >= 5
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 144;
          channel3InjDegrees = 288;
          channel4InjDegrees = 432;
          channel5InjDegrees = 576;

          maxInjOutputs = 5;

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }
    #endif

    #if INJ_CHANNELS >= 6
          if(configPage10.stagingEnabled == true) { maxInjOutputs = 6; }
    #endif
        break;
    case 6:
        channel1IgnDegrees = 0;
        channel2IgnDegrees = 120;
        channel3IgnDegrees = 240;
        maxIgnOutputs = 3;
        maxInjOutputs = 3;

    #if IGN_CHANNELS >= 6
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL))
        {
        channel4IgnDegrees = 360;
        channel5IgnDegrees = 480;
        channel6IgnDegrees = 600;
        CRANK_ANGLE_MAX_IGN = 720;
        maxIgnOutputs = 6;
        }
    #endif

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) )
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 120;
          channel3InjDegrees = 240;
          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            channel1InjDegrees = 0;
            channel2InjDegrees = 0;
            channel3InjDegrees = 0;
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            channel2InjDegrees = (channel2InjDegrees * 2) / currentStatus.nSquirts;
            channel3InjDegrees = (channel3InjDegrees * 2) / currentStatus.nSquirts;
          }
        }

    #if INJ_CHANNELS >= 6
        if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 120;
          channel3InjDegrees = 240;
          channel4InjDegrees = 360;
          channel5InjDegrees = 480;
          channel6InjDegrees = 600;

          maxInjOutputs = 6;

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }
        else if(configPage10.stagingEnabled == true) //Check if injector staging is enabled
        {
          maxInjOutputs = 6;

          if( (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage2.injLayout == INJ_SEMISEQUENTIAL) )
          {
            //Staging with 6 cylinders semi/sequential requires 7 total channels
            #if INJ_CHANNELS >= 7
              maxInjOutputs = 7;

              channel5InjDegrees = channel1InjDegrees;
              channel6InjDegrees = channel2InjDegrees;
              channel7InjDegrees = channel3InjDegrees;
              channel8InjDegrees = channel4InjDegrees;
            #else
              //This is an invalid config as there are not enough outputs to support sequential + staging
              //No staging output will be active
              maxInjOutputs = 6;
            #endif
          }
        }
    #endif
        break;
    case 8:
        channel1IgnDegrees = 0;
        channel2IgnDegrees = 90;
        channel3IgnDegrees = 180;
        channel4IgnDegrees = 270;
        maxIgnOutputs = 4;
        maxInjOutputs = 4;


        if( (configPage4.sparkMode == IGN_MODE_SINGLE))
        {
          maxIgnOutputs = 4;
          CRANK_ANGLE_MAX_IGN = 360;
        }
    

    #if IGN_CHANNELS >= 8
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL))
        {
        channel5IgnDegrees = 360;
        channel6IgnDegrees = 450;
        channel7IgnDegrees = 540;
        channel8IgnDegrees = 630;
        maxIgnOutputs = 8;
        CRANK_ANGLE_MAX_IGN = 720;
        }
    #endif

        //For alternating injection, the squirt occurs at different times for each channel
        if( (configPage2.injLayout == INJ_SEMISEQUENTIAL) || (configPage2.injLayout == INJ_PAIRED) )
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 90;
          channel3InjDegrees = 180;
          channel4InjDegrees = 270;

          if (!configPage2.injTiming)
          {
            //For simultaneous, all squirts happen at the same time
            channel1InjDegrees = 0;
            channel2InjDegrees = 0;
            channel3InjDegrees = 0;
            channel4InjDegrees = 0;
          }
          else if (currentStatus.nSquirts > 2)
          {
            //Adjust the injection angles based on the number of squirts
            channel2InjDegrees = (channel2InjDegrees * 2) / currentStatus.nSquirts;
            channel3InjDegrees = (channel3InjDegrees * 2) / currentStatus.nSquirts;
            channel4InjDegrees = (channel4InjDegrees * 2) / currentStatus.nSquirts;
          }
        }

    #if INJ_CHANNELS >= 8
        else if (configPage2.injLayout == INJ_SEQUENTIAL)
        {
          channel1InjDegrees = 0;
          channel2InjDegrees = 90;
          channel3InjDegrees = 180;
          channel4InjDegrees = 270;
          channel5InjDegrees = 360;
          channel6InjDegrees = 450;
          channel7InjDegrees = 540;
          channel8InjDegrees = 630;

          maxInjOutputs = 8;

          CRANK_ANGLE_MAX_INJ = 720;
          currentStatus.nSquirts = 1;
          req_fuel_uS = req_fuel_uS * 2;
        }
    #endif

        break;
    default: //Handle this better!!!
        channel1InjDegrees = 0;
        channel2InjDegrees = 180;
        break;
    }

    currentStatus.status3 |= currentStatus.nSquirts << BIT_STATUS3_NSQUIRTS1; //Top 3 bits of the status3 variable are the number of squirts. This must be done after the above section due to nSquirts being forced to 1 for sequential
    
    //Special case:
    //3 or 5 squirts per cycle MUST be tracked over 720 degrees. This is because the angles for them (Eg 720/3=240) are not evenly divisible into 360
    //This is ONLY the case on 4 stroke systems
    if( (currentStatus.nSquirts == 3) || (currentStatus.nSquirts == 5) )
    {
      if(configPage2.strokes == FOUR_STROKE) { CRANK_ANGLE_MAX_INJ = (720U / currentStatus.nSquirts); }
    }
    
    switch(configPage2.injLayout)
    {
    case INJ_PAIRED:
        //Paired injection
        fuelSchedule1.pStartFunction = openInjector1;
        fuelSchedule1.pEndFunction = closeInjector1;
        fuelSchedule2.pStartFunction = openInjector2;
        fuelSchedule2.pEndFunction = closeInjector2;
        fuelSchedule3.pStartFunction = openInjector3;
        fuelSchedule3.pEndFunction = closeInjector3;
        fuelSchedule4.pStartFunction = openInjector4;
        fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
        fuelSchedule5.pStartFunction = openInjector5;
        fuelSchedule5.pEndFunction = closeInjector5;
#endif
        break;

    case INJ_SEMISEQUENTIAL:
        //Semi-Sequential injection. Currently possible with 4, 6 and 8 cylinders. 5 cylinder is a special case
        if( configPage2.nCylinders == 4 )
        {
          if(configPage4.inj4cylPairing == INJ_PAIR_13_24)
          {
            fuelSchedule1.pStartFunction = openInjector1and3;
            fuelSchedule1.pEndFunction = closeInjector1and3;
            fuelSchedule2.pStartFunction = openInjector2and4;
            fuelSchedule2.pEndFunction = closeInjector2and4;
          }
          else
          {
            fuelSchedule1.pStartFunction = openInjector1and4;
            fuelSchedule1.pEndFunction = closeInjector1and4;
            fuelSchedule2.pStartFunction = openInjector2and3;
            fuelSchedule2.pEndFunction = closeInjector2and3;
          }
        }
        else if( configPage2.nCylinders == 5 ) //This is similar to the paired injection but uses five injector outputs instead of four
        {
          fuelSchedule1.pStartFunction = openInjector1;
          fuelSchedule1.pEndFunction = closeInjector1;
          fuelSchedule2.pStartFunction = openInjector2;
          fuelSchedule2.pEndFunction = closeInjector2;
          fuelSchedule3.pStartFunction = openInjector3and5;
          fuelSchedule3.pEndFunction = closeInjector3and5;
          fuelSchedule4.pStartFunction = openInjector4;
          fuelSchedule4.pEndFunction = closeInjector4;
        }
        else if( configPage2.nCylinders == 6 )
        {
          fuelSchedule1.pStartFunction = openInjector1and4;
          fuelSchedule1.pEndFunction = closeInjector1and4;
          fuelSchedule2.pStartFunction = openInjector2and5;
          fuelSchedule2.pEndFunction = closeInjector2and5;
          fuelSchedule3.pStartFunction = openInjector3and6;
          fuelSchedule3.pEndFunction = closeInjector3and6;
        }
        else if( configPage2.nCylinders == 8 )
        {
          fuelSchedule1.pStartFunction = openInjector1and5;
          fuelSchedule1.pEndFunction = closeInjector1and5;
          fuelSchedule2.pStartFunction = openInjector2and6;
          fuelSchedule2.pEndFunction = closeInjector2and6;
          fuelSchedule3.pStartFunction = openInjector3and7;
          fuelSchedule3.pEndFunction = closeInjector3and7;
          fuelSchedule4.pStartFunction = openInjector4and8;
          fuelSchedule4.pEndFunction = closeInjector4and8;
        }
        else
        {
          //Fall back to paired injection
          fuelSchedule1.pStartFunction = openInjector1;
          fuelSchedule1.pEndFunction = closeInjector1;
          fuelSchedule2.pStartFunction = openInjector2;
          fuelSchedule2.pEndFunction = closeInjector2;
          fuelSchedule3.pStartFunction = openInjector3;
          fuelSchedule3.pEndFunction = closeInjector3;
          fuelSchedule4.pStartFunction = openInjector4;
          fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
          fuelSchedule5.pStartFunction = openInjector5;
          fuelSchedule5.pEndFunction = closeInjector5;
#endif
        }
        break;

    case INJ_SEQUENTIAL:
        //Sequential injection
        fuelSchedule1.pStartFunction = openInjector1;
        fuelSchedule1.pEndFunction = closeInjector1;
        fuelSchedule2.pStartFunction = openInjector2;
        fuelSchedule2.pEndFunction = closeInjector2;
        fuelSchedule3.pStartFunction = openInjector3;
        fuelSchedule3.pEndFunction = closeInjector3;
        fuelSchedule4.pStartFunction = openInjector4;
        fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
        fuelSchedule5.pStartFunction = openInjector5;
        fuelSchedule5.pEndFunction = closeInjector5;
#endif
#if INJ_CHANNELS >= 6
        fuelSchedule6.pStartFunction = openInjector6;
        fuelSchedule6.pEndFunction = closeInjector6;
#endif
#if INJ_CHANNELS >= 7
        fuelSchedule7.pStartFunction = openInjector7;
        fuelSchedule7.pEndFunction = closeInjector7;
#endif
#if INJ_CHANNELS >= 8
        fuelSchedule8.pStartFunction = openInjector8;
        fuelSchedule8.pEndFunction = closeInjector8;
#endif
        break;

    default:
        //Paired injection
        fuelSchedule1.pStartFunction = openInjector1;
        fuelSchedule1.pEndFunction = closeInjector1;
        fuelSchedule2.pStartFunction = openInjector2;
        fuelSchedule2.pEndFunction = closeInjector2;
        fuelSchedule3.pStartFunction = openInjector3;
        fuelSchedule3.pEndFunction = closeInjector3;
        fuelSchedule4.pStartFunction = openInjector4;
        fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
        fuelSchedule5.pStartFunction = openInjector5;
        fuelSchedule5.pEndFunction = closeInjector5;
#endif
        break;
    }

    switch(configPage4.sparkMode)
    {
    case IGN_MODE_WASTED:
        //Wasted Spark (Normal mode)
        ignitionSchedule1.pStartCallback = beginCoil1Charge;
        ignitionSchedule1.pEndCallback = endCoil1Charge;
        ignitionSchedule2.pStartCallback = beginCoil2Charge;
        ignitionSchedule2.pEndCallback = endCoil2Charge;
        ignitionSchedule3.pStartCallback = beginCoil3Charge;
        ignitionSchedule3.pEndCallback = endCoil3Charge;
        ignitionSchedule4.pStartCallback = beginCoil4Charge;
        ignitionSchedule4.pEndCallback = endCoil4Charge;
        ignitionSchedule5.pStartCallback = beginCoil5Charge;
        ignitionSchedule5.pEndCallback = endCoil5Charge;
        break;

    case IGN_MODE_SINGLE:
        //Single channel mode. All ignition pulses are on channel 1
        ignitionSchedule1.pStartCallback = beginCoil1Charge;
        ignitionSchedule1.pEndCallback = endCoil1Charge;
        ignitionSchedule2.pStartCallback = beginCoil1Charge;
        ignitionSchedule2.pEndCallback = endCoil1Charge;
        ignitionSchedule3.pStartCallback = beginCoil1Charge;
        ignitionSchedule3.pEndCallback = endCoil1Charge;
        ignitionSchedule4.pStartCallback = beginCoil1Charge;
        ignitionSchedule4.pEndCallback = endCoil1Charge;
#if IGN_CHANNELS >= 5
        ignitionSchedule5.pStartCallback = beginCoil1Charge;
        ignitionSchedule5.pEndCallback = endCoil1Charge;
#endif
#if IGN_CHANNELS >= 6
        ignitionSchedule6.pStartCallback = beginCoil1Charge;
        ignitionSchedule6.pEndCallback = endCoil1Charge;
#endif
#if IGN_CHANNELS >= 7
        ignitionSchedule7.pStartCallback = beginCoil1Charge;
        ignitionSchedule7.pEndCallback = endCoil1Charge;
#endif
#if IGN_CHANNELS >= 8
        ignitionSchedule8.pStartCallback = beginCoil1Charge;
        ignitionSchedule8.pEndCallback = endCoil1Charge;
#endif
        break;

    case IGN_MODE_WASTEDCOP:
        //Wasted COP mode. Note, most of the boards can only run this for 4-cyl only.
        if( configPage2.nCylinders <= 3)
        {
          //1-3 cylinder wasted COP is the same as regular wasted mode
          ignitionSchedule1.pStartCallback = beginCoil1Charge;
          ignitionSchedule1.pEndCallback = endCoil1Charge;
          ignitionSchedule2.pStartCallback = beginCoil2Charge;
          ignitionSchedule2.pEndCallback = endCoil2Charge;
          ignitionSchedule3.pStartCallback = beginCoil3Charge;
          ignitionSchedule3.pEndCallback = endCoil3Charge;
        }
        else if( configPage2.nCylinders == 4 )
        {
          //Wasted COP mode for 4 cylinders. Ignition channels 1&3 and 2&4 are paired together
          ignitionSchedule1.pStartCallback = beginCoil1and3Charge;
          ignitionSchedule1.pEndCallback = endCoil1and3Charge;
          ignitionSchedule2.pStartCallback = beginCoil2and4Charge;
          ignitionSchedule2.pEndCallback = endCoil2and4Charge;

          ignitionSchedule3.pStartCallback = nullCallback;
          ignitionSchedule3.pEndCallback = nullCallback;
          ignitionSchedule4.pStartCallback = nullCallback;
          ignitionSchedule4.pEndCallback = nullCallback;
        }
        else if( configPage2.nCylinders == 6 )
        {
          //Wasted COP mode for 6 cylinders. Ignition channels 1&4, 2&5 and 3&6 are paired together
          ignitionSchedule1.pStartCallback = beginCoil1and4Charge;
          ignitionSchedule1.pEndCallback = endCoil1and4Charge;
          ignitionSchedule2.pStartCallback = beginCoil2and5Charge;
          ignitionSchedule2.pEndCallback = endCoil2and5Charge;
          ignitionSchedule3.pStartCallback = beginCoil3and6Charge;
          ignitionSchedule3.pEndCallback = endCoil3and6Charge;

          ignitionSchedule4.pStartCallback = nullCallback;
          ignitionSchedule4.pEndCallback = nullCallback;
          ignitionSchedule5.pStartCallback = nullCallback;
          ignitionSchedule5.pEndCallback = nullCallback;
#if IGN_CHANNELS >= 6
          ignitionSchedule6.pStartCallback = nullCallback;
          ignitionSchedule6.pEndCallback = nullCallback;
#endif
        }
        else if( configPage2.nCylinders == 8 )
        {
          //Wasted COP mode for 8 cylinders. Ignition channels 1&5, 2&6, 3&7 and 4&8 are paired together
          ignitionSchedule1.pStartCallback = beginCoil1and5Charge;
          ignitionSchedule1.pEndCallback = endCoil1and5Charge;
          ignitionSchedule2.pStartCallback = beginCoil2and6Charge;
          ignitionSchedule2.pEndCallback = endCoil2and6Charge;
          ignitionSchedule3.pStartCallback = beginCoil3and7Charge;
          ignitionSchedule3.pEndCallback = endCoil3and7Charge;
          ignitionSchedule4.pStartCallback = beginCoil4and8Charge;
          ignitionSchedule4.pEndCallback = endCoil4and8Charge;

          ignitionSchedule5.pStartCallback = nullCallback;
          ignitionSchedule5.pEndCallback = nullCallback;
#if IGN_CHANNELS >= 6
          ignitionSchedule6.pStartCallback = nullCallback;
          ignitionSchedule6.pEndCallback = nullCallback;
#endif
#if IGN_CHANNELS >= 7
          ignitionSchedule7.pStartCallback = nullCallback;
          ignitionSchedule7.pEndCallback = nullCallback;
#endif
#if IGN_CHANNELS >= 8
          ignitionSchedule8.pStartCallback = nullCallback;
          ignitionSchedule8.pEndCallback = nullCallback;
#endif
        }
        else
        {
          //If the person has inadvertently selected this when running more than 4 cylinders or other than 6 cylinders, just use standard Wasted spark mode
          ignitionSchedule1.pStartCallback = beginCoil1Charge;
          ignitionSchedule1.pEndCallback = endCoil1Charge;
          ignitionSchedule2.pStartCallback = beginCoil2Charge;
          ignitionSchedule2.pEndCallback = endCoil2Charge;
          ignitionSchedule3.pStartCallback = beginCoil3Charge;
          ignitionSchedule3.pEndCallback = endCoil3Charge;
          ignitionSchedule4.pStartCallback = beginCoil4Charge;
          ignitionSchedule4.pEndCallback = endCoil4Charge;
          ignitionSchedule5.pStartCallback = beginCoil5Charge;
          ignitionSchedule5.pEndCallback = endCoil5Charge;
        }
        break;

    case IGN_MODE_SEQUENTIAL:
        ignitionSchedule1.pStartCallback = beginCoil1Charge;
        ignitionSchedule1.pEndCallback = endCoil1Charge;
        ignitionSchedule2.pStartCallback = beginCoil2Charge;
        ignitionSchedule2.pEndCallback = endCoil2Charge;
        ignitionSchedule3.pStartCallback = beginCoil3Charge;
        ignitionSchedule3.pEndCallback = endCoil3Charge;
        ignitionSchedule4.pStartCallback = beginCoil4Charge;
        ignitionSchedule4.pEndCallback = endCoil4Charge;
        ignitionSchedule5.pStartCallback = beginCoil5Charge;
        ignitionSchedule5.pEndCallback = endCoil5Charge;
#if IGN_CHANNELS >= 6
        ignitionSchedule6.pStartCallback = beginCoil6Charge;
        ignitionSchedule6.pEndCallback = endCoil6Charge;
#endif
#if IGN_CHANNELS >= 7
        ignitionSchedule7.pStartCallback = beginCoil7Charge;
        ignitionSchedule7.pEndCallback = endCoil7Charge;
#endif
#if IGN_CHANNELS >= 8
        ignitionSchedule8.pStartCallback = beginCoil8Charge;
        ignitionSchedule8.pEndCallback = endCoil8Charge;
#endif
        break;

    case IGN_MODE_ROTARY:
        if(configPage10.rotaryType == ROTARY_IGN_FC)
        {
          //Ignition channel 1 is a wasted spark signal for leading signal on both rotors
          ignitionSchedule1.pStartCallback = beginCoil1Charge;
          ignitionSchedule1.pEndCallback = endCoil1Charge;
          ignitionSchedule2.pStartCallback = beginCoil1Charge;
          ignitionSchedule2.pEndCallback = endCoil1Charge;

          ignitionSchedule3.pStartCallback = beginTrailingCoilCharge;
          ignitionSchedule3.pEndCallback = endTrailingCoilCharge1;
          ignitionSchedule4.pStartCallback = beginTrailingCoilCharge;
          ignitionSchedule4.pEndCallback = endTrailingCoilCharge2;
        }
        else if(configPage10.rotaryType == ROTARY_IGN_FD)
        {
          //Ignition channel 1 is a wasted spark signal for leading signal on both rotors
          ignitionSchedule1.pStartCallback = beginCoil1Charge;
          ignitionSchedule1.pEndCallback = endCoil1Charge;
          ignitionSchedule2.pStartCallback = beginCoil1Charge;
          ignitionSchedule2.pEndCallback = endCoil1Charge;

          //Trailing coils have their own channel each
          //IGN2 = front rotor trailing spark
          ignitionSchedule3.pStartCallback = beginCoil2Charge;
          ignitionSchedule3.pEndCallback = endCoil2Charge;
          //IGN3 = rear rotor trailing spark
          ignitionSchedule4.pStartCallback = beginCoil3Charge;
          ignitionSchedule4.pEndCallback = endCoil3Charge;

          //IGN4 not used
        }
        else if(configPage10.rotaryType == ROTARY_IGN_RX8)
        {
          //RX8 outputs are simply 1 coil and 1 output per plug

          //IGN1 is front rotor, leading spark
          ignitionSchedule1.pStartCallback = beginCoil1Charge;
          ignitionSchedule1.pEndCallback = endCoil1Charge;
          //IGN2 is rear rotor, leading spark
          ignitionSchedule2.pStartCallback = beginCoil2Charge;
          ignitionSchedule2.pEndCallback = endCoil2Charge;
          //IGN3 = front rotor trailing spark
          ignitionSchedule3.pStartCallback = beginCoil3Charge;
          ignitionSchedule3.pEndCallback = endCoil3Charge;
          //IGN4 = rear rotor trailing spark
          ignitionSchedule4.pStartCallback = beginCoil4Charge;
          ignitionSchedule4.pEndCallback = endCoil4Charge;
        }
        else { } //No action for other RX ignition modes (Future expansion / MISRA compliant). 
        break;

    default:
        //Wasted spark (Shouldn't ever happen anyway)
        ignitionSchedule1.pStartCallback = beginCoil1Charge;
        ignitionSchedule1.pEndCallback = endCoil1Charge;
        ignitionSchedule2.pStartCallback = beginCoil2Charge;
        ignitionSchedule2.pEndCallback = endCoil2Charge;
        ignitionSchedule3.pStartCallback = beginCoil3Charge;
        ignitionSchedule3.pEndCallback = endCoil3Charge;
        ignitionSchedule4.pStartCallback = beginCoil4Charge;
        ignitionSchedule4.pEndCallback = endCoil4Charge;
        ignitionSchedule5.pStartCallback = beginCoil5Charge;
        ignitionSchedule5.pEndCallback = endCoil5Charge;
        break;
    }

    //Begin priming the fuel pump. This is turned off in the low resolution, 1s interrupt in timers.ino
    //First check that the priming time is not 0
    if(configPage2.fpPrime > 0)
    {
      FUEL_PUMP_ON();
      currentStatus.fuelPumpOn = true;
    }
    else { currentStatus.fpPrimed = true; } //If the user has set 0 for the pump priming, immediately mark the priming as being completed

    interrupts();
    readCLT(false); // Need to read coolant temp to make priming pulsewidth work correctly. The false here disables use of the filter
    readTPS(false); // Need to read tps to detect flood clear state

    /* tacho sweep function. */
    currentStatus.tachoSweepEnabled = (configPage2.useTachoSweep > 0);
    /* SweepMax is stored as a byte, RPM/100. divide by 60 to convert min to sec (net 5/3).  Multiply by ignition pulses per rev.
       tachoSweepIncr is also the number of tach pulses per second */
    tachoSweepIncr = configPage2.tachoSweepMaxRPM * maxIgnOutputs * 5 / 3;
    
    currentStatus.initialisationComplete = true;
    digitalWrite(LED_BUILTIN, HIGH);

}
/** Set board / microcontroller specific pin mappings / assignments.
 * The boardID is switch-case compared against raw boardID integers (not enum or defined label, and probably no need for that either)
 * which are originated from tuning SW (e.g. TS) set values and are available in reference/speeduino.ini (See pinLayout, note also that
 * numbering is not contiguous here).
 */
void setPinMapping(byte boardID)
{
  //Force set defaults. Will be overwritten below if needed.
  injectorOutputControl = OUTPUT_CONTROL_DIRECT;
  ignitionOutputControl = OUTPUT_CONTROL_DIRECT;

  switch (boardID)
  {
      // Pin definitions for experimental board uEFI v3.4
      // https://github.com/Churrosoft/OpenEFI-PCB/tree/main/uEFI_rev3
      case 1:
        pinInjector1 = PNUM_NOT_DEFINED; //Output pin injector 1 is on
        pinInjector2 = PNUM_NOT_DEFINED; //Output pin injector 2 is on
        pinInjector3 = PNUM_NOT_DEFINED; //Output pin injector 3 is on
        pinInjector4 = PNUM_NOT_DEFINED; //Output pin injector 4 is on

        pinCoil1 = PNUM_NOT_DEFINED; //Pin for coil 1
        pinCoil2 = PNUM_NOT_DEFINED; //Pin for coil 2
        pinCoil3 = PNUM_NOT_DEFINED; //Pin for coil 3
        pinCoil4 = PNUM_NOT_DEFINED; //Pin for coil 4

        injectorOutputControl = OUTPUT_CONTROL_MC33810;
        ignitionOutputControl = OUTPUT_CONTROL_MC33810;
        pinMC33810_1_CS = PB11; // move to define? like memory?

        pinTrigger = PC6; // CKP
        pinTrigger2 = PC7; // CMP

        // Sensors:
        pinIAT = PA0; //ADC123
        pinTPS = PA1; //ADC123
        pinMAP = PA2; //ADC123
        pinCLT = PA3; //ADC123
        pinO2 = PA4; //ADC12
        pinBat = PA5;  //ADC12
        pinFuelPressure = PNUM_NOT_DEFINED;
        pinOilPressure = PNUM_NOT_DEFINED;        
        pinFlex = PNUM_NOT_DEFINED; // Flex sensor (Must be external interrupt enabled)
        pinBaro = PNUM_NOT_DEFINED; // SPI BARO on OpenEFI v4

       
       // Outputs:
        pinTachOut = PE11; //
        pinBoost = PA6; //
        pinIdle1 = PA7; //
        pinVVT_1 = 4; //Default VVT output
        pinFuelPump = 45; //Fuel pump output  (Goes to ULN2803)
        pinFan = 47; //Pin for the fan output (Goes to ULN2803)
        pinLaunch = 51; //Can be overwritten below

        pinStepperDir = 16; //Direction pin  for DRV8825 driver
        pinStepperStep = 17; //Step pin for DRV8825 driver
        pinStepperEnable = 24; //Enable pin for DRV8825
  }

  //Setup any devices that are using selectable pins

  if ( (configPage6.launchPin != 0) && (configPage6.launchPin < BOARD_MAX_IO_PINS) ) { pinLaunch = pinTranslate(configPage6.launchPin); }
  if ( (configPage4.ignBypassPin != 0) && (configPage4.ignBypassPin < BOARD_MAX_IO_PINS) ) { pinIgnBypass = pinTranslate(configPage4.ignBypassPin); }
  if ( (configPage2.tachoPin != 0) && (configPage2.tachoPin < BOARD_MAX_IO_PINS) ) { pinTachOut = pinTranslate(configPage2.tachoPin); }
  if ( (configPage4.fuelPumpPin != 0) && (configPage4.fuelPumpPin < BOARD_MAX_IO_PINS) ) { pinFuelPump = pinTranslate(configPage4.fuelPumpPin); }
  if ( (configPage6.fanPin != 0) && (configPage6.fanPin < BOARD_MAX_IO_PINS) ) { pinFan = pinTranslate(configPage6.fanPin); }
  if ( (configPage6.boostPin != 0) && (configPage6.boostPin < BOARD_MAX_IO_PINS) ) { pinBoost = pinTranslate(configPage6.boostPin); }
  if ( (configPage6.vvt1Pin != 0) && (configPage6.vvt1Pin < BOARD_MAX_IO_PINS) ) { pinVVT_1 = pinTranslate(configPage6.vvt1Pin); }
  if ( (configPage6.useExtBaro != 0) && (configPage6.baroPin < BOARD_MAX_IO_PINS) ) { pinBaro = pinTranslateAnalog(configPage6.baroPin); }
  if ( (configPage6.useEMAP != 0) && (configPage10.EMAPPin < BOARD_MAX_IO_PINS) ) { pinEMAP = pinTranslateAnalog(configPage10.EMAPPin); }
  if ( (configPage10.fuel2InputPin != 0) && (configPage10.fuel2InputPin < BOARD_MAX_IO_PINS) ) { pinFuel2Input = pinTranslate(configPage10.fuel2InputPin); }
  if ( (configPage10.spark2InputPin != 0) && (configPage10.spark2InputPin < BOARD_MAX_IO_PINS) ) { pinSpark2Input = pinTranslate(configPage10.spark2InputPin); }
  if ( (configPage2.vssPin != 0) && (configPage2.vssPin < BOARD_MAX_IO_PINS) ) { pinVSS = pinTranslate(configPage2.vssPin); }
  if ( (configPage10.fuelPressureEnable) && (configPage10.fuelPressurePin < BOARD_MAX_IO_PINS) ) { pinFuelPressure = pinTranslateAnalog(configPage10.fuelPressurePin); }
  if ( (configPage10.oilPressureEnable) && (configPage10.oilPressurePin < BOARD_MAX_IO_PINS) ) { pinOilPressure = pinTranslateAnalog(configPage10.oilPressurePin); }
  
  if ( (configPage10.wmiEmptyPin != 0) && (configPage10.wmiEmptyPin < BOARD_MAX_IO_PINS) ) { pinWMIEmpty = pinTranslate(configPage10.wmiEmptyPin); }
  if ( (configPage10.wmiIndicatorPin != 0) && (configPage10.wmiIndicatorPin < BOARD_MAX_IO_PINS) ) { pinWMIIndicator = pinTranslate(configPage10.wmiIndicatorPin); }
  if ( (configPage10.wmiEnabledPin != 0) && (configPage10.wmiEnabledPin < BOARD_MAX_IO_PINS) ) { pinWMIEnabled = pinTranslate(configPage10.wmiEnabledPin); }
  if ( (configPage10.vvt2Pin != 0) && (configPage10.vvt2Pin < BOARD_MAX_IO_PINS) ) { pinVVT_2 = pinTranslate(configPage10.vvt2Pin); }
  if ( (configPage13.onboard_log_trigger_Epin != 0 ) && (configPage13.onboard_log_trigger_Epin != 0) && (configPage13.onboard_log_tr5_Epin_pin < BOARD_MAX_IO_PINS) ) { pinSDEnable = pinTranslate(configPage13.onboard_log_tr5_Epin_pin); }
  

  //Currently there's no default pin for Idle Up
  
  pinIdleUp = pinTranslate(configPage2.idleUpPin);

  //Currently there's no default pin for Idle Up Output
  pinIdleUpOutput = pinTranslate(configPage2.idleUpOutputPin);

  //Currently there's no default pin for closed throttle position sensor
  pinCTPS = pinTranslate(configPage2.CTPSPin);
  
  // Air conditioning control initialisation
  if ((configPage15.airConCompPin != 0) && (configPage15.airConCompPin < BOARD_MAX_IO_PINS) ) { pinAirConComp = pinTranslate(configPage15.airConCompPin); }
  if ((configPage15.airConFanPin != 0) && (configPage15.airConFanPin < BOARD_MAX_IO_PINS) ) { pinAirConFan = pinTranslate(configPage15.airConFanPin); }
  if ((configPage15.airConReqPin != 0) && (configPage15.airConReqPin < BOARD_MAX_IO_PINS) ) { pinAirConRequest = pinTranslate(configPage15.airConReqPin); }
    
  /* Reset control is a special case. If reset control is enabled, it needs its initial state set BEFORE its pinMode.
     If that doesn't happen and reset control is in "Serial Command" mode, the Arduino will end up in a reset loop
     because the control pin will go low as soon as the pinMode is set to OUTPUT. */
  if ( (configPage4.resetControlConfig != 0) && (configPage4.resetControlPin < BOARD_MAX_IO_PINS) )
  {
    if (configPage4.resetControlPin!=0U) {
      pinResetControl = pinTranslate(configPage4.resetControlPin);
    }
    resetControl = configPage4.resetControlConfig;
    setResetControlPinState();
    pinMode(pinResetControl, OUTPUT);
  }
  

  //Finally, set the relevant pin modes for outputs
  pinMode(pinTachOut, OUTPUT);
  pinMode(pinIdle1, OUTPUT);
  pinMode(pinIdle2, OUTPUT);
  pinMode(pinIdleUpOutput, OUTPUT);
  pinMode(pinFuelPump, OUTPUT);
  pinMode(pinFan, OUTPUT);
  pinMode(pinStepperDir, OUTPUT);
  pinMode(pinStepperStep, OUTPUT);
  pinMode(pinStepperEnable, OUTPUT);
  pinMode(pinBoost, OUTPUT);
  pinMode(pinVVT_1, OUTPUT);
  pinMode(pinVVT_2, OUTPUT);
  if(configPage4.ignBypassEnabled > 0) { pinMode(pinIgnBypass, OUTPUT); }

  //This is a legacy mode option to revert the MAP reading behaviour to match what was in place prior to the 201905 firmware
  if(configPage2.legacyMAP > 0) { digitalWrite(pinMAP, HIGH); }

  if(ignitionOutputControl == OUTPUT_CONTROL_DIRECT)
  {
    pinMode(pinCoil1, OUTPUT);
    pinMode(pinCoil2, OUTPUT);
    pinMode(pinCoil3, OUTPUT);
    pinMode(pinCoil4, OUTPUT);
    #if (IGN_CHANNELS >= 5)
    pinMode(pinCoil5, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 6)
    pinMode(pinCoil6, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 7)
    pinMode(pinCoil7, OUTPUT);
    #endif
    #if (IGN_CHANNELS >= 8)
    pinMode(pinCoil8, OUTPUT);
    #endif

    ign1_pin_port = portOutputRegister(digitalPinToPort(pinCoil1));
    ign1_pin_mask = digitalPinToBitMask(pinCoil1);
    ign2_pin_port = portOutputRegister(digitalPinToPort(pinCoil2));
    ign2_pin_mask = digitalPinToBitMask(pinCoil2);
    ign3_pin_port = portOutputRegister(digitalPinToPort(pinCoil3));
    ign3_pin_mask = digitalPinToBitMask(pinCoil3);
    ign4_pin_port = portOutputRegister(digitalPinToPort(pinCoil4));
    ign4_pin_mask = digitalPinToBitMask(pinCoil4);
    ign5_pin_port = portOutputRegister(digitalPinToPort(pinCoil5));
    ign5_pin_mask = digitalPinToBitMask(pinCoil5);
    ign6_pin_port = portOutputRegister(digitalPinToPort(pinCoil6));
    ign6_pin_mask = digitalPinToBitMask(pinCoil6);
    ign7_pin_port = portOutputRegister(digitalPinToPort(pinCoil7));
    ign7_pin_mask = digitalPinToBitMask(pinCoil7);
    ign8_pin_port = portOutputRegister(digitalPinToPort(pinCoil8));
    ign8_pin_mask = digitalPinToBitMask(pinCoil8);
  } 

  if(injectorOutputControl == OUTPUT_CONTROL_DIRECT)
  {
    pinMode(pinInjector1, OUTPUT);
    pinMode(pinInjector2, OUTPUT);
    pinMode(pinInjector3, OUTPUT);
    pinMode(pinInjector4, OUTPUT);
    #if (INJ_CHANNELS >= 5)
    pinMode(pinInjector5, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 6)
    pinMode(pinInjector6, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 7)
    pinMode(pinInjector7, OUTPUT);
    #endif
    #if (INJ_CHANNELS >= 8)
    pinMode(pinInjector8, OUTPUT);
    #endif

    inj1_pin_port = portOutputRegister(digitalPinToPort(pinInjector1));
    inj1_pin_mask = digitalPinToBitMask(pinInjector1);
    inj2_pin_port = portOutputRegister(digitalPinToPort(pinInjector2));
    inj2_pin_mask = digitalPinToBitMask(pinInjector2);
    inj3_pin_port = portOutputRegister(digitalPinToPort(pinInjector3));
    inj3_pin_mask = digitalPinToBitMask(pinInjector3);
    inj4_pin_port = portOutputRegister(digitalPinToPort(pinInjector4));
    inj4_pin_mask = digitalPinToBitMask(pinInjector4);
    inj5_pin_port = portOutputRegister(digitalPinToPort(pinInjector5));
    inj5_pin_mask = digitalPinToBitMask(pinInjector5);
    inj6_pin_port = portOutputRegister(digitalPinToPort(pinInjector6));
    inj6_pin_mask = digitalPinToBitMask(pinInjector6);
    inj7_pin_port = portOutputRegister(digitalPinToPort(pinInjector7));
    inj7_pin_mask = digitalPinToBitMask(pinInjector7);
    inj8_pin_port = portOutputRegister(digitalPinToPort(pinInjector8));
    inj8_pin_mask = digitalPinToBitMask(pinInjector8);
  }
  
  if( (ignitionOutputControl == OUTPUT_CONTROL_MC33810) || (injectorOutputControl == OUTPUT_CONTROL_MC33810) )
  {
    initMC33810();
    if( (LED_BUILTIN != SCK) && (LED_BUILTIN != MOSI) && (LED_BUILTIN != MISO) ) pinMode(LED_BUILTIN, OUTPUT); //This is required on as the LED pin can otherwise be reset to an input
  }

//CS pin number is now set in a compile flag. 
// #ifdef USE_SPI_EEPROM
//   //We need to send the flash CS (SS) pin if we're using SPI flash. It cannot read from globals.
//   EEPROM.begin(USE_SPI_EEPROM);
// #endif

  tach_pin_port = portOutputRegister(digitalPinToPort(pinTachOut));
  tach_pin_mask = digitalPinToBitMask(pinTachOut);
  pump_pin_port = portOutputRegister(digitalPinToPort(pinFuelPump));
  pump_pin_mask = digitalPinToBitMask(pinFuelPump);

  //And for inputs
  #if defined(CORE_STM32)
    #ifdef INPUT_ANALOG
      pinMode(pinMAP, INPUT_ANALOG);
      pinMode(pinO2, INPUT_ANALOG);
      pinMode(pinO2_2, INPUT_ANALOG);
      pinMode(pinTPS, INPUT_ANALOG);
      pinMode(pinIAT, INPUT_ANALOG);
      pinMode(pinCLT, INPUT_ANALOG);
      pinMode(pinBat, INPUT_ANALOG);
      pinMode(pinBaro, INPUT_ANALOG);
    #else
      pinMode(pinMAP, INPUT);
      pinMode(pinO2, INPUT);
      pinMode(pinO2_2, INPUT);
      pinMode(pinTPS, INPUT);
      pinMode(pinIAT, INPUT);
      pinMode(pinCLT, INPUT);
      pinMode(pinBat, INPUT);
      pinMode(pinBaro, INPUT);
    #endif
  #elif defined(CORE_TEENSY41)
    //Teensy 4.1 has a weak pull down resistor that needs to be disabled for all analog pins. 
    pinMode(pinMAP, INPUT_DISABLE);
    pinMode(pinO2, INPUT_DISABLE);
    pinMode(pinO2_2, INPUT_DISABLE);
    pinMode(pinTPS, INPUT_DISABLE);
    pinMode(pinIAT, INPUT_DISABLE);
    pinMode(pinCLT, INPUT_DISABLE);
    pinMode(pinBat, INPUT_DISABLE);
    pinMode(pinBaro, INPUT_DISABLE);
  #endif

  //Each of the below are only set when their relevant function is enabled. This can help prevent pin conflicts that users aren't aware of with unused functions
  if( (configPage2.flexEnabled > 0) && (!pinIsOutput(pinFlex)) )
  {
    pinMode(pinFlex, INPUT); //Standard GM / Continental flex sensor requires pullup, but this should be onboard. The internal pullup will not work (Requires ~3.3k)!
  }
  if( (configPage2.vssMode > 1) && (!pinIsOutput(pinVSS)) ) //Pin mode 1 for VSS is CAN
  {
    pinMode(pinVSS, INPUT);
  }
  if( (configPage6.launchEnabled > 0) && (!pinIsOutput(pinLaunch)) )
  {
    if (configPage6.lnchPullRes == true) { pinMode(pinLaunch, INPUT_PULLUP); }
    else { pinMode(pinLaunch, INPUT); } //If Launch Pull Resistor is not set make input float.
  }
  if( (configPage2.idleUpEnabled > 0) && (!pinIsOutput(pinIdleUp)) )
  {
    if (configPage2.idleUpPolarity == 0) { pinMode(pinIdleUp, INPUT_PULLUP); } //Normal setting
    else { pinMode(pinIdleUp, INPUT); } //inverted setting
  }
  if( (configPage2.CTPSEnabled > 0) && (!pinIsOutput(pinCTPS)) )
  {
    if (configPage2.CTPSPolarity == 0) { pinMode(pinCTPS, INPUT_PULLUP); } //Normal setting
    else { pinMode(pinCTPS, INPUT); } //inverted setting
  }
  if( (configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH) && (!pinIsOutput(pinFuel2Input)) )
  {
    if (configPage10.fuel2InputPullup == true) { pinMode(pinFuel2Input, INPUT_PULLUP); } //With pullup
    else { pinMode(pinFuel2Input, INPUT); } //Normal input
  }
  if( (configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH) && (!pinIsOutput(pinSpark2Input)) )
  {
    if (configPage10.spark2InputPullup == true) { pinMode(pinSpark2Input, INPUT_PULLUP); } //With pullup
    else { pinMode(pinSpark2Input, INPUT); } //Normal input
  }
  if( (configPage10.fuelPressureEnable > 0)  && (!pinIsOutput(pinFuelPressure)) )
  {
    pinMode(pinFuelPressure, INPUT);
  }
  if( (configPage10.oilPressureEnable > 0) && (!pinIsOutput(pinOilPressure)) )
  {
    pinMode(pinOilPressure, INPUT);
  }
  if( (configPage13.onboard_log_trigger_Epin > 0) && (!pinIsOutput(pinSDEnable)) )
  {
    pinMode(pinSDEnable, INPUT);
  }
  if(configPage10.wmiEnabled > 0)
  {
    pinMode(pinWMIEnabled, OUTPUT);
    if(configPage10.wmiIndicatorEnabled > 0)
    {
      pinMode(pinWMIIndicator, OUTPUT);
      if (configPage10.wmiIndicatorPolarity > 0) { digitalWrite(pinWMIIndicator, HIGH); }
    }
    if( (configPage10.wmiEmptyEnabled > 0) && (!pinIsOutput(pinWMIEmpty)) )
    {
      if (configPage10.wmiEmptyPolarity == 0) { pinMode(pinWMIEmpty, INPUT_PULLUP); } //Normal setting
      else { pinMode(pinWMIEmpty, INPUT); } //inverted setting
    }
  } 

  if((pinAirConComp>0) && ((configPage15.airConEnable) == 1))
  {
    pinMode(pinAirConComp, OUTPUT);
  }

  if((pinAirConRequest > 0) && ((configPage15.airConEnable) == 1) && (!pinIsOutput(pinAirConRequest)))
  {
    if((configPage15.airConReqPol) == 1)
    {
      // Inverted
      // +5V is ON, Use external pull-down resistor for OFF
      pinMode(pinAirConRequest, INPUT);
    }
    else
    {
      //Normal
      // Pin pulled to Ground is ON. Floating (internally pulled up to +5V) is OFF.
      pinMode(pinAirConRequest, INPUT_PULLUP);
    }
  }

  if((pinAirConFan > 0) && ((configPage15.airConEnable) == 1) && ((configPage15.airConFanEnabled) == 1))
  {
    pinMode(pinAirConFan, OUTPUT);
  }  

  //These must come after the above pinMode statements
  triggerPri_pin_port = portInputRegister(digitalPinToPort(pinTrigger));
  triggerPri_pin_mask = digitalPinToBitMask(pinTrigger);
  triggerSec_pin_port = portInputRegister(digitalPinToPort(pinTrigger2));
  triggerSec_pin_mask = digitalPinToBitMask(pinTrigger2);
  triggerThird_pin_port = portInputRegister(digitalPinToPort(pinTrigger3));
  triggerThird_pin_mask = digitalPinToBitMask(pinTrigger3);

  flex_pin_port = portInputRegister(digitalPinToPort(pinFlex));
  flex_pin_mask = digitalPinToBitMask(pinFlex);

}
/** Initialise the chosen trigger decoder.
 * - Set Interrupt numbers @ref triggerInterrupt, @ref triggerInterrupt2 and @ref triggerInterrupt3  by pin their numbers (based on board CORE_* define)
 * - Call decoder specific setup function triggerSetup_*() (by @ref config4.TrigPattern, set to one of the DECODER_* defines) and do any additional initialisations needed.
 * 
 * @todo Explain why triggerSetup_*() alone cannot do all the setup, but there's ~10+ lines worth of extra init for each of decoders.
 */
void initialiseTriggers(void)
{
  byte triggerInterrupt = 0; // By default, use the first interrupt
  byte triggerInterrupt2 = 1;
  byte triggerInterrupt3 = 2;

  #if defined(CORE_AVR)
    switch (pinTrigger) {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt = 0; break;
      case 3:
        triggerInterrupt = 1; break;
      case 18:
        triggerInterrupt = 5; break;
      case 19:
        triggerInterrupt = 4; break;
      case 20:
        triggerInterrupt = 3; break;
      case 21:
        triggerInterrupt = 2; break;
      default:
        triggerInterrupt = 0; break; //This should NEVER happen
    }
  #else
    triggerInterrupt = pinTrigger;
  #endif

  #if defined(CORE_AVR)
    switch (pinTrigger2) {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt2 = 0; break;
      case 3:
        triggerInterrupt2 = 1; break;
      case 18:
        triggerInterrupt2 = 5; break;
      case 19:
        triggerInterrupt2 = 4; break;
      case 20:
        triggerInterrupt2 = 3; break;
      case 21:
        triggerInterrupt2 = 2; break;
      default:
        triggerInterrupt2 = 0; break; //This should NEVER happen
    }
  #else
    triggerInterrupt2 = pinTrigger2;
  #endif

  #if defined(CORE_AVR)
    switch (pinTrigger3) {
      //Arduino Mega 2560 mapping
      case 2:
        triggerInterrupt3 = 0; break;
      case 3:
        triggerInterrupt3 = 1; break;
      case 18:
        triggerInterrupt3 = 5; break;
      case 19:
        triggerInterrupt3 = 4; break;
      case 20:
        triggerInterrupt3 = 3; break;
      case 21:
        triggerInterrupt3 = 2; break;
      default:
        triggerInterrupt3 = 0; break; //This should NEVER happen
    }
  #else
    triggerInterrupt3 = pinTrigger3;
  #endif

  pinMode(pinTrigger, INPUT);
  pinMode(pinTrigger2, INPUT);
  pinMode(pinTrigger3, INPUT);

  detachInterrupt(triggerInterrupt);
  detachInterrupt(triggerInterrupt2);
  detachInterrupt(triggerInterrupt3);
  //The default values for edges
  primaryTriggerEdge = 0; //This should ALWAYS be changed below
  secondaryTriggerEdge = 0; //This is optional and may not be changed below, depending on the decoder in use
  tertiaryTriggerEdge = 0; //This is even more optional and may not be changed below, depending on the decoder in use

  //Set the trigger function based on the decoder in the config
  switch (configPage4.TrigPattern)
  {
    case DECODER_MISSING_TOOTH:
      //Missing tooth decoder
      triggerSetup_missingTooth();
      triggerHandler = triggerPri_missingTooth;
      triggerSecondaryHandler = triggerSec_missingTooth;
      triggerTertiaryHandler = triggerThird_missingTooth;
      
      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_missingTooth;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }
      if(configPage10.TrigEdgeThrd == 0) { tertiaryTriggerEdge = RISING; }
      else { tertiaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);

      if(BIT_CHECK(decoderState, BIT_DECODER_HAS_SECONDARY)) { attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge); }
      if(configPage10.vvt2Enabled > 0) { attachInterrupt(triggerInterrupt3, triggerTertiaryHandler, tertiaryTriggerEdge); } // we only need this for vvt2, so not really needed if it's not used

      break;

    case DECODER_BASIC_DISTRIBUTOR:
      // Basic distributor
      triggerSetup_BasicDistributor();
      triggerHandler = triggerPri_BasicDistributor;
      getRPM = getRPM_BasicDistributor;
      getCrankAngle = getCrankAngle_BasicDistributor;
      triggerSetEndTeeth = triggerSetEndTeeth_BasicDistributor;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case 2:
      triggerSetup_DualWheel();
      triggerHandler = triggerPri_DualWheel;
      triggerSecondaryHandler = triggerSec_DualWheel;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_GM7X:
      triggerSetup_GM7X();
      triggerHandler = triggerPri_GM7X;
      getRPM = getRPM_GM7X;
      getCrankAngle = getCrankAngle_GM7X;
      triggerSetEndTeeth = triggerSetEndTeeth_GM7X;

      if(configPage4.TrigEdge == 0) { attachInterrupt(triggerInterrupt, triggerHandler, RISING); } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { attachInterrupt(triggerInterrupt, triggerHandler, FALLING); }

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_4G63:
      triggerSetup_4G63();
      triggerHandler = triggerPri_4G63;
      triggerSecondaryHandler = triggerSec_4G63;
      getRPM = getRPM_4G63;
      getCrankAngle = getCrankAngle_4G63;
      triggerSetEndTeeth = triggerSetEndTeeth_4G63;

      primaryTriggerEdge = CHANGE;
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_24X:
      triggerSetup_24X();
      triggerHandler = triggerPri_24X;
      triggerSecondaryHandler = triggerSec_24X;
      getRPM = getRPM_24X;
      getCrankAngle = getCrankAngle_24X;
      triggerSetEndTeeth = triggerSetEndTeeth_24X;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = CHANGE; //Secondary is always on every change

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_JEEP2000:
      triggerSetup_Jeep2000();
      triggerHandler = triggerPri_Jeep2000;
      triggerSecondaryHandler = triggerSec_Jeep2000;
      getRPM = getRPM_Jeep2000;
      getCrankAngle = getCrankAngle_Jeep2000;
      triggerSetEndTeeth = triggerSetEndTeeth_Jeep2000;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_AUDI135:
      triggerSetup_Audi135();
      triggerHandler = triggerPri_Audi135;
      triggerSecondaryHandler = triggerSec_Audi135;
      getRPM = getRPM_Audi135;
      getCrankAngle = getCrankAngle_Audi135;
      triggerSetEndTeeth = triggerSetEndTeeth_Audi135;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = RISING; //always rising for this trigger

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_HONDA_D17:
      triggerSetup_HondaD17();
      triggerHandler = triggerPri_HondaD17;
      triggerSecondaryHandler = triggerSec_HondaD17;
      getRPM = getRPM_HondaD17;
      getCrankAngle = getCrankAngle_HondaD17;
      triggerSetEndTeeth = triggerSetEndTeeth_HondaD17;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_MIATA_9905:
      triggerSetup_Miata9905();
      triggerHandler = triggerPri_Miata9905;
      triggerSecondaryHandler = triggerSec_Miata9905;
      getRPM = getRPM_Miata9905;
      getCrankAngle = getCrankAngle_Miata9905;
      triggerSetEndTeeth = triggerSetEndTeeth_Miata9905;

      //These may both need to change, not sure
      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_MAZDA_AU:
      triggerSetup_MazdaAU();
      triggerHandler = triggerPri_MazdaAU;
      triggerSecondaryHandler = triggerSec_MazdaAU;
      getRPM = getRPM_MazdaAU;
      getCrankAngle = getCrankAngle_MazdaAU;
      triggerSetEndTeeth = triggerSetEndTeeth_MazdaAU;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NON360:
      triggerSetup_non360();
      triggerHandler = triggerPri_DualWheel; //Is identical to the dual wheel decoder, so that is used. Same goes for the secondary below
      triggerSecondaryHandler = triggerSec_DualWheel; //Note the use of the Dual Wheel trigger function here. No point in having the same code in twice.
      getRPM = getRPM_non360;
      getCrankAngle = getCrankAngle_non360;
      triggerSetEndTeeth = triggerSetEndTeeth_non360;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NISSAN_360:
      triggerSetup_Nissan360();
      triggerHandler = triggerPri_Nissan360;
      triggerSecondaryHandler = triggerSec_Nissan360;
      getRPM = getRPM_Nissan360;
      getCrankAngle = getCrankAngle_Nissan360;
      triggerSetEndTeeth = triggerSetEndTeeth_Nissan360;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = CHANGE;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_SUBARU_67:
      triggerSetup_Subaru67();
      triggerHandler = triggerPri_Subaru67;
      triggerSecondaryHandler = triggerSec_Subaru67;
      getRPM = getRPM_Subaru67;
      getCrankAngle = getCrankAngle_Subaru67;
      triggerSetEndTeeth = triggerSetEndTeeth_Subaru67;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = FALLING;

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_DAIHATSU_PLUS1:
      triggerSetup_Daihatsu();
      triggerHandler = triggerPri_Daihatsu;
      getRPM = getRPM_Daihatsu;
      getCrankAngle = getCrankAngle_Daihatsu;
      triggerSetEndTeeth = triggerSetEndTeeth_Daihatsu;

      //No secondary input required for this pattern
      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_HARLEY:
      triggerSetup_Harley();
      triggerHandler = triggerPri_Harley;
      //triggerSecondaryHandler = triggerSec_Harley;
      getRPM = getRPM_Harley;
      getCrankAngle = getCrankAngle_Harley;
      triggerSetEndTeeth = triggerSetEndTeeth_Harley;

      primaryTriggerEdge = RISING; //Always rising
      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_36_2_2_2:
      //36-2-2-2
      triggerSetup_ThirtySixMinus222();
      triggerHandler = triggerPri_ThirtySixMinus222;
      triggerSecondaryHandler = triggerSec_ThirtySixMinus222;
      getRPM = getRPM_ThirtySixMinus222;
      getCrankAngle = getCrankAngle_missingTooth; //This uses the same function as the missing tooth decoder, so no need to duplicate code
      triggerSetEndTeeth = triggerSetEndTeeth_ThirtySixMinus222;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_36_2_1:
      //36-2-1
      triggerSetup_ThirtySixMinus21();
      triggerHandler = triggerPri_ThirtySixMinus21;
      triggerSecondaryHandler = triggerSec_missingTooth;
      getRPM = getRPM_ThirtySixMinus21;
      getCrankAngle = getCrankAngle_missingTooth; //This uses the same function as the missing tooth decoder, so no need to duplicate code
      triggerSetEndTeeth = triggerSetEndTeeth_ThirtySixMinus21;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_420A:
      //DSM 420a
      triggerSetup_420a();
      triggerHandler = triggerPri_420a;
      triggerSecondaryHandler = triggerSec_420a;
      getRPM = getRPM_420a;
      getCrankAngle = getCrankAngle_420a;
      triggerSetEndTeeth = triggerSetEndTeeth_420a;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      secondaryTriggerEdge = FALLING; //Always falling edge

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_WEBER:
      //Weber-Marelli
      triggerSetup_DualWheel();
      triggerHandler = triggerPri_Webber;
      triggerSecondaryHandler = triggerSec_Webber;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_ST170:
      //Ford ST170
      triggerSetup_FordST170();
      triggerHandler = triggerPri_missingTooth;
      triggerSecondaryHandler = triggerSec_FordST170;
      getRPM = getRPM_FordST170;
      getCrankAngle = getCrankAngle_FordST170;
      triggerSetEndTeeth = triggerSetEndTeeth_FordST170;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);

      break;
	  
    case DECODER_DRZ400:
      triggerSetup_DRZ400();
      triggerHandler = triggerPri_DualWheel;
      triggerSecondaryHandler = triggerSec_DRZ400;
      getRPM = getRPM_DualWheel;
      getCrankAngle = getCrankAngle_DualWheel;
      triggerSetEndTeeth = triggerSetEndTeeth_DualWheel;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_NGC:
      //Chrysler NGC - 4, 6 and 8 cylinder
      triggerSetup_NGC();
      triggerHandler = triggerPri_NGC;
      getRPM = getRPM_NGC;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_NGC;

      primaryTriggerEdge = CHANGE;
      if (configPage2.nCylinders == 4) {
        triggerSecondaryHandler = triggerSec_NGC4;
        secondaryTriggerEdge = CHANGE;
      }
      else {
        triggerSecondaryHandler = triggerSec_NGC68;
        secondaryTriggerEdge = FALLING;
      }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;

    case DECODER_VMAX:
      triggerSetup_Vmax();
      triggerHandler = triggerPri_Vmax;
      getRPM = getRPM_Vmax;
      getCrankAngle = getCrankAngle_Vmax;
      triggerSetEndTeeth = triggerSetEndTeeth_Vmax;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = true; } // set as boolean so we can directly use it in decoder.
      else { primaryTriggerEdge = false; }
      
      attachInterrupt(triggerInterrupt, triggerHandler, CHANGE); //Hardcoded change, the primaryTriggerEdge will be used in the decoder to select if it`s an inverted or non-inverted signal.
      break;

    case DECODER_RENIX:
      //Renault 44 tooth decoder
      triggerSetup_Renix();
      triggerHandler = triggerPri_Renix;
      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;
      triggerSetEndTeeth = triggerSetEndTeeth_Renix;

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt 
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }

      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;

    case DECODER_ROVERMEMS:
      //Rover MEMs - covers multiple flywheel trigger combinations.
      triggerSetup_RoverMEMS();
      triggerHandler = triggerPri_RoverMEMS;
      getRPM = getRPM_RoverMEMS;
      triggerSetEndTeeth = triggerSetEndTeeth_RoverMEMS;
            
      triggerSecondaryHandler = triggerSec_RoverMEMS; 
      getCrankAngle = getCrankAngle_missingTooth;   

      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      if(configPage4.TrigEdgeSec == 0) { secondaryTriggerEdge = RISING; }
      else { secondaryTriggerEdge = FALLING; }
      
      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      attachInterrupt(triggerInterrupt2, triggerSecondaryHandler, secondaryTriggerEdge);
      break;   

    case DECODER_SUZUKI_K6A:
      triggerSetup_SuzukiK6A();
      triggerHandler = triggerPri_SuzukiK6A; // only primary, no secondary, trigger pattern is over 720 degrees
      getRPM = getRPM_SuzukiK6A;
      getCrankAngle = getCrankAngle_SuzukiK6A;
      triggerSetEndTeeth = triggerSetEndTeeth_SuzukiK6A;


      if(configPage4.TrigEdge == 0) { primaryTriggerEdge = RISING; } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { primaryTriggerEdge = FALLING; }
      
      attachInterrupt(triggerInterrupt, triggerHandler, primaryTriggerEdge);
      break;


    default:
      triggerHandler = triggerPri_missingTooth;
      getRPM = getRPM_missingTooth;
      getCrankAngle = getCrankAngle_missingTooth;

      if(configPage4.TrigEdge == 0) { attachInterrupt(triggerInterrupt, triggerHandler, RISING); } // Attach the crank trigger wheel interrupt (Hall sensor drags to ground when triggering)
      else { attachInterrupt(triggerInterrupt, triggerHandler, FALLING); }
      break;
  }

  #if defined(CORE_TEENSY41)
    //Teensy 4 requires a HYSTERESIS flag to be set on the trigger pins to prevent false interrupts
    setTriggerHysteresis();
  #endif
}

static inline bool isAnyFuelScheduleRunning(void) {
  return fuelSchedule1.Status==RUNNING
      || fuelSchedule2.Status==RUNNING
      || fuelSchedule3.Status==RUNNING
      || fuelSchedule4.Status==RUNNING
#if INJ_CHANNELS >= 5      
      || fuelSchedule5.Status==RUNNING
#endif
#if INJ_CHANNELS >= 6
      || fuelSchedule6.Status==RUNNING
#endif
#if INJ_CHANNELS >= 7
      || fuelSchedule7.Status==RUNNING
#endif
#if INJ_CHANNELS >= 8
      || fuelSchedule8.Status==RUNNING
#endif
      ;
}

static inline bool isAnyIgnScheduleRunning(void) {
  return ignitionSchedule1.Status==RUNNING      
#if IGN_CHANNELS >= 2 
      || ignitionSchedule2.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 3 
      || ignitionSchedule3.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 4       
      || ignitionSchedule4.Status==RUNNING
#endif      
#if IGN_CHANNELS >= 5      
      || ignitionSchedule5.Status==RUNNING
#endif
#if IGN_CHANNELS >= 6
      || ignitionSchedule6.Status==RUNNING
#endif
#if IGN_CHANNELS >= 7
      || ignitionSchedule7.Status==RUNNING
#endif
#if IGN_CHANNELS >= 8
      || ignitionSchedule8.Status==RUNNING
#endif
      ;
}

/** Change injectors or/and ignition angles to 720deg.
 * Roll back req_fuel size and set number of outputs equal to cylinder count.
* */
void changeHalfToFullSync(void)
{
  //Need to do another check for injLayout as this function can be called from ignition
  noInterrupts();
  if( (configPage2.injLayout == INJ_SEQUENTIAL) && (CRANK_ANGLE_MAX_INJ != 720) && (!isAnyFuelScheduleRunning()))
  {
    CRANK_ANGLE_MAX_INJ = 720;
    req_fuel_uS *= 2;
    
    fuelSchedule1.pStartFunction = openInjector1;
    fuelSchedule1.pEndFunction = closeInjector1;
    fuelSchedule2.pStartFunction = openInjector2;
    fuelSchedule2.pEndFunction = closeInjector2;
    fuelSchedule3.pStartFunction = openInjector3;
    fuelSchedule3.pEndFunction = closeInjector3;
    fuelSchedule4.pStartFunction = openInjector4;
    fuelSchedule4.pEndFunction = closeInjector4;
#if INJ_CHANNELS >= 5
    fuelSchedule5.pStartFunction = openInjector5;
    fuelSchedule5.pEndFunction = closeInjector5;
#endif
#if INJ_CHANNELS >= 6
    fuelSchedule6.pStartFunction = openInjector6;
    fuelSchedule6.pEndFunction = closeInjector6;
#endif
#if INJ_CHANNELS >= 7
    fuelSchedule7.pStartFunction = openInjector7;
    fuelSchedule7.pEndFunction = closeInjector7;
#endif
#if INJ_CHANNELS >= 8
    fuelSchedule8.pStartFunction = openInjector8;
     fuelSchedule8.pEndFunction = closeInjector8;
#endif

    switch (configPage2.nCylinders)
    {
      case 4:
        maxInjOutputs = 4;
        break;
            
      case 6:
        maxInjOutputs = 6;
        break;

      case 8:
        maxInjOutputs = 8;
        break;

      default:
        break; //No actions required for other cylinder counts

    }
  }
  interrupts();

  //Need to do another check for sparkMode as this function can be called from injection
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (CRANK_ANGLE_MAX_IGN != 720) && (!isAnyIgnScheduleRunning()) )
  {
    CRANK_ANGLE_MAX_IGN = 720;
    maxIgnOutputs = configPage2.nCylinders;
    switch (configPage2.nCylinders)
    {
    case 4:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      break;

    case 6:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      ignitionSchedule3.pStartCallback = beginCoil3Charge;
      ignitionSchedule3.pEndCallback = endCoil3Charge;
      break;

    case 8:
      ignitionSchedule1.pStartCallback = beginCoil1Charge;
      ignitionSchedule1.pEndCallback = endCoil1Charge;
      ignitionSchedule2.pStartCallback = beginCoil2Charge;
      ignitionSchedule2.pEndCallback = endCoil2Charge;
      ignitionSchedule3.pStartCallback = beginCoil3Charge;
      ignitionSchedule3.pEndCallback = endCoil3Charge;
      ignitionSchedule4.pStartCallback = beginCoil4Charge;
      ignitionSchedule4.pEndCallback = endCoil4Charge;
      break;

    default:
      break; //No actions required for other cylinder counts
      
    }
  }
}

/** Change injectors or/and ignition angles to 360deg.
 * In semi sequentiol mode req_fuel size is half.
 * Set number of outputs equal to half cylinder count.
* */
void changeFullToHalfSync(void)
{
  if(configPage2.injLayout == INJ_SEQUENTIAL)
  {
    CRANK_ANGLE_MAX_INJ = 360;
    req_fuel_uS /= 2;
    switch (configPage2.nCylinders)
    {
      case 4:
        if(configPage4.inj4cylPairing == INJ_PAIR_13_24)
        {
          fuelSchedule1.pStartFunction = openInjector1and3;
          fuelSchedule1.pEndFunction = closeInjector1and3;
          fuelSchedule2.pStartFunction = openInjector2and4;
          fuelSchedule2.pEndFunction = closeInjector2and4;
        }
        else
        {
          fuelSchedule1.pStartFunction = openInjector1and4;
          fuelSchedule1.pEndFunction = closeInjector1and4;
          fuelSchedule2.pStartFunction = openInjector2and3;
          fuelSchedule2.pEndFunction = closeInjector2and3;
        }
        maxInjOutputs = 2;
        break;
            
      case 6:
        fuelSchedule1.pStartFunction = openInjector1and4;
        fuelSchedule1.pEndFunction = closeInjector1and4;
        fuelSchedule2.pStartFunction = openInjector2and5;
        fuelSchedule2.pEndFunction = closeInjector2and5;
        fuelSchedule3.pStartFunction = openInjector3and6;
        fuelSchedule3.pEndFunction = closeInjector3and6;
        maxInjOutputs = 3;
        break;

      case 8:
        fuelSchedule1.pStartFunction = openInjector1and5;
        fuelSchedule1.pEndFunction = closeInjector1and5;
        fuelSchedule2.pStartFunction = openInjector2and6;
        fuelSchedule2.pEndFunction = closeInjector2and6;
        fuelSchedule3.pStartFunction = openInjector3and7;
        fuelSchedule3.pEndFunction = closeInjector3and7;
        fuelSchedule4.pStartFunction = openInjector4and8;
        fuelSchedule4.pEndFunction = closeInjector4and8;
        maxInjOutputs = 4;
        break;
    }
  }

  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    CRANK_ANGLE_MAX_IGN = 360;
    maxIgnOutputs = configPage2.nCylinders / 2;
    switch (configPage2.nCylinders)
    {
      case 4:
        ignitionSchedule1.pStartCallback = beginCoil1and3Charge;
        ignitionSchedule1.pEndCallback = endCoil1and3Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and4Charge;
        ignitionSchedule2.pEndCallback = endCoil2and4Charge;
        break;
            
      case 6:
        ignitionSchedule1.pStartCallback = beginCoil1and4Charge;
        ignitionSchedule1.pEndCallback = endCoil1and4Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and5Charge;
        ignitionSchedule2.pEndCallback = endCoil2and5Charge;
        ignitionSchedule3.pStartCallback = beginCoil3and6Charge;
        ignitionSchedule3.pEndCallback = endCoil3and6Charge;
        break;

      case 8:
        ignitionSchedule1.pStartCallback = beginCoil1and5Charge;
        ignitionSchedule1.pEndCallback = endCoil1and5Charge;
        ignitionSchedule2.pStartCallback = beginCoil2and6Charge;
        ignitionSchedule2.pEndCallback = endCoil2and6Charge;
        ignitionSchedule3.pStartCallback = beginCoil3and7Charge;
        ignitionSchedule3.pEndCallback = endCoil3and7Charge;
        ignitionSchedule4.pStartCallback = beginCoil4and8Charge;
        ignitionSchedule4.pEndCallback = endCoil4and8Charge;
        break;
    }
  }
}
