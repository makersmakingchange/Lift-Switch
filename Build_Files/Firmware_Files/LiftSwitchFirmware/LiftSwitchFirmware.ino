/*
                   LIFT SWITCH Mk-IV

            Stan Cassidy Center for Rehabilitation
                      Jan, 2024

            Arduino Platform: Adafruit Trinket M0

            Prcessor: ATSAMD21

            Firmware version number#: 20240105-01 v1.2

            (C)2023 STAN CASSIDY CENTER FOR REHABILITATION (SCCR)

                           ===== DISCLAMER =====
  THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  ----------------------------------------------------------------------------------------------
  Special features:
     # Adafruit Trinket M0 microcontroller
     # USB B micro port for subsequent programing and interface to PC (mouse/kb emulator)
     # Accessible switch "connected" detection
     # Tri-colour status LED

  Single button operation:
     # Turn power on
     # Press MODE to change mode of operation:
         - Normal (momentary) lift switch operation  (indicated by GREEN LED)
         - Latching: lift once for on, lift a second time for off (indicatted by BLUE LED)
         - Reverse (momentary) operation (indicated by RED LED)

  LED messages:
     # MODE function - 
        GREEN to indicate normal (momentary) operation; a second press will turn the LED
        BLUE to indicate latched function; a third press will turn LED 
        RED to indicate reverse operation.
        OFF - LED goes out after 2 seconds to conserve battery power.

  FROM DOCS:
  The selected mode is indicated by the light colour:
  Blue is Latching
  Red is Reverse Acting
  Green is Momentary
.
  Mode of operation:
     # In momentary mode, the solid state relay conducts for 500 milliseconds when the input button is released (opened).
        When used as a mouse device, it provides a single left click.
     # In latch mode, the relay is latched in conducting mode when the input switch contacts are
       released (opened) then closed, the relay opens (not conducting) on second opening and closing of
       input switch contacts.
       When used as a mouse device, provides a left button press.
     # in reverse mode, contacts are open (not conducting) when switch is pressed, and conducting when switch is released.
        When used as a mouse device, provides a reversed left button press.

  Notes:
  - Ensure you have https://adafruit.github.io/arduino-board-index/package_adafruit_index.json on your Additional Boards Manager URL
      and install "Adafruit SAMD Boards" v1.7.13+
  - Choose Board "Adafruit SAMD Board -> Adafruit SAMD Trinket M0 (SAMD21)"
  - Choose USB Stack: "TinyUSB"
  - Need to install Adafruit DotStar 1.2.5+, FlashStorage 1.0.0+

  Development Notes:
  - If you get programming errors after the WIRE library inclusion - try UNPLUGGING any switches that's plugged in the 3.5mm sterio jack. Also, reset the board using the onboard reset button
*/

// External libraries:
#include <Adafruit_TinyUSB.h>
#include <Adafruit_DotStar.h>
#include <FlashAsEEPROM.h>
#include <FlashStorage.h>

#define VERSION_LIFTSWITCH "1.2"

#define MOMEN 250  // Momentary relay closure time in milliseconds

// I/O Mapping:
#define SWITCH_IN 2  // Normally open momentary adaptive switch input on pin D7 (active low)
#define SSR 0        // Solid state relay control output on pin D9 (active high)
#define MODE_SW 1    // Mode pushbutton input on pin D8 (active low)
#define VBAT_PIN  A1 // VBAT Pin in the trinket 
 
#define MOUSE_DELAY 100   // Minimum amount of time between input signals for mouse functions
#define BAUD_RATE 115200  // USB communications baud rate

#define adc_disable() (ADC->CTRLA.bit.ENABLE = 0x00) // disable ADC (before power-off)

#define MODE_MOMENT 0
#define MODE_LATCH 1
#define MODE_TIMED_LATCH 2
#define MODE_REV 3
byte opMode = MODE_MOMENT;       // Mode of operation: 0 = momentary (GREEN); 1 = latching (BLUE); 2 = timed latch (YELLOW); 3 = reverse (RED)
//Mode persistance
FlashStorage(mode_storage, byte);


byte inputButton;      // stores status of input button read
byte lastButton = 1;   // stores state of last button read
byte lastMode = 1;     // Stores state of last mode switch read
bool relayOff = true;  // status of output relay (active high)

uint32_t lastDebounceTime = 0;  // the last time the output pin was toggled
#define debounceDelay 50        // the debounce time; increase if the output flickers


//Set up the on board RGB LED
Adafruit_DotStar pixel(1, INTERNAL_DS_DATA, INTERNAL_DS_CLK, DOTSTAR_BGR);

//Config Mouse for TinyUSB
// HID report descriptor using TinyUSB's template - Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_MOUSE()
};
// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);


//Timed Latch Delay persistance
int32_t timedLatchDelay = 7000;  //Amount of time to hold down then timed latch. Will be read out of flash memory. This is default
FlashStorage(timedLatchDelay_storage, unsigned long);

void setup() {

  //Might want to turn off the ADC
  // Read through the init in wiring.c to see what we might be able to turn off to save power
  //set_sleep_mode(SLEEP_MODE_STANDBY); // will use this more later
  adc_disable();

  //Set up 'Friendly' names when plugged into a computer (Instead of just 'Trinket M0' showing up)
  TinyUSBDevice.setManufacturerDescriptor("SCCR");
  TinyUSBDevice.setProductDescriptor("Lift Switch");

  //https://github.com/adafruit/ArduinoCore-samd/issues/128 - not sure we have a onboard buck regulator in the SAMD21 we can use https://cdn-learn.adafruit.com/downloads/pdf/adafruit-trinket-m0-circuitpython-arduino.pdf
  //SUPC->VREG.bit.SEL = 1;

  // I/O configuration:
  pinMode(SWITCH_IN, INPUT_PULLUP);
  pinMode(MODE_SW, INPUT_PULLUP);
  pinMode(SSR, OUTPUT);

 // pinMode(VBAT_PIN, INPUT);

  pixel.begin();

  usb_hid.begin();

  Serial.begin(BAUD_RATE);

  //Wait for USB to time out if not connected
  unsigned long _start = millis();
  boolean _serialConnected = true;
  while(!Serial)         //Wait until the serial port is opened with a 5 second timeout
  {
    //Blink while looking for serial
    pixel.setPixelColor(0, 50, 0, 50);  //Turn on VIOLET LED 
 
    //Make the LED breath
    for (int i = 0; i < 255; i+=2) {
      pixel.setBrightness( i );
      pixel.show(); //Show the color
      delay(3);
    }
    for (int i = 255; i > 0; i-=2) {
      pixel.setBrightness( i );
      pixel.show(); //Show the color
      delay(3);
    }

    if (millis() - _start > 5000)
    {
      //NO Serial connected
      _serialConnected = false;
      break;
    }
  }

  // Print info so that when a switch is plugged in, we can see the firmware version / date it was last programmed
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing firmware version " VERSION_LIFTSWITCH));

  // wait until device mounted. Might need to change in the future if we're also using bluetooth (not just Serial USB)
  if (_serialConnected)
  {
    while (!TinyUSBDevice.mounted()) delay(1);
  }

  opMode = readMode();
  timedLatchDelay = readTimeLatchedDelay();

  //Indicate the initial mode by turning on the LED
  pixel.setBrightness( 255 );
  pixel.setPixelColor(0, getModeColor(opMode));  //Turn on GREEN LED - the color for the start-up mode. 
  pixel.show(); //Show the color
  delay(2000); // Delay 2 seconds so user can see the mode they're in. 
  pixel.setPixelColor(0, 0);  //Turn LED OFF 
  pixel.show(); //Show the color

}  // end of setup()


void loop() {
  // poll gpio once each 10 ms
  delay(10);

  // Check if Mode change is requested
  bool _changedMode = doMode();

  if (_changedMode) 
  {
    pixel.setPixelColor(0, getModeColor(opMode));  //Turn off the LED
    pixel.show(); //Show the new mode

    delay(2000); // Delay 2 seconds so user can see the mode they're in. 

    // TURN OFF LED
    pixel.setPixelColor(0, 0);  //Turn off the LED
    pixel.show(); //Show the new mode

    storeMode(opMode);
  }
/*
  //Check to see if enough time has passed to read the button. Once the button is pressed, we'll STOP reading it for a bit to wait for it to settle down
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Check status of input button and perform function
    uint8_t reading = digitalRead(SWITCH_IN);  // Read current state of the button. We'll use this to debounce. reading will = 0 (LOW) when the button is pressed
    if (reading != inputButton) {  // Something has changed
      {
        // reset the debouncing timer
        lastDebounceTime = millis();
        inputButton = reading;
      }
    }
*/

  // Check status of input button and perform function
  inputButton = digitalRead(SWITCH_IN);                                  // Read and debounce the button
  if (inputButton != lastButton) {                                    //Something has changed
    delay(50);                                                        // Debounce delay

    if (digitalRead(SWITCH_IN)) {  // Switch released

      // Remote wakeup
      if ( TinyUSBDevice.mounted() && TinyUSBDevice.suspended() )
      {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        TinyUSBDevice.remoteWakeup();
      }
  
      //We're about to send a new mouse (HID) packet. Make sure we're ready
      //while (!usb_hid.ready());

      switch (opMode) {
        case MODE_MOMENT:           // do a mouse click & momentary relay

          Serial.printf("MOMENTARY CLICK\n");
          
          mouseClick();            // Send a mouse left click to the computer

          digitalWrite(SSR, HIGH);  // Momentary operation of solid-state relay
          delay(MOMEN);
          digitalWrite(SSR, LOW);

          break;

        case MODE_LATCH:                      // do left click & latching relay

          Serial.printf("LATCH CLICK\n");

          relayOff = !relayOff;
          if (relayOff) {

            digitalWrite(SSR, LOW); // Turn off relay (open contacts)
            mouseRelease();  // Release mouse left button
          }
          if (!relayOff) {
            digitalWrite(SSR, HIGH);  // Turn on relay (close contacts)
            mousePress();  // Release mouse left button
          }
          break;

        case MODE_TIMED_LATCH:                      // do left click & latching relay. Wait 3 7 seconds, then turn off. 

          Serial.printf("TIMED LATCH CLICK\n");

          digitalWrite(SSR, HIGH);  // Turn on relay
          mousePress();  // Do a left mouse press

          delay(timedLatchDelay);

          digitalWrite(SSR, LOW); // Turn off relay (open contacts)
          mouseRelease();  // Release mouse left button

          break;

        case MODE_REV:                     // do mouse left button & reverse relay

          Serial.printf("REV CLICK\n");

          digitalWrite(SSR, HIGH);  // Turn on relay
          mousePress();  // Do a left mouse press
          break;

        default:
          break;
      }                                         // done switch/case
    }
    else {                                    // Switch pressed
      if (opMode == MODE_REV)
      {
        digitalWrite(SSR, LOW);  // Turn off relay
        mouseRelease();          // Release left mouse button
      }
    }

    lastButton = inputButton; // Store most recent state of button
  }


  //check for input. TL 3000 would set the timed latech to 3000 seconds
  if (Serial.available() > 0)
  {
    String str = Serial.readString();
    str.trim();
    Serial.print("Serial command received: ");
    Serial.println(str);

    if (str.startsWith("TL "))
    {
      //update the stored timed latch settings
      String newTimedLatchDelay = str.substring(3);
      Serial.print("Setting delay for timed latch: ");
      Serial.println(newTimedLatchDelay);
      timedLatchDelay = newTimedLatchDelay.toInt(); //returns a long, despite the name of the function.
      
      //Store it
      timedLatchDelay_storage.write(timedLatchDelay);
    }
  }
}  // done loop()

// ---------------------------------------------------------------------------------------- //
//                                                                                          //
//                                        FUNCTIONS                                         //
//                                                                                          //
// ---------------------------------------------------------------------------------------- //
void mouseClick()
{
  // Serial.printf("Mouse Click\n");
  mousePress();
  delay(10);
  mouseRelease();
}

void mousePress() {
  if (usb_hid.ready())
    usb_hid.mouseButtonPress(0,1); 
  
  // Serial.printf("  Mouse Press\n");
}

void mouseRelease()
{
  if (usb_hid.ready())
    usb_hid.mouseButtonRelease(0); 
  
  Serial.printf("  Mouse Release\n");
}


uint32_t getBatteryVoltage(void)
{
  return 1;
}

/**
 * Returns the proper LED color for the given mode 
 * 
 * @param opMode current mode
 * @return unit32 representing the color assigned to the given mode. 
 * Notes: See https://adafruit.github.io/Adafruit_DotStar/html/class_adafruit___dot_star.html
 */
uint32_t getModeColor(byte opMode)
{
  uint32_t modeColor = pixel.Color(100, 100, 100); //DEFAULT color for unknown mode

  if (opMode == MODE_MOMENT)
    modeColor = pixel.Color(0, 50, 0); // GREEN
    
  else if (opMode == MODE_LATCH)
    modeColor = pixel.Color(0, 0, 50);  // BLUE

  else if (opMode == MODE_TIMED_LATCH)
    modeColor = pixel.Color(50, 50, 0);  // YELLOW

  else if (opMode == MODE_REV)
    modeColor = pixel.Color(50, 0, 0);  // RED

  return modeColor;
}

/**
 * Checks for mode button press and advances the mode to the next mode 
 * 
 * @param filename GIF File name to be loaded
 * @return true of the mode was changed. False otherwise
 * Notes:  Could probably move to be interrupt driven?
 */
bool doMode(void) {
  // Read MODE button to detect user request, change operating mode accordingly
  // Mode of operation: 0 = momentary (GREEN); 1 = latching (BLUE); 2 = reverse (RED)

  bool _modeChanged = false;


  // Can we detect the battery voltage?
  


  // Local (private) variables
  byte _reading = digitalRead(MODE_SW);
  if (_reading == 0)
  {  // Button pressed
    _modeChanged = true;
  
    do {
      delay(50);  // Debounce delay
      _reading = digitalRead(MODE_SW);
    } while (!_reading);  // Loop until button released

#define MODE_MOMENT 0
#define MODE_LATCH 1
#define MODE_TIMED_LATCH 2
#define MODE_REV 3


    switch (opMode) { //Check the LAST mode and move to NEW MODE
      case 0: //Was momentary                                 
        opMode = MODE_LATCH;                // Change to latching relay output
        break;

      case 1: //Was Latch
        opMode = MODE_TIMED_LATCH;          // Change to timed latch mode
        break;

      case 2: //Was Timed Latch
        opMode = MODE_REV;                  // Change to reverse relay output
        break;

      case 3: //Was reverse
        opMode = MODE_MOMENT;              // Change to momentary relay output
        break;

      default:
        break;
    }  // done switch/case statement
  }
  
  return _modeChanged;
}

/**
 * Stores MODE in Flash storage 
 * 
 * @param currentMode a byte representing the operating mode you want to store. Typcially, the LAST mode the user selected
 * @return none
 * Notes:  See https://github.com/cmaglie/FlashStorage https://github.com/cmaglie/FlashStorage/blob/master/examples/EmulateEEPROM/EmulateEEPROM.ino
 */
void storeMode(byte newMode)
{
  Serial.printf("Storing mode: %d \n", newMode);
  mode_storage.write(newMode);
}

/**
 * Gets the current MODE in Flash storage. If none was stored, the default (MODE_MOMENT) is returned
 * 
 * @return byte representing the stored mode.
 * Notes:  See https://github.com/cmaglie/FlashStorage https://github.com/cmaglie/FlashStorage/blob/master/examples/EmulateEEPROM/EmulateEEPROM.ino
 */
byte readMode()
{
  byte _mode = MODE_MOMENT;
  _mode = mode_storage.read();

  if ((_mode != MODE_MOMENT) &&
        (_mode != MODE_LATCH) &&
          (_mode != MODE_TIMED_LATCH) &&
            (_mode != MODE_REV))
  {
     Serial.printf("Unknown mode. Returning %d \n", MODE_MOMENT);
    _mode = MODE_MOMENT;
  }
  else
  {
    Serial.printf("Valid mode found: %d \n", _mode);
  }

  return _mode;
}

long readTimeLatchedDelay()
{
  long _storedDelay = 7000; // Default
  _storedDelay = timedLatchDelay_storage.read(); 

  Serial.printf("Stored time latch delay: %d \n", _storedDelay);

  //Check for negative. This should really be an unsigned value, but String.toInt() returns a long, not an unsigned long
  if (_storedDelay <= 0)
  {
    Serial.printf("Invalid time latch delay. Defaulting 7000. \n", _storedDelay);

    _storedDelay = 7000; //default to 7000.
  }

  return _storedDelay;

}
// ---------------------------------------------------------------------------------------- //
