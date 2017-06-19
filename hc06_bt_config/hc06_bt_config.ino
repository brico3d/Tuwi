/*
    The following code allows you to configure the HC-06 Bluetooth module from Arduino and the TwRobot aplication
    You can configurate the name, baud rate and password used with the serial port.
    Bluetooth configuration is stored in the module, so you only need to configure once.
    
    After recording the program in the Arduino, there are 10 seconds wait to connect the bluetooth
    module, once the LED turns off will start the setup process (TX and RX leds of Arduino blink)
    and at the end the led begin to blink.

    The configuration must be done with the unpaired module (not connected to anything)
*/



// Config Options

//char ssid[10]		= "Tuwi";	// Name of module
//char baudios		= '7';		// 1=>1200, 2=>2400, 3=>4800, 4=>9600, 5=>19200, 6=>38400, 7=>57600, 8=>115200
//char password[10]	= "1234";	// Password module.


void setup(){
  //Baud Rate actual configuration (9600 factory)
  Serial.begin(9600);
	
  // Wait time
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(10000);
  digitalWrite(13,LOW);
	
  // Module configuration
	
  // Configuration start
  Serial.print("AT"); delay(1000);

  // Name record
  Serial.print("AT+NAME"); Serial.print("Tuwi"); delay(1000);

  // Baud record
  Serial.print("AT+BAUD"); Serial.print('7'); delay(1000);

  // Password record
  Serial.print("AT+PIN"); Serial.print("1234"); delay(1000);	
}

void loop()
{
	// With the led blinking the configuration is finished
	digitalWrite(13, !digitalRead(13));
	delay(500);
}
