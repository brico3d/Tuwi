  // Programa control Tuwi con App Joystick BT Commander
  // Hecho por: Brico3D
  // Consultar: https://brico3d.blogspot.com
  
  #include <PID_v1.h>
  #include <math.h>
  #include <Ultrasonic.h>
  
  long distance;
  int joyX, joyY;
  int modo;
  
  #define    STX          0x02
  #define    ETX          0x03
  #define    SLOW         750                             // Datafields refresh rate (ms)
  #define    FAST         250                             // Datafields refresh rate (ms)
  
  #define encoderL_pin 2                                  // Pin 2, encoder izquierdo
  #define encoderR_pin 3                                  // Pin 3, encoder derecho
  
  #define motor1PinPlus 13                                // In1 Motor1
  #define motor1PinMinus 12                               // In2 Motor1
  #define motor2PinPlus 7                                 // In1 Motor2
  #define motor2PinMinus 8                                // In2 Motor2
  #define motor1PinEnable 11                              // Enable Motor1
  #define motor2PinEnable 9                               // Enable Motor2
  
  #define TRIGGER_PIN  4                                  // Arduino pin tied to trigger pin on the ultrasonic sensor.
  #define ECHO_PIN     5                                  // Arduino pin tied to echo pin on the ultrasonic sensor.
  #define MAX_DISTANCE 150                                // Maxima distancia que pretendemos medir
  
  #define speakerPin 6                                    //Pin de conexion al buzzer
  
  #define S_ON               0
  #define S_connection       1
  #define S_disconnection    2
  #define S_mode1            3
  #define S_mode2            4
  #define S_look             5
  #define S_lost             6
  
  //Variables de comunicacion
  byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
  byte buttonStatus = 0;                                  // first Byte sent to Android device
  long previousMillis = 0;                                // will store last time Buttons status was updated
  long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
  String displayStatus = "Reposo";                          // message to Android device
  int connection = 0;

  //Variables de conversion de velocidad
  bool forwardL, forwardR;                                //Direccion de cada rueda
  int SpeedL, SpeedR;                                     //Velocidad de cada rueda
  double Vmax = 2.8;                                      //Velocidad maxima 280rpm
  
  //Variables adquisición de velocidad
  unsigned int NL = 0, NR = 0;      // [RPM] Revoluciones por minuto calculadas.
  volatile byte nL = 0, nR = 0;     // [pulses] Número de pulsos leídos
  unsigned long timeold = 0;    // Tiempo de muestreo
  unsigned int S = 20;      // Pasos del encoder.
  
  //Inicialización del control de velocidad
  double SetpointL, SetpointR, InputL, OutputL, InputR, OutputR;
  double Kp=0.33, Ki=3.5, Kd=0.014;
  PID myPIDL(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
  PID myPIDR(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

  //Iniciamos el sensor de ultrasonidos
  Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
  int distancia, vel;
  int separacion = 20;
  int count = 0, busca = 0;
  int t_busca = 10;
  
  void setup()  {
    Serial.begin(57600);
    while(Serial.available())  Serial.read();         // empty RX buffer
    
    //Inicialización de los pines para las interrupciones de los encoder
    pinMode(encoderL_pin, INPUT);
    pinMode(encoderR_pin, INPUT);
    attachInterrupt(0, counterL, CHANGE);                 // La interrupcion se ejecuta en flanco pos y neg
    attachInterrupt(1, counterR, CHANGE);
    
    //Inicio controladores
     myPIDL.SetMode(AUTOMATIC);
     myPIDR.SetMode(AUTOMATIC);
     
     sing(S_ON);
     modo = 0;
}
  
  void loop() {
    
    bluetooth();
        
    //Ejecución periódica cada 100ms
    if (millis() - timeold >= 100){                      // Tiempo de actualizacion

      //Modo Control 
      if (modo == 1){               
        CortoVel(joyX, joyY);                       // Conversion cordenadas a velocidad
        encoderread();                              // Lectura de la velocidad
        
        InputL = NL;                                // Variables a controlar
        InputR = NR;   
        SetpointL = SpeedL;                         // Referencias de velocidad
        SetpointR = SpeedR;
        myPIDL.Compute();                           // Calculo de los controladores
        myPIDR.Compute();      
  
        setMotorL(forwardL, OutputL);
        setMotorR(forwardR, OutputR);
      }
      
      //Modo Seguimiento
      if (modo == 2){
        if (count == 10){ 
          sing(S_look); 
          count = 0;
        }
        
        distancia = ultrasonic.Ranging(CM);                  //Medimos la distancia con el 
        
        if (distancia < 50){
          vel = 10*(distancia-separacion);
          SpeedL = abs(vel);
          SpeedR = abs(vel);
          forwardL = true;
          forwardR = true;
          if (vel<0){
            forwardL = false;
            forwardR = false;
          } 
        }  
        else{
          SpeedL = 0;
          SpeedR = 0;
        }
        
        encoderread();                              // Lectura de la velocidad
        
        InputL = NL;                                // Variables a controlar
        InputR = NR;   
        SetpointL = SpeedL;                         // Referencias de velocidad
        SetpointR = SpeedR;
        myPIDL.Compute();                           // Calculo de los controladores
        myPIDR.Compute();      
  
        setMotorL(forwardL, OutputL);
        setMotorR(forwardR, OutputR);
        
        count++;
      } 
    }
  }
  
  //Funciones de comunicacion
  void bluetooth() {
    if(Serial.available())  {                           // data received from smartphone
      delay(2);
      if (connection == 0) sing(S_connection); 
      connection = 1;
      
      cmd[0] =  Serial.read();  
      if(cmd[0] == STX)  {
        int i=1;
        while(Serial.available())  {
          delay(1);
          cmd[i] = Serial.read();
          if(cmd[i]>127 || i>7)                 break;     // Communication error
          if((cmd[i]==ETX) && (i==2 || i==7))   break;     // Button or Joystick data
          i++;
        }
        if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >
        else if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
      }
    } 
    sendBlueToothData();
  }
  
  void sendBlueToothData()  {
    static long previousMillis = 0;                             
    long currentMillis = millis();
    if(currentMillis - previousMillis > sendInterval) {   // send data back to smartphone
      previousMillis = currentMillis; 
  
  // Data frame transmitted back from Arduino to Android device:
  // < 0X02   Buttons state   0X01   DataField#1   0x04   DataField#2   0x05   DataField#3    0x03 >  
  // < 0X02      "01011"      0X01     "120.00"    0x04     "-4500"     0x05  "Motor enabled" 0x03 >    // example
  
      Serial.print((char)STX);                                             // Start of Transmission
      Serial.print(getButtonStatusString());  Serial.print((char)0x1);     // buttons status feedback
      Serial.print(distancia);                       Serial.print((char)0x4);     // datafield #1
      Serial.print(displayStatus);            Serial.print((char)0x5);     // datafield #2
      //Serial.print("#3");                                         // datafield #3
      Serial.print((char)ETX);                                             // End of Transmission
    }  
  }
  
  String getButtonStatusString()  {
    String bStatus = "";
    for(int i=0; i<6; i++)  {
      if(buttonStatus & (B100000 >>i))      bStatus += "1";
      else                                  bStatus += "0";
    }
    return bStatus;
  }
  
  void getJoystickState(byte data[8])    {
    joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48);       // obtain the Int from the ASCII representation
    joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
    joyX = joyX - 200;                                                  // Offset to avoid
    joyY = joyY - 200;                                                  // transmitting negative numbers
  
    if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return;      // commmunication error
    
  // Your code here ...
      //Serial.print("Joystick position:  ");
      //Serial.print(joyX);  
      //Serial.print(", ");  
      //Serial.println(joyY); 
  }
  
  void getButtonState(int bStatus)  {
    switch (bStatus) {
  // -----------------  BUTTON #1  -----------------------
      case 'A':
        buttonStatus |= B000001;        // ON
        sing(S_mode1);
        modo = 1;
        displayStatus = "Cont";
        //Serial.println(displayStatus);
        //digitalWrite(ledPin, HIGH);
        break;
        case 'B':
        buttonStatus &= B111110;        // OFF
        modo = 0;
        displayStatus = "Reposo";
        setMotorL(forwardL, 0);
        setMotorR(forwardR, 0);
        
        //Serial.println(displayStatus);
        //digitalWrite(ledPin, LOW);
        break;
  
  // -----------------  BUTTON #2  -----------------------
      case 'C':
        buttonStatus |= B000010;        // ON
        sing(S_mode2);
        modo = 2;
        displayStatus = "Seg";
        //Serial.println(displayStatus);
        break;
      case 'D':
        buttonStatus &= B111101;        // OFF
        modo = 0;
        displayStatus = "Reposo";
        setMotorL(forwardL, 0);
        setMotorR(forwardR, 0);
        //Serial.println(displayStatus);
        break;
  }
}  
  //Funciones de interrupcion
  void counterL(){
      nL++;     // Incrementa los impulsos
  }
   
  void counterR(){
      nR++;     // Incrementa los impulsos
  } 
  
  //Funcion lectura velocidad
  void encoderread(){
      
    detachInterrupt(0);        //Desconecta la interrupcion
    detachInterrupt(1);    
    NL = (nL*30)/((millis()-timeold)/1000.0*S);   // Calcula las revoluciones por minuto [RPM].
    NR = (nR*30)/((millis()-timeold)/1000.0*S);
    
    nL = 0;                                       // Inicializa la cuenta de pulsos
    nR = 0;
    timeold = millis();                           //  Guarda el tiempo actual.
    attachInterrupt(0, counterL, CHANGE);         // Reinicia la interrupción
    attachInterrupt(1, counterR, CHANGE);
  }
  
  //Funcion de conversion de cordenadas a velocidades de rueda
  void CortoVel(int X, int Y) {
    
    int a, b, c;
    
    double angulo = atan2(Y, X);
    angulo = (360*angulo)/(3.14159*2);
    if (angulo < 0) angulo = 360 + angulo;
      
    a = sqrt(square(X)+square(Y));
    b = -X + Y;
    c = X + Y;
    
    if (angulo >= 0 && angulo <= 90){
      SpeedL = a;
      SpeedR = b;
    }
    else if (angulo > 90 && angulo < 180){
       SpeedL = c;
       SpeedR = a;
    }
    else if (angulo >=180 && angulo <=270){
       SpeedL = -a;
       SpeedR = b;
    }  
    else if (angulo > 271 && angulo <360){
       SpeedL = c;
       SpeedR = -a;
    }
  
    if (SpeedL < 0){
      forwardL = false;
      SpeedL = Vmax*abs(SpeedL);
    }
    else{
      forwardL = true;
      SpeedL = Vmax*SpeedL;
    }
    
    if (SpeedR < 0){
      forwardR = false;
      SpeedR = Vmax*abs(SpeedR);
    }
    else{
      forwardR = true;
      SpeedR = Vmax*SpeedR;
    } 
  }
     
  //Funcion de control de los motores   
  void setMotorL(boolean forward, int motor_speed)
  {
     digitalWrite(motor1PinPlus, forward);
     digitalWrite(motor1PinMinus, !forward);
     analogWrite(motor1PinEnable, motor_speed);
  }
  
     void setMotorR(boolean forward, int motor_speed)
  {
     digitalWrite(motor2PinPlus, forward);
     digitalWrite(motor2PinMinus, !forward);
     analogWrite(motor2PinEnable, motor_speed);
  }
  
  //Funcion de sonido
  void tono(float noteFrequency, long noteDuration, int silentDuration){
    tone(speakerPin, noteFrequency);
    delay(noteDuration);
    noTone(speakerPin);
    delay(silentDuration);
  }
  
  void sing(int songName){
     switch(songName){
       case S_ON:
          tono(1318.51,50,100);
          tono(1567.98,50,80);
          tono(2349.32,300,0);
        break;
        
       case S_connection:
          tono(2637.02,50,50);
          tono(2349.32,50,50);
          tono(2093,50,50);
       break;
       
       case S_disconnection:
          tono(1318.51,50,50);
          tono(1174.66,50,50);
          tono(1318.51,50,50);
       break;
       
       case S_mode1:
          tono(659.26,50,30);
          tono(1318.51,55,25);
          tono(1760,60,10);
        break;
    
        case S_mode2:
          tono(659.26,50,30);
          tono(1760,55,25);
          tono(1318.51,50,10);
        break;
        
        case S_look:
          tono(1975.53,50,50);
        break;
   
        case S_lost:
          tono(659.26,50,30);
          tono(523.25,55,25);
          tono(587.33,200,10);
        break;
       }
 }
