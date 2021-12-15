int PRINT_MODE = 0; // printing mode
                    // 1 - prints for serial monitor
                    // 0 - prints for python
                    // 2 - prints for copy-paste-save_as-.csv
#define JIG 1 // 0 - if test jig is not available ( homebase functions omitted )
              // 1 - if test jig is available ( homebase functions working )
              
#define MOVE_DELAY 2000

#define MOTOR1OFFSET -24 // home switch calibration
#define MOTOR2OFFSET -88 //home switch calibration


// Define two steppers and the pins they will use
AccelStepper stepper1(AccelStepper::DRIVER, 3,6);
AccelStepper stepper2(AccelStepper::DRIVER, 7, 10);

int pos = 400;
int away1=0;
int away2=0;
int homebase=0;
int serialID=0;

//stepper_move(move right-left, move up-down)
void stepper_move(int pos1, int pos2){
  stepper1.setCurrentPosition(0);
  stepper1.moveTo(pos1);
  while (stepper1.currentPosition() != pos1){
    stepper1.run();
  }
  
  stepper2.setCurrentPosition(0);
  stepper2.moveTo(pos2);
  while (stepper2.currentPosition() != pos2){
    stepper2.run();
  } 
}


void calib_init()
{
  pinMode(11, INPUT); 
  pinMode(2, INPUT);
  
  stepper1.setMaxSpeed(500);
  stepper2.setMaxSpeed(500);
  stepper1.setAcceleration(3000);
  stepper2.setAcceleration(3000);

  if (PRINT_MODE == 1) {Serial.println( "Motor Initialization Done");} 
}

void go_home_2() {
  if (JIG) {
    homebase = digitalRead(11);
    if (homebase == HIGH) {
            away2 = 1;}
      //stepper2.setSpeed(200);
      while (away2 == 0 ) {
        delay(3); // for stability daw
        homebase = digitalRead(11);
          if (homebase == HIGH) {
            away2 = 1; 
            Serial.println("done");
            delay(100);
            }
          else {
             stepper_move(0,-8);
             delay(200);
          } 
          //stepper_move(0,MOTOR2OFFSET);
      }
      delay(MOVE_DELAY);
      //away = 0;
      Serial.println("go home 2");
  }
  
}

void go_home_1() {
  if (JIG) {
    homebase = digitalRead(2);
    if (homebase == HIGH) {
            away1 = 1;}
    //Serial.println(JIG);
      //motor1.setSpeed(MOTORSPEED1);
      while (away1 == 0 ) {
        delay(3);
        homebase = digitalRead(2);
          if (homebase == HIGH) {away1 = 1; Serial.println("done");}
          else {
            stepper_move(-8,0);
            delay(200);
          } 
        //stepper_move(MOTOR1OFFSET,0);
      }

      delay(MOVE_DELAY);
      //away = 0;
      Serial.println("go home 1");
  }
}

void go_home_adis(){
  float *data = (float*)malloc(sizeof(int)*3);
  float *deg = (float*)malloc(sizeof(int)*3);
  int step1, step2;
  float ref1, ref2;
  int home_pos = 0;
  do{
    data = get_data_polling(150,65535);
    deg = convert(data);      
    
    ref1=0.0;
    step1=int(4.44*(deg[1]-ref1));
    
    stepper_move(step1,0); 
    
    if (deg[0]<0) {ref2=-90.0; step2=int(4.44*(deg[2]-ref2));}
    else {ref2=-90.0; step2=-abs(int(4.44*(deg[2]-ref2)));}
    
    stepper_move(0,step2); 
    if (print_mode){
      Serial.print("--");
      Serial.print("data : ");
      Serial.print(data[0]);
      Serial.print(",");
      Serial.print(data[1]);
      Serial.print(",");
      Serial.println(data[2]);

      Serial.print("--");
      Serial.print("angle : ");
      Serial.print(deg[0]);
      Serial.print(",");
      Serial.print(deg[1]);
      Serial.print(",");
      Serial.println(deg[2]);
      
      Serial.print("--");
      Serial.println(step1);
      Serial.print("--");
      Serial.println(step2);    
      }
    if (step1 ==0 && step2 ==0 ){home_pos = 1;}
    delay(2000);
  }
  while (home_pos==0);
  Serial.println("--go home done!");
  Serial.println("&&");
  
}
