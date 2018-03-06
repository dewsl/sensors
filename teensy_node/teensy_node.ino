#include <FlexCAN.h>
#include <kinetis_flexcan.h>

// FlexCAN CANReceiver(40000);

#define OKSTR "OK"
#define ERRORSTR "ERROR"
#define AT "AT"
#define ATCMDTRUE "ATE"

int led = 13;

static CAN_message_t msg;
// static uint8_t hex[17] = "0123456789abcdef";
bool ate=true;

void setup (void){
	while( !Serial );
	Serial.begin(9600);
	Serial.println(F("CAN Not Receiving"));
	Can0.begin(40000);
	// Can0.setNumTxBoxes(1);
  // CANReceiver.begin();
	pinMode(led,OUTPUT);
	digitalWrite(led,HIGH);

}

void loop(void) {
  // while (!Can0.available());
  if (Can0.read(msg)){
    Serial.print("ID seen: "); 
    Serial.println(msg.id);
  }
  delay(100);

}


void getATCommand(){
  String serial_line, command;
  int i_equals = 0;
  
  do{
    serial_line = Serial.readStringUntil('\r\n');
  } while(serial_line == "");
  serial_line.toUpperCase();
  serial_line.replace("\r","");

  if (ate) Serial.println(serial_line);
    i_equals = serial_line.indexOf('=');
  if (i_equals == -1) command = serial_line;
  else command = serial_line.substring(0,i_equals);
  
  if (command == AT){
    Serial.println(OKSTR);
  } else if (command == ATCMDTRUE){
    ate = true;
    Serial.println(OKSTR);
  } else if(command == "ATCANSEND" ){
    send_frame();
    Serial.println(OKSTR);
  }  else if(command == "ATSNIFF" ){
    listen_for_frame();
    Serial.println(OKSTR);
  }
  else{
    Serial.println(ERRORSTR);
  }

}

void getArguments(String at_cmd, String *arguments){
  int i_from = 0, i_to = 0, i_arg = 0;
  bool f_exit = true;
  String sub;

  i_from = at_cmd.indexOf('=');

  do{
    i_to = at_cmd.indexOf(',',i_from+1);
    if (i_to < 0){
      sub = at_cmd.substring(i_from+1);
      f_exit = false;
    }
    else sub = at_cmd.substring(i_from+1,i_to);

    arguments[i_arg] = sub;
    i_from = i_to;
    i_arg += 1;

  } while(f_exit);
}

void send_frame(){
  CAN_message_t msg;
  msg.ext = 1;
  msg.id = 2;
  msg.len = 8;
  msg.buf[0] = 0;
  msg.buf[1] = 1;
  msg.buf[2] = 2;
  msg.buf[3] = 3;
  msg.buf[4] = 4;
  msg.buf[5] = 5;
  msg.buf[6] = 6;
  msg.buf[7] = 7;
  Can0.write(msg);
}

void listen_for_frame(){
  CAN_message_t msg;
  while (!Can0.available());
  Can0.read(msg);
  Serial.print("ID seen: "); 
  Serial.print(msg.id);
}