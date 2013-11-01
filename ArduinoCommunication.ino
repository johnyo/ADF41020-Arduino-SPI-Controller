    int ledPin=13;
    int matlabData;
    String wordz;
     
    void setup() {
      pinMode(ledPin,OUTPUT);
      Serial.begin(9600);
    }
     
    void loop() {
      if(Serial.available()>0){ // if there is data to read
        matlabData=Serial.read(); // read data
        Serial.print(char(matlabData));
      }
    
//    wordz += String(char(matlabData));
//    }
//    
//    if ( wordz == "Command1"){
//      Serial.println();
//      Serial.println("Command1 Executed");
//      wordz = "";
//    }
//    
//    if ( wordz == "Command2"){
//      Serial.println();
//      Serial.println("Command2 Executed");
//      wordz = "";
//    }
//    
    
    }
