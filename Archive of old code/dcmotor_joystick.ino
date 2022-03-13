int button=2;
int sw;
const int x_axis = 0;
const int y_axis = 1;
const int motor1 = 5;
const int motor2 = 6;
int x_hiz=0;
int y_hiz=0;
int a=0;

void setup() {

  Serial.begin(9600);
pinMode(button, INPUT);
digitalWrite(button,HIGH);
pinMode(motor1,OUTPUT);
pinMode(motor2,OUTPUT);
}

void loop() {
 sw = digitalRead(button);
if(sw==LOW)
          {
          a++;
          if (a==1){while(1){            
                 x_hiz= analogRead(x_axis);
                 x_hiz= map(x_hiz,0,1023,255,0);
                 analogWrite(motor1, x_hiz);

                 y_hiz= analogRead(y_axis);
                 y_hiz= map(y_hiz,0,1023,255,0);
                 analogWrite(motor2, y_hiz);

                 Serial.println(x_hiz);
                 Serial.println(y_hiz);
                 Serial.println(a);
                 Serial.println("----------------------");
                  sw=digitalRead(button);
                if(sw==LOW){break;}
                 }
          }       

             
          if (a==2)
              {
                x_hiz=0;
                analogWrite(motor1, x_hiz);

                y_hiz=0;
                analogWrite(motor2, y_hiz);
                a=0; 

                Serial.println(x_hiz);
                 Serial.println(y_hiz);
                 Serial.println(a);
                 Serial.println("----------------------");
                }
        

    }
  }
 
  
  
