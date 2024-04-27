#define POT A5

#define FPWM1 10
#define FPWM2 11
#define FDIR1 12
#define FDIR2 13

#define BPWM1 6
#define BPWM2 9
#define BDIR1 5
#define BDIR2 8

//#define Speed 255
bool isSpeed=false;

unsigned int Speed=50;
unsigned int prevSpeed=0;
char ch1='S';
char ch2='S';
String str;
void setup() 
{

  Serial.begin(9600);

  pinMode(POT,INPUT);
  
  pinMode(FPWM1,OUTPUT);
  pinMode(FPWM2,OUTPUT);
  pinMode(FDIR1,OUTPUT);
  pinMode(FDIR2,OUTPUT);

  pinMode(BPWM1,OUTPUT);
  pinMode(BPWM2,OUTPUT);
  pinMode(BDIR1,OUTPUT);
  pinMode(BDIR2,OUTPUT);
}

void Forward(unsigned int Speed)
{
  downSpeed();
  digitalWrite(FDIR1,HIGH);
  //analogWrite(FPWM1,Speed);

  digitalWrite(FDIR2,HIGH);
  //analogWrite(FPWM2,Speed);


  digitalWrite(BDIR1,HIGH);
  //analogWrite(BPWM1,Speed);

  digitalWrite(BDIR2,HIGH);
  //analogWrite(BPWM2,Speed);
  upSpeed();
}

void Backward(unsigned int Speed)
{
  downSpeed();
  digitalWrite(FDIR1,LOW);
//  analogWrite(FPWM1,Speed);

  digitalWrite(FDIR2,LOW);
//  analogWrite(FPWM2,Speed);


  digitalWrite(BDIR1,LOW);
//  analogWrite(BPWM1,Speed);

  digitalWrite(BDIR2,LOW);
//  analogWrite(BPWM2,Speed);
  upSpeed();
}

void RotateRight(unsigned int Speed)
{
  downSpeed();
  digitalWrite(FDIR1,HIGH);
//  analogWrite(FPWM1,Speed);

  digitalWrite(FDIR2,LOW);
//  analogWrite(FPWM2,Speed);


  digitalWrite(BDIR1,HIGH);
//  analogWrite(BPWM1,Speed);

  digitalWrite(BDIR2,LOW);
//  analogWrite(BPWM2,Speed);
  upSpeed();
}

void RotateLeft(unsigned int Speed)
{
  downSpeed();
  digitalWrite(FDIR1,LOW);
//  analogWrite(FPWM1,Speed);

  digitalWrite(FDIR2,HIGH);
//  analogWrite(FPWM2,Speed);


  digitalWrite(BDIR1,LOW);
//  analogWrite(BPWM1,Speed);

  digitalWrite(BDIR2,HIGH);
//  analogWrite(BPWM2,Speed);
  upSpeed();
}

void Stop()
{
  for(int i=Speed;i>=0;i--)
  {
    analogWrite(FPWM1,i);
    analogWrite(FPWM2,i);
    analogWrite(BPWM1,i);
    analogWrite(BPWM2,i);
    delay(5);
  }
    digitalWrite(FDIR1,LOW);
//  analogWrite(FPWM1,Speed);

  digitalWrite(FDIR2,LOW);
//  analogWrite(FPWM2,Speed);


  digitalWrite(BDIR1,LOW);
//  analogWrite(BPWM1,Speed);

  digitalWrite(BDIR2,LOW);
}

void upSpeed()
{
  Speed=map(Speed,0,100,0,255);
  for(int i=prevSpeed;i<=Speed;i++)
  {
    analogWrite(FPWM1,i);
    analogWrite(FPWM2,i);
    analogWrite(BPWM1,i);
    analogWrite(BPWM2,i);
    delay(5);
  }
}

void downSpeed()
{
  Speed=map(Speed,0,100,0,255);
  for(int i=prevSpeed;i>=Speed;i--)
  {
    analogWrite(FPWM1,i);
    analogWrite(FPWM2,i);
    analogWrite(BPWM1,i);
    analogWrite(BPWM2,i);
    delay(5);
  }
}

void loop() 
{
  if(Serial.available())
  {
    delay(5);
    str=Serial.readString();
    if(str[0]=='F'||str[0]=='B'||str[0]=='R'||str[0]=='L')
    {
      ch2=str[0];
      switch(ch2)
      {
        case 'F':
          Forward(Speed);
          break;
        case 'B':
          Backward(Speed);
          break;
        case 'R':
          RotateRight(Speed);
          break;
        case 'L':
          RotateLeft(Speed);
          break;
        case 'S':
          Stop();
          break;
      }
    }
    else
    {
      isSpeed=true;
      prevSpeed=Speed;
      Speed=str.toInt();
      if(Speed>prevSpeed)
        upSpeed();
      else
        downSpeed();
    }
//    if(ch1!=ch2)
//    {
//      ch1=ch2;
      
//    }
  }
  /*unsigned int readNew=analogRead(POT);
  if(Speed!=readNew)
  {
    Speed=readNew;
    switch(ch)
    {
      case 'F':
        Forward(Speed);
        break;
      case 'B':
        Backward(Speed);
        break;
      case 'R':
        RotateRight(Speed);
        break;
      case 'L':
        RotateLeft(Speed);
        break;
    }
  }*/
  delay(15);
}
