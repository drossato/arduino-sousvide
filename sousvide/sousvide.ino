// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>

#define ONE_WIRE_BUS A1
#define RELAY_CONTROL 3
#define BT_RX A3
#define BT_TX A2

class PWM_Relay
{
    float period = 10000;
    unsigned long previousMillis = 0;
    float dutyCycle = 0; // 0.0 to 1.0
    int relayPin;

    int relayState = 0;
    void relayOn()
    {
      relayState = 1;
      digitalWrite(relayPin, LOW);
    }

    void relayOff()
    {
      relayState = 0;
      digitalWrite(relayPin, HIGH);
    }

  public:
    PWM_Relay(int pin)
    {
      relayPin = pin;
      pinMode(relayPin, OUTPUT);
    }
    
    void turnOff()
    {
      relayOff();
      setDutyCycle(0);
    }
    
    void setDutyCycle(float duty)
    {
      dutyCycle = constrain(duty, 0.0, 1.0);
    }

    float getDutyCycle()
    {
      return dutyCycle;
    }

    void update()
    {
      unsigned long currentMillis = millis();
      if ((unsigned long)(currentMillis - previousMillis) >= period)
      {
        previousMillis = currentMillis;
      }
      if (currentMillis - previousMillis < period * dutyCycle)
      {
        relayOn();
      }
      else
      {
        relayOff();
      }
    }
};

class PID_Relay
{
    float period = 20000;
    unsigned long previousMillis = 0;
    PWM_Relay *pwm;

    float kp = .7, ki = 0.1, kd = 0;
    float kb = ki;
    float lastError = 0, intError = 0, difError = 0;
    float output = 0;

    bool backCalcWindup = false;


  public:

    float getIntegral()
    {
      return intError;
    }

    PID_Relay(PWM_Relay *p)
    {
      pwm = p;
    }

    void update(float setPoint, float temperature)
    {
      unsigned long currentMillis = millis();
      if (previousMillis==0 || (unsigned long)(currentMillis - previousMillis) >= period)
      {
        previousMillis = currentMillis;
        float error = setPoint - temperature;
        difError = error - lastError;
        
        
        if(backCalcWindup) //back-calculation
        {
          intError += error + (kb/ki)*(constrain(output, 0.0, 1.0)-output); //intError will be multiplied by ki on the output calculation, not before integration
        }
        else //clamping
        {
          //intError = constrain(intError, -1.0/kp, 1.0/kp); //prevent wind-up
          if(error>0 && kp * error + ki * intError + kd * difError > 1.0)
            intError += 0;
          else
            intError += error;          
        }        

        output = kp * error + ki * intError + kd * difError;
        pwm->setDutyCycle(output);

        /*Serial.print(setPoint);
        Serial.print(";");
        Serial.print(temperature);
        Serial.print(";");
        Serial.print(error);
        Serial.print(";");
        Serial.print(intError);
        Serial.print(";");
        Serial.print(output);
        Serial.print(";");
        Serial.println("");*/

        lastError = error;

      }
    }
};

class FSM_Sensor
{
    float samplingPeriod = 1000;
    int sampleMeanSize = 20;

    int presentCycle = 0;
    float accumulator = 0;
    unsigned long previousMillis = 0;
    float lastValidSample = 0;
    float presentMeasure = 0;
    int lostMeasurements = 0;
    bool sensorOK = false;
    DallasTemperature *sensors;

  public:
    FSM_Sensor(DallasTemperature *s)
    {
      sensors = s;
    }
    void begin()
    {
      sensors->begin();
      presentMeasure = sensors->getTempCByIndex(0);
    }

    void update()
    {
      unsigned long currentMillis = millis();
      if ((unsigned long)(currentMillis - previousMillis) >= samplingPeriod / sampleMeanSize)
      {
        previousMillis = currentMillis;
        presentCycle++;
        sensors->requestTemperatures();
        float tempC = sensors->getTempCByIndex(0);
        if (tempC > 0) //invalid reading
          lastValidSample = tempC;
        else
        {
          lostMeasurements++;
          //Serial.println(tempC);
        }
        accumulator += lastValidSample;
        if (lostMeasurements > sampleMeanSize / 4)
          sensorOK = false;
      }
      
      if (presentCycle >= sampleMeanSize)
      {
        if (lostMeasurements < sampleMeanSize / 2)
          sensorOK = true;
        else
          sensorOK = false;
        presentMeasure = accumulator / presentCycle;
        presentCycle = 0;
        accumulator = 0;
        lostMeasurements = 0;
      }
    }

    float getMeasure()
    {
      return presentMeasure;
    }

    bool isMeasuring()  //if have initial measurement and is still connected
    {
      return sensorOK;
    }

    void setSamplingPeriod(float period)
    {
      samplingPeriod = period;
    }

    void setSampleMeanSize(int _size)
    {
      sampleMeanSize = _size;
    }
};

class FSM_Logger
{
    unsigned long previousMillis = 0;
    int loggingPeriod = 1000;

  public:
    FSM_Logger(int _loggingPeriod)
    {
      loggingPeriod = _loggingPeriod;
    }

    void update(float temp, float dutyCycle)
    {
      unsigned long currentMillis = millis();
      if ((unsigned long)(currentMillis - previousMillis) >= loggingPeriod)
      {
        previousMillis = currentMillis;
        Serial.print(temp);
        Serial.print(",");
        Serial.print(dutyCycle);
        Serial.print(";");
        Serial.println("");
      }
    }
};


OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
FSM_Sensor sensor(&sensors);
FSM_Logger logger(1000);
PWM_Relay relay(RELAY_CONTROL);
PID_Relay pid(&relay);
SoftwareSerial bluetooth(BT_TX, BT_RX);
int setpoint = 0;
bool turnedOn = false;


void setup(void)
{
  Serial.begin(2000000);
  sensor.begin();
  bluetooth.begin(9600);
}

void loop(void)
{
  sensor.update();
  if(sensor.isMeasuring() && turnedOn)
  {
    pid.update(setpoint, sensor.getMeasure());
    relay.update();
  }
  else
  {
    relay.turnOff();
  }
  logger.update(sensor.getMeasure(), relay.getDutyCycle());

  if (bluetooth.available()) {
    String s = bluetooth.readString();
    if(s[0]=='S')
    {
      setpoint = s.substring(1).toInt();
      bluetooth.println("S-OK");
    }
    if(s[0]=='L')
    {
      turnedOn = true;
      bluetooth.println("L-OK");
    }
    if(s[0]=='D')
    {
      turnedOn = false;
      bluetooth.println("D-OK");
    }
    if(s[0]=='T')
    {
      if(sensor.isMeasuring())
        bluetooth.println(sensor.getMeasure());
      else
        bluetooth.println("FAIL");
    }
  }  
}
