#include <MySensor.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <avr/wdt.h>
#include <SimpleTimer.h>
#include <CS_MQ7.h>
#include <math.h>


#define NDEBUG                        // enable local debugging information


#define SKETCH_NAME "CO meter"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"
#define NODE_ID 100 


#define CO_CHILD_ID 80
#define COPPM_CHILD_ID 81
#define RELAYCYCLES_CHILD_ID 90
#define BUZZER_CHILD_ID 70
#define TEMP1_CHILD_ID	60
#define AMBIENTTEMP_CHILD_ID	61


#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 
#define NIGHTMODE_CHILD_ID                    105
#define STOPCOSENSOR_CHILD_ID                 106


#define BUTTON1_PIN	7
#define RED_LED_PIN	3
#define GREEN_LED_PIN	5
#define BUZZER_PIN	17
#define ONE_WIRE_BUS              4      // Pin where dallase sensor is connected 

#define RADIO_RESET_DELAY_TIME 50 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10

#define CO_LEVEL_WARN  170//40
#define CO_LEVEL_ALARM 190//60

#define TEMPCHECK_TIME 15000

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

boolean boolRecheckSensorValues = false;

boolean bNightMode = false;

boolean bGatewayPresent = true;

boolean bGreenLedDisplayed = false;
boolean bWarnState =false;
boolean bAlarmState =false;
boolean bBuzzerActivated=false;
boolean bprevBuzzerState = false;

float fAmbientTemp = 0;
float lastTemp1 = -1;

unsigned long previousTempMillis=0;


OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 

CS_MQ7 MQ7(15);  // 12 = digital Pin connected to "tog" from sensor board
                     // 13 = digital Pin connected to LED Power Indicator

int CoSensorOutput = 0; //analog Pin connected to "out" from sensor board
int CoData = 0;         //analog sensor data
double lastCoData = 0;         //analog sensor data
double lastCoPPMData = 0;         //analog sensor data

unsigned long relayCyclesCount = 0;

unsigned long cyclecount=0;
unsigned long resultSum=0;
bool firstrun=false;


 double Ro = 9900;

SimpleTimer timer;
SimpleTimer timerRelayUsage;

MySensor sensor_node;

MyMessage msgCOLevel(CO_CHILD_ID, V_LEVEL);
MyMessage msgCOPPMLevel(COPPM_CHILD_ID, V_LEVEL);
MyMessage msgBuzzerState(BUZZER_CHILD_ID, V_STATUS);
MyMessage msgRelayCycles(RELAYCYCLES_CHILD_ID, V_DISTANCE);
MyMessage msgCOSensorState(STOPCOSENSOR_CHILD_ID, V_STATUS);
MyMessage msgNightModeStatus(NIGHTMODE_CHILD_ID, V_STATUS);


void setup() {

 Serial.begin(115200);

 //pinMode(A0, OUTPUT);


  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN,LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN,LOW);

	pinMode(BUZZER_PIN, OUTPUT);


  sensor_node.begin(incomingMessage, NODE_ID, false);

  sensor_node.wait(RADIO_RESET_DELAY_TIME);
  sensor_node.sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER);


    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(CO_CHILD_ID, S_AIR_QUALITY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(COPPM_CHILD_ID, S_AIR_QUALITY);   

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(BUZZER_CHILD_ID, S_LIGHT); 

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(RELAYCYCLES_CHILD_ID, S_DISTANCE); 

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(STOPCOSENSOR_CHILD_ID, S_LIGHT); 


    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(NIGHTMODE_CHILD_ID, S_LIGHT); 

    sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.present(AMBIENTTEMP_CHILD_ID, S_TEMP); 


        //reboot sensor command
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
    sensor_node.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 

    //reget sensor values
    sensor_node.wait(RADIO_RESET_DELAY_TIME);
  	sensor_node.present(RECHECK_SENSOR_VALUES, S_LIGHT); 


blinkGreenLed();

  	sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(NIGHTMODE_CHILD_ID, V_LIGHT);

  	sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(STOPCOSENSOR_CHILD_ID, V_LIGHT);

  	sensor_node.wait(RADIO_RESET_DELAY_TIME); 
    sensor_node.request(RELAYCYCLES_CHILD_ID, V_DISTANCE);



  timer.setInterval(5000, displayAlarm);
  timerRelayUsage.setInterval(600000, reportRelayUsage);

			MQ7.checkStopped = true;

      //Enable watchdog timer
  wdt_enable(WDTO_8S);


        #ifdef NDEBUG
          Serial.print(F("End setup "));
        #endif


}



void loop() {

  timer.run();
  timerRelayUsage.run();

checkCOLevel();

checkTemperature();

    //reset watchdog timer
    wdt_reset();   


}


void incomingMessage(const MyMessage &message) {

  if (message.isAck())
  {
    gotAck = true;
    return;
  }


    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
  

    if ( message.sensor == AMBIENTTEMP_CHILD_ID && strlen(message.getString())>0) {
         
         fAmbientTemp = message.getFloat();

     }

    if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;


         }

     }


    if ( message.sensor == RELAYCYCLES_CHILD_ID && strlen(message.getString())>0) {
         
         MQ7.relayCyclesCount = message.getULong();

     }


    if ( message.sensor == BUZZER_CHILD_ID  && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {
			digitalWrite(BUZZER_PIN, HIGH);
            bBuzzerActivated = true;
         }
         else
         {
			digitalWrite(BUZZER_PIN, LOW);			
            bBuzzerActivated = false;

            
         }

			reportBuzzerState();

     }



    if ( message.sensor == STOPCOSENSOR_CHILD_ID  && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {

            MQ7.checkStopped = true;
         }
         else
         {

            MQ7.checkStopped = false;

            
         }

                  //Отсылаем состояние переключателя
                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                        sensor_node.send(msgCOSensorState.set(MQ7.checkStopped?"1":"0"), true);
                          sensor_node.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false; 

     }



    if ( message.sensor == NIGHTMODE_CHILD_ID  && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {
            //digitalWrite(LED1_PIN, HIGH);
            //glowLed();
            bNightMode = true;
         }
         else
         {
         	//digitalWrite(LED1_PIN, LOW);
         	//fadeLed();
            bNightMode = false;

            
         }

                  //Отсылаем состояние переключателя
                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                        sensor_node.send(msgNightModeStatus.set(bNightMode?"1":"0"), true);
                          sensor_node.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false; 

     }


 

        return;      
} 



void checkCOLevel()
{

MQ7.CoPwrCycler();  


  /* your code here and below! */
  
  if(MQ7.currentState() == LOW){   //we are at 1.4v, read sensor data!



  	if ( !bGreenLedDisplayed && !bWarnState && !bAlarmState)
  	{
  		glowGreenLed();

  		bGreenLedDisplayed = true;

  	}

    CoData = analogRead(CoSensorOutput);
    //Serial.println(CoData);
    cyclecount++;
    resultSum = resultSum + CoData;
    firstrun=true;
  }
  else{                            //sensor is at 5v, heating time
    //Serial.println("sensor heating!");

    if (firstrun)
    {
 	//   b=round(p1*exp((c1-p2)/p3));

  		//blinkGreenLed();



  	if ( bGreenLedDisplayed  && !bWarnState && !bAlarmState )
  	{
  		fadeGreenLed();

  		bGreenLedDisplayed = false;

  	}

 	    #ifdef NDEBUG
		Serial.println(resultSum);
		Serial.println(cyclecount);
		#endif

    	double pop=resultSum/cyclecount;
		
 	    #ifdef NDEBUG
		Serial.println(pop);
		#endif


	if ( pop != lastCoData )
	{

			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgCOLevel.set((unsigned long)pop), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

		lastCoData = pop;
	}


/*
switch( MQ7.relayCyclesCount )
{
	case 3: pop = CO_LEVEL_WARN;
			break;
	case 7: pop = CO_LEVEL_ALARM;
			break;
	case 11: pop = 2;
			break;			
}

*/

	if ( pop >= CO_LEVEL_WARN && pop < CO_LEVEL_ALARM )
	{
		bAlarmState = false;
		bWarnState = true;

	}
	else if ( pop >= CO_LEVEL_ALARM )
	{
		bAlarmState = true;
		bWarnState = false;

	}
	else
	{
		bAlarmState = false;
		bWarnState = false;

	}

  //float v = map(pop, 0, 1024, 0, 1400);
   // Serial.print("MAP: ");
    //Serial.println(v);

   double ppm=round(10.0*exp((pop-339.0)/175.0));

   	if ( ppm != lastCoPPMData )
	{

			            iCount = MESSAGE_ACK_RETRY_COUNT;

			              while( !gotAck && iCount > 0 )
			                {
			      
			    	            sensor_node.send(msgCOPPMLevel.set((unsigned long)ppm), true);
			                    sensor_node.wait(RADIO_RESET_DELAY_TIME);
			                  iCount--;
			                 }

			                gotAck = false;	

		lastCoPPMData = ppm;
	}



//double ppm = 37143 * pow (pop, -3.178);

//double ppm = 100 * pow(((10000.0 / 9900.0) * ((1.4 / v) - 1.0)), -1.6);

    double Rs = ((1400.0 - pop) * 10000.0) / 1400.0;    
    //double Rs = (5000.0 - res) * (10000.0 / res);

 	#ifdef NDEBUG    
    Serial.print("Rs: ");
    Serial.println(Rs);
    //double Ro = Rs / 10000.0;
    //Serial.print("Ro: ");
    //Serial.println(Ro);
    //double ppm = pow( ((Rs/Ro)/22.07), (1.0/-0.0667) );
    Serial.print("PPM: ");
    Serial.println(ppm);
    Serial.print("Relay: ");
    Serial.println(MQ7.relayCyclesCount);    
    #endif

		firstrun=false;
		cyclecount=0;
		resultSum=0;   
    }


  }   

}



void reportBuzzerState()
{

                  //Отсылаем состояние переключателя
                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                        sensor_node.send(msgBuzzerState.set(bBuzzerActivated?"1":"0"), true);
                          sensor_node.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false; 


}


void displayAlarm()
{


if ( !MQ7.checkStopped == true )
{

		if ( bWarnState && !bAlarmState)
		{
			analogWrite(GREEN_LED_PIN, 0);

			blinkRed();

			digitalWrite(BUZZER_PIN, HIGH);
			bBuzzerActivated = true;
		   
		    if ( bGatewayPresent )
		    { 
		       sensor_node.wait(20);
		     }
		     else
		     {
		      delay(20);
		     }	
			digitalWrite(BUZZER_PIN, LOW);	
			bBuzzerActivated = false;

		}
		else if ( !bWarnState && bAlarmState )
		{
			analogWrite(GREEN_LED_PIN, 0);
			analogWrite(RED_LED_PIN, 255);	
			digitalWrite(BUZZER_PIN, HIGH);	
			bBuzzerActivated = true;
		}
		else
		{
			analogWrite(RED_LED_PIN, 0);	
			digitalWrite(BUZZER_PIN, LOW);
			bBuzzerActivated = false;
		}


		if ( bprevBuzzerState != bBuzzerActivated )
		{

			reportBuzzerState();
			bprevBuzzerState = bBuzzerActivated;	

		}

}
else
{

blinkRedGreen();

}


}


void blinkRedGreen()
{

	for (int i=0; i<=250; i+=25)
	{
	analogWrite(GREEN_LED_PIN, i);

    if ( bGatewayPresent )
    { 
       sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }

	}

	for (int i=250; i>=0; i-=25)
	{
	analogWrite(GREEN_LED_PIN, i);
    if ( bGatewayPresent )
    { 
	     sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }
	}

	for (int i=0; i<=250; i+=25)
	{
	analogWrite(RED_LED_PIN, i);

    if ( bGatewayPresent )
    { 
       sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }

	}

	for (int i=250; i>=0; i-=25)
	{
	analogWrite(RED_LED_PIN, i);
    if ( bGatewayPresent )
    { 
	     sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }
	}		

}

void blinkRed()
{


	analogWrite(RED_LED_PIN, 255);


    if ( bGatewayPresent )
    { 
       sensor_node.wait(60);
     }
     else
     {
      delay(60);
     }

	analogWrite(RED_LED_PIN, 0);

}

void blinkGreenLed()
{

	for (int i=0; i<=250; i+=25)
	{
	analogWrite(GREEN_LED_PIN, i);

    if ( bGatewayPresent )
    { 
       sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }

	}


	for (int i=250; i>=0; i-=25)
	{
	analogWrite(GREEN_LED_PIN, i);
    if ( bGatewayPresent )
    { 
	     sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }
	}

}


void glowGreenLed()
{

	for (int i=0; i<=250; i+=25)
	{
	analogWrite(GREEN_LED_PIN, i);

    if ( bGatewayPresent )
    { 
       sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }

	}
}


void fadeGreenLed()
{

	for (int i=250; i>=0; i-=10)
	{
	analogWrite(GREEN_LED_PIN, i);
    if ( bGatewayPresent )
    { 
	     sensor_node.wait(20);
     }
     else
     {
      delay(20);
     }
	}

}


void reportRelayUsage()
{

                  //Отсылаем состояние переключателя
                  iCount = MESSAGE_ACK_RETRY_COUNT;

                    while( !gotAck && iCount > 0 )
                      {
            
                        sensor_node.send(msgRelayCycles.set(MQ7.relayCyclesCount), true);
                          sensor_node.wait(RADIO_RESET_DELAY_TIME);
                        iCount--;
                       }

                      gotAck = false; 


}

/*

MQ7.checkStopped
fAmbientTemp
*/

void checkTemperature()
{


    unsigned long currentTempMillis = millis();
    if((currentTempMillis - previousTempMillis ) > TEMPCHECK_TIME ) {
        // Save the current millis
        previousTempMillis = currentTempMillis;

// Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)


 //   if ( bNoControllerMode != LOW )
//    { 
       sensor_node.wait(conversionTime);
 //    }
  //   else
  //   {
  //    delay(conversionTime);
  //   }

 float temperature = static_cast<float>(static_cast<int>(sensors.getTempCByIndex(0) * 10.)) / 10.;



         if (temperature != lastTemp1 && temperature != -127.00 && temperature != 85.00 ) {

          		#ifdef NDEBUG                
                Serial.print ("Temp: ");
          	    Serial.println (temperature); 
          	    #endif


		if ( temperature > fAmbientTemp ) 		
		{
			MQ7.checkStopped = false;

		}
		else
		{
			MQ7.checkStopped = true;

		}

            lastTemp1 = temperature;
        	} 




      }
}



