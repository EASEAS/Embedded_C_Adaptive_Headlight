#include <avr/io.h>
#include <util/delay.h>

#define MASK 1

int servo_count = 0;
int servo_max = 150;
int servo_point = 60;

int light_count = 0;
int light_max = 150;
int light_point = 30;

unsigned int current_state = 0;
unsigned int next_state = 0;
unsigned int state_count = 0;
unsigned int state_point = 20;
unsigned int state_max = 50; //5 seconds

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_pin()
{
  DDRD |= (MASK<<PD2);
  DDRD |= (MASK<<PD3); 
  DDRC &= ~(MASK<<PC1);
  SREG 	|= (MASK<<7);
  
  PORTD &= ~(MASK<<PD2);
  
}
void init_adc()
{
   // ADMUX |= 0x45;
  //AVcc with external capacitor at AREF (5V) pin
  ADMUX &= ~(MASK<<REFS1);
  ADMUX |= (MASK<<REFS0); 
  
  //ADLAR = 0 configuration
  ADMUX &= ~(MASK<<ADLAR);

  //ADC Enable
  ADCSRA |=(MASK<<ADEN);
}
int adc_read(int select)
{
  unsigned int adc_result=0;
  if (select == 0)
  {
    ADMUX &= ~(MASK<<MUX0);
    ADMUX &= ~(MASK<<MUX1);
    ADMUX &= ~(MASK<<MUX2);
    ADMUX &= ~(MASK<<MUX3);
  }else if (select == 1)
  {
    ADMUX |=  (MASK<<MUX0);
    ADMUX &= ~(MASK<<MUX1);
    ADMUX &= ~(MASK<<MUX2);
    ADMUX &= ~(MASK<<MUX3);
  }else if (select == 2 )
  {
    ADMUX &= ~(MASK<<MUX0);
    ADMUX |=  (MASK<<MUX1);
    ADMUX &= ~(MASK<<MUX2);
    ADMUX &= ~(MASK<<MUX3);
  }else if (select == 3)
  {
    ADMUX |=  (MASK<<MUX0);
    ADMUX |=  (MASK<<MUX1);
    ADMUX &= ~(MASK<<MUX2);
    ADMUX &= ~(MASK<<MUX3);
  }

	// start single conversion write ’1' to ADSC
  // wait for conversion to complete ADSC becomes ’0' again till then, run loop continuously
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1 << ADSC));
  adc_result = ADCH;
  adc_result = adc_result << 8;
  adc_result = adc_result | ADCL;
  return (adc_result);

  
}

void init_timer()
{
  //reset clock registers for normal clock operation
  TCCR0A &= ~((MASK<<WGM00)| (MASK<<WGM01));
  TCCR0B &= ~(MASK<<WGM02);
  
  //timer a
  TIMSK0 |= (1<<TOIE0);
}


void set_timer()
{
  /*set source of clock registers. 
  *	This will start the clock
  */
  TCCR0B |= (MASK<<CS00);
  //TCCR0B &= ~(MASK<<CS00);
  
  //TCCR0B |= (MASK<<CS01);
  TCCR0B &= ~(MASK<<CS01);
  
  //TCCR0B |= (MASK<<CS02);
  TCCR0B &= ~(MASK<<CS02);

}
ISR(TIMER0_OVF_vect)
{  
  	//update headlight servo angle
	servo_count++;
	if (servo_count >= servo_point && servo_count < servo_max){
 		PORTD |= (MASK<<PD2);
  	}else if (servo_count >= servo_max){
    	PORTD &= ~(MASK<<PD2);
      	servo_count = 0;
  	}
  
  	//update headlight power
  	light_count++;
	if (light_count >= light_point && light_count < light_max){
 		PORTD |= (MASK<<PD3);
  	}else if (light_count >= light_max){
    	PORTD &= ~(MASK<<PD3);
      	light_count = 0;
  	}
  
  	//update headlight state
  	if (current_state != next_state)
    {	
      	state_count++;

      	if (state_count >= state_max){

 			current_state = next_state; 
          	state_count = 0;
        }

    }
      	
  	set_timer();
}
//determine if speed is in range for state transition
void determine_state(unsigned int speed)
{
  	if (speed < 50 && next_state != 1)
    {
      	Serial.println("state 1 started");
      	next_state = 1;
      	state_count = 0;
    }else if (speed >= 50 && speed < 100 && next_state != 2)
    {
      Serial.println("state 2 started");
      	next_state = 2;
      	state_count = 0;
    }else if (speed > 100 && next_state != 3)
    {
      Serial.println("state 3 started");
      	next_state = 3;
      	state_count = 0;
    }
}
void set_light_mode()
{
  if (current_state == 0)
  {
    light_point = 250;
  }else if (current_state == 1)
  {
    light_point = 40;
  }else if (current_state == 2)
  {
    light_point = 80;
  }else if (current_state == 3)
  {
    light_point = 140;
  }
}
int main()
{
  Serial.begin(9600);
  init_pin();
  init_timer();
  init_adc();

  
  while(1)
  {
	int steering_angle = adc_read(0);
	int speed = adc_read(1);
   	servo_point = map(steering_angle,0,1023,30,84);
    //light_point = map(speed,0,1023,0,118);
	determine_state(speed);
    //Serial.println((double)servo_point/(double)servo_max);
    set_light_mode();
    Serial.println(current_state);
    //Serial.println(servo_point);
  }
  return 0;
}
