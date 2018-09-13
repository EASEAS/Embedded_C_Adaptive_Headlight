#include <avr/io.h>
#include <util/delay.h>

#define MASK 1
#define ONE 1
int ADH_RightRange = 0;
int ADH_LeftRange = 0;
int ADH_servo_count = 0;
int ADH_servo_max = 150;
int ADH_servo_point = 60;

int ADH_light_count = 0;
int ADH_light_max = 150;
int ADH_light_point = 30;

unsigned int ADH_current_state = 0;
unsigned int ADH_next_state = 0;
unsigned long ADH_state_count = 0;

unsigned long ADH_state_max = 305000; //5 seconds

void init_pin()
{
  DDRD |= (MASK<<PD2);
  DDRD |= (MASK<<PD3); 
  DDRC &= ~(MASK<<PC1);
  SREG   |= (MASK<<7);
  
  PORTD &= ~(MASK<<PD2);
  
}
void init_adc()
{
 ADMUX &= ~(ONE<<REFS1);//0 source
  ADMUX |= (ONE<<REFS0); //1
  
  
  //Left-adjust OFF
  //ADMUX &= ~(ONE<<ADLAR);
  
  //ADC Enable
 // ADCSRA |= (ONE<<ADEN);
  ADCSRA=(1<<ADEN)|(1<<ADPS2)|(0<<ADPS1)|(1<<ADPS0); //Prescalar div factor =128
}
uint16_t adc_read(uint8_t ch)
{
   //Select ADC Channel ch 
   ADMUX=(ONE<<REFS0)|ch;
   //Start Single conversion
   ADCSRA|=(ONE<<ADSC);
   //Wait for conversion to complete
   while(!(ADCSRA & (ONE<<ADIF)));
   //Clear ADIF by writing one to it
   ADCSRA|=(ONE<<ADIF);
   return(ADC);
}

void init_timer()
{
  //reset clock registers for normal clock operation
  TCCR0A &= ~((MASK<<WGM00)|(MASK<<WGM01));
  TCCR0B &= ~(MASK<<WGM02);
  TCNT0 = 0;
  //timer a
  TIMSK0 |= (1<<TOIE0);

  set_timer();
}


void set_timer()
{
  /*set source of clock registers. 
  * This will start the clock
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
  ADH_servo_count++;
  if (ADH_servo_count >= ADH_servo_point && ADH_servo_count < ADH_servo_max){
      
    PORTD |= (MASK<<PD2);
    }else if (ADH_servo_count >= ADH_servo_max){
      PORTD &= ~(MASK<<PD2);
        ADH_servo_count = 0;
    }
  
    //update headlight power
    ADH_light_count++;
  if (ADH_light_count >= ADH_light_point && ADH_light_count < ADH_light_max){
    PORTD |= (MASK<<PD3);
    }else if (ADH_light_count >= ADH_light_max){
      PORTD &= ~(MASK<<PD3);
        ADH_light_count = 0;
    }
  
    //update headlight state
    if (ADH_current_state != ADH_next_state)
    { 
        ADH_state_count++;

        if (ADH_state_count >= ADH_state_max){
      //Serial.println("state changed");
      ADH_current_state = ADH_next_state; 
            ADH_state_count = 0;
        }

    }
        
    //set_timer();
}
//determine if speed is in range for state transition.
//starts timer for transition in 
void determine_state(unsigned int ADH_speed)
{
    if (ADH_current_state == 0){
      ADH_current_state = 1;
      ADH_next_state = 1;
    }
    if (ADH_speed < 50 && ADH_next_state != 1)
    {
        //Serial.println("state 1 started");
        ADH_next_state = 1;
        ADH_state_count = 0;
    }else if (ADH_speed >= 50 && ADH_speed < 100 && ADH_next_state != 2)
    {
      //Serial.println("state 2 started");
        ADH_next_state = 2;
        ADH_state_count = 0;
    }else if (ADH_speed > 100 && ADH_next_state != 3)
    {
      //Serial.println("state 3 started");
        ADH_next_state = 3;
        ADH_state_count = 0;
    }
}
void set_light_mode()
{
  if (ADH_current_state == 0)
  {
    ADH_light_point = 250;
  }else if (ADH_current_state == 1)
  {
    ADH_light_point = 120;
  }else if (ADH_current_state == 2)
  {
    ADH_light_point = 100;
  }else if (ADH_current_state == 3)
  {
    ADH_light_point = 80;
  }
}
void setRange()
{
  int ADH_center = 57;
  if (ADH_current_state == 1)
  {
   
    int ADH_offset = 8;
    ADH_LeftRange = ADH_center - ADH_offset;
    ADH_RightRange = ADH_center + ADH_offset;
  }else if (ADH_current_state == 2 || ADH_current_state == 3)
  {
      
    int ADH_offset = 8;
    ADH_LeftRange = ADH_center - ADH_offset;
    ADH_RightRange = ADH_center + 2*ADH_offset;
  }else{
    int ADH_offset = 0;
    ADH_LeftRange = ADH_center - ADH_offset;
    ADH_RightRange = ADH_center + ADH_offset;
  }
    
}
int main()
{
  init_pin();
  init_timer();
  init_adc();

  while(1)
  {
    
    int ADH_steering_angle = adc_read(0);
    int ADH_speed = adc_read(1);
  
    //convert steering angle into lerped
    setRange();
    ADH_servo_point = map(ADH_steering_angle,0,1023,0,150);
    
    //convert potentiometer reading into speed KMph
    ADH_speed = map(ADH_speed,0,1023,0,400);
    

    determine_state(ADH_speed);
    set_light_mode();

  }
  return 0;
}
