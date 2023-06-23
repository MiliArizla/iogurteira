/*
Proportional temperature controller using a min delay of 2 seconds for actuaiting a relay
A lot of ASSEMBLY/REGISTRY comunication with the ATMega328p for educational pourposes ^^
*/


// Include necessary libraries
#include <OneWire.h>  
#include <DallasTemperature.h>


// We use a lot of volatiles because most of these values are 
// usefull in multiple scenarios of the code (for example:
// current temperature, control value, control booleans, etc.)
volatile bool converted;
volatile bool newValue;
volatile uint16_t value;
volatile float p_value = 15;
volatile float temp;
volatile uint16_t control;
volatile bool blocked = false;
volatile bool recently_blocked = false;

#define temp_wire 2 // Sensor port

OneWire oneWire(temp_wire); // Create one-wire object
DallasTemperature sensors(&oneWire); // Create temperature sensor object (inherits one wire object)

void setup()
{
  Serial.begin(9600); // Serial communications for debugging and data 
  sensors.begin(); // Start sensors

  // Select A0
  ADMUX |= (1<<REFS0);
  // Enable ADC; Enable Interrupt;
  ADCSRA |= (1<<ADEN)|(1<<ADSC)|(1<<ADIE);
  // Set trigger on TIMER1 match B
  ADCSRB |= (1<<ADTS2)|(1<<ADTS0);

  // Set PB1 to be an output (Pin9 Arduino UNO)
  DDRB |= (1 << PB1);
  // Clear Timer/Counter Control Registers
  TCCR1A = 0;
  TCCR1B = 0;
  // Set inverting mode
  TCCR1A |= (1 << COM1A1);
  // Set phase-correct PWM mode 
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13);
  // Set prescaler to 1024 and starts PWM
  TCCR1B |= (1 << CS12);
  TCCR1B |= (1 << CS10);
  // Set PWM frequency/top value
  ICR1 = 0xFFFF;
  // Set dutycicle 15625 -> 2 secs
  OCR1A = 15625;
  
  // Make a first reading
  sensors.requestTemperaturesByIndex(0);
  // Set lowest resolution -> Faster (.5 degrees)
  sensors.setResolution(9);
  // Enable interrupts
  sei();
}

void convert_desired_temp(float delta_temp) {
    if (delta_temp < 1.0L) {
      TCCR1A &= ~(1<<COM1A1); // Disable PWM
      PORTB &= ~(1<<PB1); // PIN Power Off
      control = 0x0000; // Min Dutycicle
      OCR1A = 15625; // Timer1 - 2 seconds 
      blocked = true; // Block Control Update
      TCNT1 = 0; // Reset Timer1
      TIMSK1 |= (1<<OCIE1A); // Enable MatchA Interrupt

    } else if (delta_temp > p_value) {
      TCCR1A &= ~(1<<COM1A1); // Disable PWM
      PORTB |= (1<<PB1); // PIN Power On
      control = 0xFFFF; // Max Dutycicle
      OCR1A = 15625; // Timer1 - 2 seconds
      blocked = true; // Block Control Update
      TCNT1 = 0; // Reset Timer1
      TIMSK1 |= (1<<OCIE1A); // Enable MatchA Interrupt
    } else {
      TCCR1A |= (1<<COM1A1); // Enable PWM
      control = uint16_t((pow(2,16)-2L*15625L -1)*delta_temp/(p_value - 1.0L) + 15625L); // Calculate proportional dutycicle
      OCR1A = control; // Update Dutycicle
    }
}

float convert_control(uint16_t control) {
    return 2*float(control - 1)/15625L; // Needs to be recalculated
}

void loop() {
  if (recently_blocked) {
    blocked = false; // Unblock it
    TIMSK1 &= ~(1<<OCIE1A); // Disable interrupts
    recently_blocked = false; // Reprepare trigger for unblocking
  }

  if (newValue & !blocked) {
    newValue = false; // No new ADC value to update
    float desired_temp = float(value) * 100L / 1023L; // Map desired temperature
    float delta_temp = desired_temp - temp; // Calculate temperature difference
    convert_desired_temp(delta_temp); // Update control

    // Optional printing 
    Serial.print(temp);
    Serial.print(" ");
    Serial.print(desired_temp);
    Serial.print(" ");
    Serial.println(convert_control(control));
  }

  if (sensors.getWaitForConversion()) { // If new temperature reading
    temp = sensors.getTempCByIndex(0); // Get temperature
    sensors.requestTemperaturesByIndex(0); // Ask for new temperature
  }
  
  if (converted) { // If has converted
    ADCSRA |= (1<<ADSC)|(1<<ADIE);   // Start converting voltage on A0
    converted = false; // Reanable trigger
  }
}

// When there is an ADC value
ISR(ADC_vect){ 
  value = ADC; // Read ADC value
  converted = true; // Trigger flag
  newValue = true;
}

ISR(TIMER1_COMPA_vect) { // When two seconds have passed 
  recently_blocked = true; // Unblock controller
}


//BTW this code could be reused and improved in many ways I hope someone uses it 
