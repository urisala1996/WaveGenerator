/* PRIVATE PARAMETERS */
#define DEBUG 1
#define LOW_LEVEL 20
#define HIGH_LEVEL 1014
/*********************/

/* PUBLIC PARAMETERS */
#define AMPLITUDE 10
#define MIN_FREQ 5    //0.5 Hz
#define MAX_FREQ 260  //26 Hz
/*********************/

/* PIN DEFINITIONS */
int PLC_IN = A1;
/*********************/

/* PRIVATE VARIABLES */
int digital_sample = 0;
int analog_val = 0;
int frequency_out = 0;
int last_freq = 0;
int output_arr[1024];
char sbuffer[64];
/*********************/

void setup() {
  if(DEBUG){
    Serial.begin(9600);
  }
  Serial3.begin(9600);
  // Initialize pins

  //INITIAL DELAY WHEN TURNING ON THE SIGNAL GENERATOR
  delay(5000);
  
  //Set the output amplitude
  sprintf(sbuffer,"ba%01d.00\n",AMPLITUDE);
  Serial3.write(sbuffer);
  
  if(DEBUG) Serial.println(sbuffer);
  
  //Compute the output frequency map
  compute_frequencies(LOW_LEVEL,HIGH_LEVEL);

}

void loop() {

  //Get Value Read from PLC PIN
  analog_val = analogRead(PLC_IN);
  
  if(analog_val < LOW_LEVEL){
    frequency_out = 0;
  }else if (analog_val > HIGH_LEVEL){
    frequency_out = MAX_FREQ;
  }else{
    frequency_out = output_arr[analog_val];
  }

  sprintf(sbuffer,"bf%010d\n",frequency_out*10);
  if(last_freq != frequency_out){
    Serial3.write(sbuffer);
  }
  
  last_freq = frequency_out;
  
  if(DEBUG){
    Serial.print(analog_val);
    Serial.print("-");
    Serial.println(sbuffer);
  }

  delay(150);
}


void compute_frequencies(int in_low, int in_high){
    for(int i=0; i<1024; i++){
      output_arr[i] = map(i,in_low,in_high,MIN_FREQ,MAX_FREQ); 
    }
}
