const double VCC = 3.3;             // NodeMCU on board 3.3v vcc
const double R2 = 10000;            // 10k ohm series resistor
const double adc_resolution = 1023*3.3; // 10-bit adc

// const double A = 0.001129148;   // thermistor equation parameters
// const double B = 0.000234125;
// const double C = 0.0000000876741; 
const double A = 0.8272069482e-3;   // thermistor equation parameters
const double B = 2.087897328e-4;
const double C = 0.8062131944e-7; 

void setup() {
  Serial.begin(9600);  /* Define baud rate for serial communication */
}

void loop() {
  double Vout, Rth, temperature, adc_value; 

  adc_value = analogRead(A0);
  Vout = (adc_value * VCC) / adc_resolution;
  Rth = (VCC * R2 / Vout) - R2;
  Serial.println(" adc value");
  Serial.println(adc_value);
  Serial.println("rth");
  Serial.println(Rth);

/*  Steinhart-Hart Thermistor Equation:
 *  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
 *  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

  temperature = temperature - 273.15;  // Temperature in degree celsius
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" degree celsius");
  delay(500);
}

