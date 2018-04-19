#include <Servo.h>
#include <avr/pgmspace.h>

// **************************************************************************************** //
// Evolution Display Project                                                                //
// Developped by Félix Brousseau-Vaillancourt                                               //
//          and  Marc-Antoine Dumas                                                         //
// March 2017                                                                               //
// Features :                                                                               //
//      - Displays the Evolution vehicule speed using a servo-motor                         //
//      - (IN PROGRESS) Displays the vehicule RPM (shifting indicator) using 5 leds         //
//      - (NOT IMPLEMENTED) Dispays the vehicule current gear (approximately)               //
// **************************************************************************************** //

Servo myservo;  // range is from 0 km/h to 40 km/h. 180 degrees display. 1 km/h = 4.5 deg

// Considering a 0.478 meter wheel diameter, if the Evolution vehicule is cruising at 43.2 km/h
// the magnet fixed on the backwheel will complete 8 circles in a second (125 ms each).

const float radius = 0.239;                   // diameter = 0.478 meters, including rubber wheel
const float circumference = 2 * PI * radius;  // in meters

volatile bool hall_detected = 0;

// servo motor positions (degrees) for speed pointer position
float current_pos = 0;
float intermediary_pos = 0;
float target_pos = 0;
const byte _servo_pin = 4;

float speed_kmh = 0;
float current_speed = 0;
int current_rpm = 0;
String inString = "";
unsigned int current_gear = 0;

// hall sensor
const byte _hall_sensor_pin = 2;
const byte _hall_sensor_state_led_pin = 6;    // Hall sensor state
 
// time benching variables, debugging purposes
unsigned long current_time = 0;
unsigned long current_time_bench = 0;
unsigned long last_time = 0;
unsigned long last_time_pos = 0;
unsigned long last_time_bench = 0;
unsigned long time_difference = 0;
unsigned long time_difference_bench = 0;

//RPM lights
const int LEDV1=8;
const int LEDV2=9;
const int LEDB1=10;
const int LEDB2=11;
const int LEDR1=12;
const int LEDR2=13;

int digit1 = 0;
int digit2 = 0;

int rpm=0;  //motor RPM

const int arr_rpm[36] = { 2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,
                  3000,3100,3200,3300,3400,3500,3600,3700,3800,3900,
                  4000,4100,4200,4300,4400,4500,4600,4700,4800,4900,
                  5000,5100,5200,5300,5400,5500 };
                  
//Pour enregistrer l'affichage 7 segments dans la mémoire flash
PROGMEM const uint8_t Code7Segments[] = {
    //  7 Segments
    0b1111110, // 0
    0b0110000, // 1
    0b1101101, // 2
    0b1111001, // 3
    0b0110011, // 4
    0b1011011, // 5
    0b1011111, // 6
    0b1110000, // 7
    0b1111111, // 8
    0b1111011, // 9
};                  

PROGMEM const uint16_t arr_speeds_gear[11][36] = 
{   
    //Gear 1 
    {  568, 596, 625, 653, 681, 710, 738, 766, 795, 823, 852, 880, 908, 
    937, 965, 994, 1022, 1050, 1079, 1107, 1135, 1164, 1192, 1221, 1249, 
    1277, 1306, 1334, 1363, 1391, 1419, 1448, 1476, 1505, 1533, 1561 }, 
    //Gear 2
     {   734, 770, 807, 844, 880, 917, 954, 990, 1027, 1064, 1100, 1137, 
    1174, 1211, 1247, 1284, 1321, 1357, 1394, 1431, 1467, 1504, 1541, 
    1577, 1614, 1651, 1687, 1724, 1761, 1797, 1834, 1871, 1907, 1944, 
    1981, 2018 }, 
    //Gear 3                                         
    {   830, 871, 912, 954, 995, 1037, 1078, 1120, 1161, 1203, 1244, 
    1286, 1327, 1369, 1410, 1452, 1493, 1535, 1576, 1618, 1659, 1701, 
    1742, 1783, 1825, 1866, 1908, 1949, 1991, 2032, 2074, 2115, 2157, 
    2198, 2240, 2281 }, 
    //Gear 4                                         
    {   946, 993, 1040, 1088, 1135, 1182, 1230, 1277, 1324, 1372, 1419, 
    1466, 1513, 1561, 1608, 1655, 1703, 1750, 1797, 1844, 1892, 1939, 
    1986, 2034, 2081, 2128, 2176, 2223, 2270, 2317, 2365, 2412, 2459, 
    2507, 2554, 2601 }, 
    //Gear 5 
    {   1072, 1126, 1179, 1233, 1286, 1340, 1393, 1447, 1501, 1554, 
    1608, 1661, 1715, 1769, 1822, 1876, 1929, 1983, 2037, 2090, 2144, 
    2197, 2251, 2305, 2358, 2412, 2465, 2519, 2573, 2626, 2680, 2733, 
    2787, 2841, 2894, 2948 }, 
    //Gear 6                                         
    {    1222, 1283, 1344, 1405, 1466, 1527, 1588, 1649, 1710, 1771, 
    1832, 1894, 1955, 2016, 2077, 2138, 2199, 2260, 2321, 2382, 2443, 
    2504, 2565, 2627, 2688, 2749, 2810, 2871, 2932, 2993, 3054, 3115, 
    3176, 3237, 3298, 3360 }, 
    //Gear 7              
    {    1392, 1461, 1531, 1601, 1670, 1740, 1809, 1879, 1949, 2018, 
    2088, 2157, 2227, 2297, 2366, 2436, 2505, 2575, 2645, 2714, 2784, 
    2853, 2923, 2993, 3062, 3132, 3201, 3271, 3340, 3410, 3480, 3549, 
    3619, 3688, 3758, 3828 }, 
    //Gear 8                                         
    {    1575, 1654, 1733, 1811, 1890, 1969, 2048, 2126, 2205, 2284, 
    2363, 2441, 2520, 2599, 2678, 2756, 2835, 2914, 2993, 3071, 3150, 
    3229, 3308, 3386, 3465, 3544, 3623, 3701, 3780, 3859, 3938, 4016, 
    4095, 4174, 4253, 4331 }, 
    //Gear 9              
    {    1796, 1886, 1975, 2065, 2155, 2245, 2335, 2424, 2514, 2604, 
    2694, 2784, 2873, 2963, 3053, 3143, 3233, 3322, 3412, 3502, 3592, 
    3682, 3771, 3861, 3951, 4041, 4130, 4220, 4310, 4400, 4490, 4579, 
    4669, 4759, 4849, 4939}, 
    //Gear 10                                         
    {    2034, 2136, 2237, 2339, 2441, 2542, 2644, 2746, 2848, 2949, 
    3051, 3153, 3254, 3356, 3458, 3559, 3661, 3763, 3864, 3966, 4068, 
    4170, 4271, 4373, 4475, 4576, 4678, 4780, 4881, 4983, 5085, 5187, 
    5288, 5390, 5492, 5593}, 
    //Gear 11              
    {    2319, 2435, 2551, 2667, 2783, 2899, 3015, 3131, 3247, 3363, 
    3479, 3595, 3711, 3827, 3943, 4059, 4175, 4291, 4407, 4523, 4639, 
    4755, 4871, 4987, 5103, 5219, 5335, 5451, 5567, 5683, 5799, 5915, 
    6031, 6146, 6262, 6378}                                         
};

const int SEG_A=A0;
const int SEG_B=A1;
const int SEG_C=A2;
const int SEG_D=A3;
const int SEG_E=A4;
const int SEG_F=A5;
const int SEG_G=7;
const int CA_DIGIT1=3;
const int CA_DIGIT2=5;

void hallDetectedInterrupt() {
    
    hall_detected = 1;

}

int getCurrentGear() {

    int gear = 0;
    int diff_rpm = 0;
    int lowest_diff_rpm_index=0;
    int lowest_diff_rpm=10000;
    
    float diff_speed = 0;
    int lowest_diff_speed_gear_index=0;
    int lowest_diff_speed=10000;
    
    for (int j = 0; j < 36; j++)
    {
        diff_rpm = abs(current_rpm - arr_rpm[j]);
        
        if (diff_rpm < lowest_diff_rpm)
        {
            lowest_diff_rpm = diff_rpm;
            lowest_diff_rpm_index = j;
        }
    }

    for (int i = 0; i < 11; i++)
    {
        float vitesse = (float) (pgm_read_word(&(arr_speeds_gear[i][lowest_diff_rpm_index])))/100;
        diff_speed = abs(current_speed - vitesse);
        
        /*Serial.print("i : ");
        Serial.println(i,1);
        Serial.print("current_speed : ");
        Serial.println(current_speed,1);
        Serial.print("diff_speed : ");
        Serial.println(diff_speed,1);*/
        if ( diff_speed < lowest_diff_speed)
        {
            lowest_diff_speed = diff_speed;
            lowest_diff_speed_gear_index = i;
        }
    }              

    /*Serial.print("speed_kmh : ");
    Serial.println(speed_kmh,1);
    Serial.print("current_rpm : ");
    Serial.println(current_rpm,1);
    Serial.print("lowest_diff_rpm_index : ");
    Serial.println(lowest_diff_rpm_index,1);
    Serial.print("lowest_diff_speed_gear_index : ");
    Serial.println(lowest_diff_speed_gear_index,1);*/
    gear = lowest_diff_speed_gear_index + 1;
    return gear;

}

void setup() {

    // Serial is only for development purposes, REMOVE in actual release
    Serial.begin(115200);             // setting a fast baud rate is required to keep the loop fast
    myservo.attach(4);                // attaches the servo on pin 9 to the servo object
    pinMode(_hall_sensor_state_led_pin, OUTPUT);
    pinMode(_hall_sensor_pin, INPUT_PULLUP);
    // Interrupt is called when the hall sensor value rises
    attachInterrupt(digitalPinToInterrupt(_hall_sensor_pin), hallDetectedInterrupt, RISING);

    pinMode(LEDV1, OUTPUT);
    pinMode(LEDV2, OUTPUT);
    pinMode(LEDB1, OUTPUT);
    pinMode(LEDB2, OUTPUT);
    pinMode(LEDR1, OUTPUT);
    pinMode(LEDR2, OUTPUT);

    pinMode(SEG_A, OUTPUT);
    pinMode(SEG_B, OUTPUT);
    pinMode(SEG_C, OUTPUT);
    pinMode(SEG_D, OUTPUT);
    pinMode(SEG_E, OUTPUT);
    pinMode(SEG_F, OUTPUT);
    pinMode(SEG_G, OUTPUT);
    pinMode(CA_DIGIT1, OUTPUT);
    pinMode(CA_DIGIT2, OUTPUT);

    speed_kmh = 0;
    displayGear(0);
    
}

void hallDetected() {
    
    current_time = millis();

}

int getRPM() {
    
    while (Serial.available()) {
        delay(3);  //delay to allow buffer to fill 
        if (Serial.available() >0) {
          char c = Serial.read();  //gets one byte from serial buffer
          inString += c; //makes the string readString
        } 
      }
    rpm = inString.toInt();
    inString = "";

    if (rpm >= 1 && rpm < 600){
        digitalWrite(LEDV1, LOW); 
        digitalWrite(LEDV2, LOW); 
        digitalWrite(LEDB1, LOW);
        digitalWrite(LEDB2, LOW);
        digitalWrite(LEDR1, LOW); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 600 && rpm < 1500){
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, LOW); 
        digitalWrite(LEDB1, LOW);
        digitalWrite(LEDB2, LOW);
        digitalWrite(LEDR1, LOW); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 1500 && rpm < 3000)  { 
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, HIGH); 
        digitalWrite(LEDB1, LOW); 
        digitalWrite(LEDB2, LOW);
        digitalWrite(LEDR1, LOW); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 3000 && rpm < 3500)  {
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, HIGH); 
        digitalWrite(LEDB1, HIGH); 
        digitalWrite(LEDB2, LOW);
        digitalWrite(LEDR1, LOW); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 3500 && rpm < 4000)  {
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, HIGH); 
        digitalWrite(LEDB1, HIGH); 
        digitalWrite(LEDB2, HIGH);
        digitalWrite(LEDR1, LOW); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 4000 && rpm < 4500) {
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, HIGH); 
        digitalWrite(LEDB1, HIGH); 
        digitalWrite(LEDB2, HIGH);
        digitalWrite(LEDR1, HIGH); 
        digitalWrite(LEDR2, LOW); 
    }
    else if (rpm >= 4500)  {
        digitalWrite(LEDV1, HIGH); 
        digitalWrite(LEDV2, HIGH); 
        digitalWrite(LEDB1, HIGH); 
        digitalWrite(LEDB2, HIGH);
        digitalWrite(LEDR1, HIGH); 
        digitalWrite(LEDR2, HIGH); 
    }
    
    return rpm;
}

float evalSpeed(unsigned long &last_time, unsigned long current_time) {
    
    time_difference = current_time - last_time;
    speed_kmh = (3600 * circumference) / (float)time_difference;    // Do some maths
    //Serial.print("time_difference : ");
    //Serial.println(time_difference);
    last_time = current_time;        
    if (speed_kmh > 120)
        speed_kmh = 0;
    //Serial.print("speed_kmh : ");
    //Serial.println(speed_kmh,1);     
    return speed_kmh;

}

void displayGearDigit1(int digit1) {

    digitalWrite(CA_DIGIT1, HIGH);
    
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<6)) digitalWrite(SEG_A, 0);
    else digitalWrite(SEG_A, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<5)) digitalWrite(SEG_B, 0);
    else digitalWrite(SEG_B, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<4)) digitalWrite(SEG_C, 0);
    else digitalWrite(SEG_C, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<3)) digitalWrite(SEG_D, 0);
    else digitalWrite(SEG_D, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<2)) digitalWrite(SEG_E, 0);
    else digitalWrite(SEG_E, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<1)) digitalWrite(SEG_F, 0);
    else digitalWrite(SEG_F, 1);
    if(pgm_read_byte(&Code7Segments[digit1]) & (1<<0)) digitalWrite(SEG_G, 0);
    else digitalWrite(SEG_G, 1);
    delay(2);
    
    digitalWrite(CA_DIGIT1, LOW);

}

void displayGearDigit2(int digit2) {

    digitalWrite(CA_DIGIT2, HIGH);
    
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<6)) digitalWrite(SEG_A, 0);
    else digitalWrite(SEG_A, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<5)) digitalWrite(SEG_B, 0);
    else digitalWrite(SEG_B, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<4)) digitalWrite(SEG_C, 0);
    else digitalWrite(SEG_C, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<3)) digitalWrite(SEG_D, 0);
    else digitalWrite(SEG_D, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<2)) digitalWrite(SEG_E, 0);
    else digitalWrite(SEG_E, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<1)) digitalWrite(SEG_F, 0);
    else digitalWrite(SEG_F, 1);
    if(pgm_read_byte(&Code7Segments[digit2]) & (1<<0)) digitalWrite(SEG_G, 0);
    else digitalWrite(SEG_G, 1);
    
    delay(2);
    digitalWrite(CA_DIGIT2, LOW);
}


void displayGear(int gear) {

    switch (gear){
        case 0: {
            displayGearDigit1(0); 
            displayGearDigit2(0); break;
        }
        case 1: {
            displayGearDigit1(1); 
            displayGearDigit2(0); break;
        }
        case 2: {
            displayGearDigit1(2); 
            displayGearDigit2(0); break;
        }
        case 3: {
            displayGearDigit1(3); 
            displayGearDigit2(0); break;
        }
        case 4: {
            displayGearDigit1(4); 
            displayGearDigit2(0); break;
        }
        case 5: {
            displayGearDigit1(5); 
            displayGearDigit2(0); break;
        }
        case 6: {
            displayGearDigit1(6); 
            displayGearDigit2(0); break;
        }
        case 7: {
            displayGearDigit1(7); 
            displayGearDigit2(0); break;
        }
        case 8: {
            displayGearDigit1(8); 
            displayGearDigit2(0); break;
        }
        case 9: {
            displayGearDigit1(9); 
            displayGearDigit2(0); break;
        }
        case 10: {
            displayGearDigit1(0); 
            displayGearDigit2(1); break;
        }
        case 11: {
            displayGearDigit1(1); 
            displayGearDigit2(1); break;
        }
    }
}

void loop() {
        
    displayGear(current_gear);
    
    // current_time_bench = millis();
    // Serial.print("current_time : ");
    // Serial.println(current_time,4);

    //Serial.print("hs_value : ");
    //Serial.println(hs_value);
    //if ( hs_value < 10) hs_bin_value = 1;
    //else hs_bin_value = 0;

    displayGear(current_gear);

    if (last_time_pos - current_time > 6000)
    {
        target_pos = 0;
    }
    if (abs(target_pos - current_pos) < 2)
    {
      if (target_pos>180) { target_pos = 180; }
      myservo.write(180-target_pos);
      last_time_pos = millis();
      current_pos = target_pos;
    }
    if (current_pos != target_pos)
    {
        //Serial.print("target_pos : ");
        //Serial.println(target_pos,1);
        intermediary_pos = current_pos + ((target_pos - current_pos)/30);
        if (intermediary_pos>180) { intermediary_pos = 180; }
        myservo.write(180-intermediary_pos);
        last_time_pos = millis();
        current_pos = intermediary_pos;
    }
    // hall_detected has been set during the previous interruption
    if (hall_detected)  {
        hallDetected();
        if (current_time != last_time)  {
            // 40 km/h * 4.5 deg => speed pointer 180 degres at 40 km/h
            current_speed = evalSpeed(last_time, current_time);  
            target_pos = current_speed * 4.5; // 0 to 40 km/h display range
            hall_detected = 0;
        }
    }
    
    displayGear(current_gear);
    
    int temp_rpm = getRPM();
    if (temp_rpm > 0)
        current_rpm = temp_rpm;

    displayGear(current_gear);
    
    current_gear = getCurrentGear();

    //last_time_bench = millis();
    //time_difference_bench = last_time_bench - current_time_bench;
    //Serial.print("last_time_bench : ");
    //Serial.println(last_time_bench,4);
    //Serial.println(time_difference_bench,4);
}

