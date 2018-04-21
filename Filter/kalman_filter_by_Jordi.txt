//This is a demo of how to use Kalman filtering with a 300d/s gyroscope and accelerometers from wii nunchuck..
//Gyro: http://www.sparkfun.com/commerce/product_info.php?products_id=395
//I extreamly recommend use the gyro with a 150 d/s to increase resolution, and, if the idea
//of use it in a Helicopter cross ur mind, try to use the 75d/s, to detect minimal movements and God bless you.. 
//I don't remember the name or the source, but i extract this code from another complex job, and adapt it to Arduino,
//You can remplace the accelerometer for an analog one, is noisier but you will increase performance and reduce code... 
//By Jordi Munoz... //Si deseas la version en Espanol, no dudes en escribirme...   

#include <math.h>
#include <Wire.h>
/////////////////////////////////////////
#define NUMREADINGS 5 //Gyro noise filter
int readings[NUMREADINGS];                // the readings from the analog input (gyro)
int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
int average = 0;                          // the average
int inputPin =0;                          //Gyro Analog input
///////////////////////////////////////

float	dt	= .06; //( 1024.0 * 256.0 ) / 16000000.0; (Kalman)
int mydt = 20; //in ms.
static float		P[2][2] = { //(Kalman)
  { 
    1, 0   }
  , //(Kalman)
  { 
    0, 1   }
  ,//(Kalman)
}; //(Kalman)


/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
float			angle; //(Kalman)
float			q_bias; //(Kalman)
float			rate; //(Kalman)
float                   q_m; //(Kalman)

int joy_x_axis = 0; //NunChuck Joysticks potentiometers
int joy_y_axis = 0;
int ax_m=0;	//NunChuck X acceleration 
int az_m=0;	//NunChuck Z acceleration 
byte outbuf[6];		// array to store arduino output
int cnt = 0;            //Counter
unsigned long lastread=0;

/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
float	R_angle	= .3; //.3 deafault, but is adjusted externally (Kalman)


/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the acceleromter
 * relative to the gyros.
 */
static const float	Q_angle	= 0.001; //(Kalman)
static const float	Q_gyro	= 0.003; //(Kalman)

void nunchuck_init () //(nunchuck)
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52 (nunchuck)
  Wire.send (0x40);		// sends memory address (nunchuck)
  Wire.send (0x00);		// sends sent a zero. (nunchuck)
  Wire.endTransmission ();	// stop transmitting (nunchuck)
}
void setup()
{
  Wire.begin ();		// join i2c bus with address 0x52 (nunchuck)
  nunchuck_init (); // send the initilization handshake  
  beginSerial (38400);
  for (int i = 0; i < NUMREADINGS; i++)
    readings[i] = 0;                      // initialize all the readings to 0 (gyro average filter)
}
void send_zero () //(nunchuck)
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52 (nunchuck)
  Wire.send (0x00);		// sends one byte (nunchuck)
  Wire.endTransmission ();	// stop transmitting (nunchuck)
}
void loop()
{
  if((millis()-lastread) >= mydt) { // sample every dt ms -> 1000/dt hz.
    lastread = millis();
    total -= readings[index];               // subtract the last gyro reading
    readings[index] = analogRead(inputPin); // read from the gyro sensor
    total += readings[index];               // add the reading to the total
    index = (index + 1);                    // advance to the next index

    if (index >= NUMREADINGS)               // if we're at the end of the array...
      index = 0;                            // ...wrap around to the beginning

    average = (total / NUMREADINGS)-511;    // calculate the average of gyro input

    q_m=((average*(joy_x_axis*0.00392156862))/(180/PI)); //GYRO convertion to Radian and external correction with nunchuk joystick

    /* Unbias our gyro */
    const float		q = q_m - q_bias; //(Kalman)

    const float		Pdot[2 * 2] = {
      Q_angle - P[0][1] - P[1][0],	/* 0,0 */ //(Kalman)
      - P[1][1],		/* 0,1 */
      - P[1][1],		/* 1,0 */
      Q_gyro				/* 1,1 */
    };

    /* Store our unbias gyro estimate */
    rate = q; //(Kalman)

    /*
	 * Update our angle estimate
     	 * angle += angle_dot * dt
     	 *       += (gyro - gyro_bias) * dt
     	 *       += q * dt
     	 */
    angle += q * dt; //(Kalman)

    /* Update the covariance matrix */
    P[0][0] += Pdot[0] * dt; //(Kalman)
    P[0][1] += Pdot[1] * dt; //(Kalman)
    P[1][0] += Pdot[2] * dt; //(Kalman)
    P[1][1] += Pdot[3] * dt; //(Kalman)


    Wire.requestFrom (0x52, 6);	// request data from nunchuck

    while (Wire.available ()) //(NunChuck)
    {
      outbuf[cnt] = nunchuk_decode_byte (Wire.receive ());	// receive byte as an integer //(NunChuck)
      cnt++;//(NunChuck)
    }

    send_zero (); // send the request for next bytes
    // If we recieved the 6 bytes, then print them //(NunChuck)
    if (cnt >= 5) //(NunChuck)
    {
      print (); //(NunChuck)
    }
    cnt = 0; //(NunChuck)

    R_angle= (joy_y_axis+1)*0.0098039; //external adjust jitter of accelerometer with nunchuck joystick

    const float		angle_m = atan2( ax_m, az_m ); //(Kalman)
    const float		angle_err = angle_m - angle; //(Kalman)
    const float		C_0 = 1; //(Kalman)
    const float		PCt_0 = C_0 * P[0][0];  //(Kalman)
    const float		PCt_1 = C_0 * P[1][0]; //(Kalman)
    const float		E =R_angle+ C_0 * PCt_0; //(Kalman)
    const float		K_0 = PCt_0 / E; //(Kalman)
    const float		K_1 = PCt_1 / E; //(Kalman)	 	
    const float		t_0 = PCt_0; /* C_0 * P[0][0] + C_1 * P[1][0] (Kalman) */

    const float		t_1 = C_0 * P[0][1]; /* + C_1 * P[1][1]  = 0 (Kalman) */


    P[0][0] -= K_0 * t_0; //(Kalman)
    P[0][1] -= K_0 * t_1; //(Kalman)
    P[1][0] -= K_1 * t_0; //(Kalman)
    P[1][1] -= K_1 * t_1; //(Kalman)
    angle	+= K_0 * angle_err; //(Kalman)
    q_bias	+= K_1 * angle_err; //(Kalman)


    Serial.print(joy_y_axis); //Prints the adjust for accelerometer jitter
    Serial.print(" ");
    Serial.print(int(angle_m*57.295779513082)); //Prints the accelometer
    Serial.print(" ");
    Serial.print(int(angle*57.2957795130823)); //Prints degrees Acceleromter+Gyros+KalmanFilters
    Serial.print(" ");
    Serial.println(joy_x_axis); //Prints the Gyro adjusment


  }

}
void print ()//This is the function to decode nintendo wii nunchuck
{
  joy_x_axis = outbuf[0];
  joy_y_axis = outbuf[1];
  ax_m = (outbuf[2] * 2 * 2) -511; //Axis X Acceleromter
  //ay_m = (outbuf[3] * 2 * 2) -511; //Axis Y Acceleromter
  az_m = (outbuf[4] * 2 * 2) -511; //Axis Z Acceleromter

  // byte outbuf[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  // so we have to check each bit of byte outbuf[5]


  if ((outbuf[5] >> 2) & 1)
  {
    ax_m += 2;
  }
  if ((outbuf[5] >> 3) & 1)
  {
    ax_m += 1;
  }
  /*
  if ((outbuf[5] >> 4) & 1)
   {
   accel_y_axis += 2;
   }
   if ((outbuf[5] >> 5) & 1)
   {
   accel_y_axis += 1;
   }
   */
  if ((outbuf[5] >> 6) & 1)
  {
    az_m += 2;
  }
  if ((outbuf[5] >> 7) & 1)
  {
    az_m += 1;
  }


}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char
nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}