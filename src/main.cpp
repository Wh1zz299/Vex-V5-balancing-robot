#include "main.h"
#include <atomic>    


#define DISTANCE_PORT 5
#define DISTANCE_PORT1 15
#define MOTOR_PORT 1
#define MOTOR_PORT2 -2 // set negative for reversed

#define X 430 //Arm Length
#define Y 305 //Robot Height
#define RADIUS sqrt(Y^2+(1/2 * X)^2) //length from pivot to end of arm

#define KP 100 //too high makes it wobble //too low then nothing happens
#define KI 100 //for balancing it should be 10x the P in anything else it would be small
#define KD 100 //too high makes it jittery //too low makes it overreact(youll know what i mean)
#define DELAY 10//can be lowered but might cause system lag or overheating
#define SENSITIVITY 1 //leave for now

#define TARGET_ANGLE (0)


//from arduino opensource library 
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


void disabled() {
	lcd::set_text(5,"IM DEAD BTW");
}

std::atomic<long double> STARTTIME (0);
void startTimer() {
	STARTTIME = millis();
}

//gives the time since the start timer was called
double sinceStart() {
	return (millis()-STARTTIME.load());
}




// GYRO TASK
std::atomic<double> MESURED_ANGLE (M_PI_2); //angle of robot in radians
std::atomic<double> ANGULAR_VELOCITY (0);// angluar velocity at pivot
void calculate_fn(void* param) { 				// will be used just to find angle
	Distance distance_sensor(DISTANCE_PORT),distance_sensor1(DISTANCE_PORT1);

	double V1,V2,h1,h2;
	double top,top2,btm,btm2;
	double z,z2;
	double ANGLE;
	
	
while (true){
	startTimer();//shouldnt be needed unless vex runs program slow or too fast

	V1 = distance_sensor.get_object_velocity();
	V2 = distance_sensor1.get_object_velocity();

	ANGULAR_VELOCITY = (V1/RADIUS + (-V2)/RADIUS)/2; 

	h1 = distance_sensor.get_distance();
	h2 = distance_sensor1.get_distance();
	
	top = abs(Y-h2);
	btm = (X/2);

	z = atan(top/btm);

	top2 = abs(Y-h1);
	btm2 = (X/2);
	
	z2 = atan(top2/btm2);
	
	MESURED_ANGLE = z2;
	if (h2>h1){MESURED_ANGLE = z;}
	//detects right from left PERFECTLY
	
	//MATH MUST BE CHECKED!!!!!

//filters with both angles gets average
//this is a tested filter used for these kinda applications
//MESURED_ANGLE = ((0.966)*((ANGLE)/2)) + (0.034)*(ANGULAR_VELOCITY);

//MESURED_ANGLE = ANGLE; //(RIGHT_ANGLE+LEFT_ANGLE)/2;

lcd::set_text(3,std::to_string(MESURED_ANGLE*180/M_PI));
delay(DELAY); //delay to keep system alive
}
}

//PID TASK
std::atomic<double> MOTOR_OUT (0);
void PID_fn(void* ignore){
	double error;
	long double errorSum;
	double time = 0.02;
while (true){
	//MOTOR_OUT = (KP * MESURED_ANGLE) + (KD *ANGULAR_VELOCITY);// this is without integral and time
	
	error = TARGET_ANGLE/SENSITIVITY - MESURED_ANGLE/SENSITIVITY;//dived for less sensitivity 
	errorSum = errorSum + error;
	time = sinceStart();//shouldnt be needed unless vex runs program slow or too fast

	MOTOR_OUT = (KP*error) + (KI*errorSum*time) - KD*ANGULAR_VELOCITY/time; 

	//MOTOR_OUT = constrain((KP*error) + (KI*errorSum*time) - KD*ANGULAR_VELOCITY/time,-127,127);//for voltage

	lcd::set_text(4,std::to_string(MOTOR_OUT));
	delay(DELAY); //adjustable to be more responsive
}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	lcd::set_text(1, "Hello PROS User!");
	lcd::set_text(2,"Balancer by VVisdo");

	Task PID(PID_fn);
	delay(1);
 	Task Gyro(calculate_fn);
}

//this is where i will put motor control reading from PID task
void opcontrol() {
MotorGroup drive ({MOTOR_PORT,MOTOR_PORT2});
delay(1);

	while (true)
	{	
		//drive.move(MOTOR_OUT);
		drive.move_velocity(MOTOR_OUT);
		delay(DELAY);
	}
	


}
