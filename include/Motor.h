class Motor {

private:
  static int const SPEED_DEADZONE = 0; //<! If speed <= then assume stop
  static int const SPEED_MAX = 255;    //<! Maximum absolute speed
  static int const PWM_CRAWL = 50;     //<! If speed is not zero then don't move slower than this
  static int const PWM_MAX = 255;      //<! 100% duty cycle

  int const _direction1Pin;     //<! Used for L9110S, L298N
  int const _direction2Pin;     //<! Used for L298N only
  int const _speedPin;          //<! Used for L9110S, L298N

public:
  Motor(int const direction1Pin, int const direction2Pin, int const speedPin)
  : _direction1Pin(direction1Pin), _direction2Pin(direction2Pin), _speedPin(speedPin)
  {
    pinMode(_direction1Pin, OUTPUT);
    pinMode(_direction2Pin, OUTPUT);
    pinMode(_speedPin, OUTPUT);

    move(0);
  }

  Motor(int const directionPin, int const speedPin)
  : _direction1Pin(directionPin), _direction2Pin(-1), _speedPin(speedPin)
  {
    pinMode(_direction1Pin, OUTPUT);
    pinMode(_speedPin, OUTPUT);

    move(0);
  }
  
  void stop() {
    move(0);
  }

  void move(int speed);

  int get_fault() {
    // TODO:
    return 0; // No fault present
  }

private:
  int mapSpeedToPwm(int speed) {
    if (speed == 0)
      return 0;   // Stop is stop

    // Remap speed from crawl to full
    speed = constrain(speed, -SPEED_MAX, SPEED_MAX);  
    bool dir = (speed > 0);
    speed = map(abs(speed), 0, SPEED_MAX, PWM_CRAWL, PWM_MAX);
    return dir ? speed : -speed;
  }
  
};

#ifdef MOTOR_USING_L9110S
/* Update L9110S motor driver.
  See https://www.banggood.com/L9110S-H-Bridge-Stepper-Motor-Dual-DC-Driver-Controller-Module-p-914880.html
*/
void Motor::move(int speed)
{
  int pwm = mapSpeedToPwm(speed);
  if (abs(speed) <= SPEED_DEADZONE)          
  {
    // if inside deadzone stop
    digitalWrite(_direction1Pin, HIGH);
    analogWrite(_speedPin, PWM_MAX);
  }
  else if (speed < 0)                     
  {
    // if less than 0 then go backwards
    digitalWrite(_direction1Pin, HIGH);
    analogWrite(_speedPin, PWM_MAX-abs(pwm));
  }
  else {                               
    // otherwise go forwards
    digitalWrite(_direction1Pin, LOW);
    analogWrite(_speedPin, abs(pwm));
  }
}
#endif

#ifdef MOTOR_USING_L298N
/* Update L298N motor driver
  See https://www.banggood.com/Wholesale-L298N-Dual-H-Bridge-Stepper-Motor-Driver-Board-p-42826.html
*/
void Motor::move(int speed)
{
  int pwm = mapSpeedToPwm(speed);
  if (abs(speed) <= SPEED_DEADZONE) 
  {
    // if inside deadzone stop
    digitalWrite(_direction1Pin, HIGH);
    digitalWrite(_direction2Pin, HIGH);
    analogWrite(_speedPin, 0);
  }
  else
  {
    // if less than 0 then go backwards, otherwise go forwards
    digitalWrite(_direction1Pin, speed > 0);
    digitalWrite(_direction2Pin, speed < 0);
    analogWrite(_speedPin, abs(pwm));
  }
}
#endif  
