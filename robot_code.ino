#define IN1 11
#define IN2 13
#define ENA 5
#define IN3 12
#define IN4 10
#define ENB 3
#define num_sensors 5

int sensor_pins[num_sensors] = {A0, A1, A2, A3, A4};
int sValues[num_sensors];
int base_speed = 150;
int max_speed = 220;
int turn_speed = 90;
float kp = 0.08;
float kd = 0.2;
int prev_error = 0;
int last_side = 0;
int center_offset = 0;
bool calibrated = false;

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  for (int i = 0; i < num_sensors; i++) {
    pinMode(sensor_pins[i], INPUT);
  }
  Serial.begin(9600);
  
  // কোন ডিলে নেই - সুইচ দিলেই চালু
  calibrate_center();
}

void calibrate_center() {
  long sum = 0;
  int count = 0;
  
  for(int t=0; t<50; t++) {
    long temp_sum = 0;
    long temp_weight = 0;
    int temp_count = 0;
    
    for (int i = 0; i < num_sensors; i++) {
      int val = analogRead(sensor_pins[i]);
      if (val < 450) {
        temp_sum += (i * 1000);
        temp_weight += 1000;
        temp_count++;
      }
    }
    
    if(temp_count > 0) {
      int pos = temp_sum / temp_weight;
      sum += pos;
      count++;
    }
    delay(5);
  }
  
  if(count > 0) {
    int avg_pos = sum / count;
    center_offset = avg_pos - 2000;
  }
  
  calibrated = true;
}

void loop() {
  if(calibrated) {
    line_follow();
  }
}

int read_sensors() {
  long sum = 0;
  long weight = 0;
  int count = 0;
  
  for (int i = 0; i < num_sensors; i++) {
    int val = analogRead(sensor_pins[i]);
    sValues[i] = val;
    
    if (val < 450) {
      sum += (i * 1000);
      weight += 1000;
      count++;
    }
  }
  
  if (count == 0) return -999;
  if (count == 5) return 9999;
  
  if (sValues[0] < 450) last_side = -1;
  if (sValues[4] < 450) last_side = 1;
  
  int linePos = sum / weight;
  return (linePos - 2000 - center_offset);
}

void line_follow() {
  int error = read_sensors();
  
  if (error == -999) {
    if (last_side == -1) {
      motor_drive(-turn_speed, turn_speed);
    } else {
      motor_drive(turn_speed, -turn_speed);
    }
    return;
  }
  
  if (error == 9999) {
    motor_drive(base_speed, base_speed);
    return;
  }
  
  int adjusted_error = error;
  if(abs(error) < 200) {
    adjusted_error = 0;
  } else if(abs(error) < 400) {
    adjusted_error = error * 0.5;
  }
  
  float P = adjusted_error;
  float D = (adjusted_error - prev_error) * 0.6;
  int adjustment = (int)((kp * P) + (kd * D));
  prev_error = adjusted_error;
  
  adjustment = constrain(adjustment, -50, 50);
  
  int left = base_speed + adjustment;
  int right = base_speed - adjustment;
  
  left = constrain(left, 80, max_speed);
  right = constrain(right, 80, max_speed);
  
  motor_drive(left, right);
}

void motor_drive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -max_speed, max_speed);
  rightSpeed = constrain(rightSpeed, -max_speed, max_speed);
  
  if (leftSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(leftSpeed));
  
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  analogWrite(ENB, abs(rightSpeed));
}