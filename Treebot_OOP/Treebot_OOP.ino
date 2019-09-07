#include <SPI.h>
#include <math.h>
#include <Servo.h>
#include <String.h>
#include <SD.h>
#include "NN.h"

#define left_ir_pin A1
#define right_ir_pin A5
#define m0_pin 2
#define ml_pin 4
#define mr_pin 6

Servo m0;
Servo ml;
Servo mr;

int pop_size = 10;
int eval_steps = 200;
int post_step_delay = 200;
bool debug = false;

double max_ir_range = 450;
// If ir_reading > ir_threshold, the ir sensor is seeing something
double ir_threshold = 2.33;

int full_swing_time = 1600;
// what is the maximum angle range by which the base servo can swing during post_time_delay?
int max_swing_range;

double progress = 0;
double task_size;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  // Connect CS to 53, SCK to 52, MOSI to 51, MISO to 50, GND to ground, VCC to 5V
  if (!SD.begin(53)) {
    Serial.println("ERROR: initialization failed!");
    while(1);
  }
  
  max_swing_range = post_step_delay * (180.0 / full_swing_time);
  task_size = pop_size * eval_steps * 2;

  pinMode(left_ir_pin, INPUT);
  pinMode(right_ir_pin, INPUT);
  pinMode(m0_pin, OUTPUT);
  pinMode(ml_pin, OUTPUT);
  pinMode(mr_pin, OUTPUT);

  m0.attach(m0_pin);
  ml.attach(ml_pin);
  mr.attach(mr_pin);
}

void loop() {
  String header = "genome";
  String tail = ".EXT";  
  double fitnesses[pop_size];

  String first_env[2] = {"close", "far"};
  for (int i = 0; i < pop_size; i++) {
    String full_file_name = header + i + tail;
    NN nn = NN(full_file_name);
    fitnesses[i] = Evaluate(nn, first_env);
    Serial.println();
  }

  bool pause = true;
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("Please adjust the environment settings to far-close, send C to serial monitor when you are done");
  Serial.print("Fitnesses(close-far): ");
  for (int i = 0; i < pop_size; i++) {
    Serial.print(fitnesses[i]);
    Serial.print(" ");
  }
  while (pause) {
    while (Serial.available() > 0) {
      if (Serial.read() == 'C') {
        pause = false;
        Serial.println();
      }
    }
  }

  String second_env[2] = {"far", "close"};
  for (int i = 0; i < pop_size; i++) {
    String full_file_name = header + i + tail;
    NN nn = NN(full_file_name);
    fitnesses[i] = Evaluate(nn, second_env);
    Serial.println();
  }

  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("////////////////////////////////////////////////////////");
  Serial.println("Evaluation completed, please restore the environment setting to close-far");
  Serial.print("Fitnesses(far-close): ");
  for (int i = 0; i < pop_size; i++) {
    Serial.print(fitnesses[i]);
    Serial.print(" ");
  }
  while(1);
}

double Evaluate(NN nn, String env[2]) {
  // Are left and right cylinders seen at each evaluation step?
  // 0 -> not seen
  // 1 -> seen, cylinder is far
  // -1 -> seen, cylinder is close
  int left_see_p[eval_steps];
  int right_see_p[eval_steps];
  for (int i = 0; i < eval_steps; i++) {
    left_see_p[i] = 0;
    right_see_p[i] = 0;
  }
  int prev_m0_angle = 90;

  m0.write(90);
  ml.write(45);
  mr.write(135);
  delay(2000);
  
  double ir_values[2];
  int see_p[2];
  int motor_commands[3];
  for (int i = 0; i < eval_steps; i++) {
    int m0_angle = Estimate_m0_Angle(m0.read(), prev_m0_angle);
    prev_m0_angle = m0_angle;
    int ml_angle = ml.read();
    int mr_angle = mr.read();
    IR_Reader(ir_values);
    See_p(env, m0_angle, ml_angle, mr_angle, ir_values[0], ir_values[1], see_p);
    left_see_p[i] = see_p[0];
    right_see_p[i] = see_p[1];
    nn.StepNN(ir_values[0], ir_values[1], motor_commands);
    m0.write(motor_commands[0]);
    ml.write(motor_commands[1]);
    mr.write(motor_commands[2]);
    // Make sure the motor has moved to the intended position before the next command comes in
    delay(post_step_delay);

    Serial.print(progress / task_size);
    Serial.print(" ");
    progress++;
    
    if (debug) {
      Serial.print("-------------------------------------------------------");
      Serial.print("Step: ");
      Serial.print(i);
      Serial.print("\n");
      
      Serial.print("M0:");
      Serial.print(m0_angle);
      Serial.print(" ML:");
      Serial.print(ml_angle);
      Serial.print(" MR:");
      Serial.print(mr_angle);
      Serial.print("\n");
      
      Serial.print("LIR:");
      Serial.print(ir_values[0]);
      Serial.print(" RIR:");
      Serial.print(ir_values[1]);
      Serial.print("\n");
      
      Serial.print("Left_seen?:");
      Serial.print(left_see_p[i]);
      Serial.print(" Right_seen?:");
      Serial.print(right_see_p[i]);
      Serial.print("\n");
    }
  }

  return Compute_Fitness(left_see_p, right_see_p);
}

int Estimate_m0_Angle(int m0, int prev_m0_angle) {
  // if |m0 - prev_m0_angle| is smaller than max_swing_range, then the base servo will be able to make the full swing, in which case just return m0
  // otherwise return max_swing_range with its sign consistent with the direction of turn.
  if (abs(m0 - prev_m0_angle) <= max_swing_range) {
    return m0;
  }
  else {
    // since the calculation of result will involve prev_m0_angle, must calculate the result before resetting prev_m0_angle
    int result = ((m0 - prev_m0_angle) > 0) ? (prev_m0_angle + max_swing_range) : (prev_m0_angle - max_swing_range);
    return result;
  }
}

void IR_Reader(double result[2]) {
  double left_ir_sum = 0;
  double right_ir_sum = 0;
  
  for (int i = 0; i < 10; i++) {
    left_ir_sum += analogRead(left_ir_pin);
    right_ir_sum += analogRead(right_ir_pin);
  }
  left_ir_sum = left_ir_sum / 10;
  right_ir_sum = right_ir_sum / 10;
  
  // Clamp the IR readings down to the [0, 10] range and store into result array
  left_ir_sum = left_ir_sum / max_ir_range * 10;
  right_ir_sum = right_ir_sum / max_ir_range * 10;
  result[0] = left_ir_sum;
  result[1] = right_ir_sum;
}

void See_p(String env[2], int m0_angle, int ml_angle, int mr_angle, double left_ir, double right_ir, int result[2]) {
  // Is the left cylinder being seen?
  if (((m0_angle + ml_angle < 180) && (left_ir > ir_threshold)) || ((m0_angle + mr_angle < 180) && (right_ir > ir_threshold))) {
    if (env[0].equals("close")) {
      result[0] = -1;
    } else {
      result[0] = 1;
    }
  } else {
    result[0] = 0;
  }
  
  // Is the right cylinder being seen?
  if (((m0_angle + ml_angle > 180) && (left_ir > ir_threshold)) || ((m0_angle + mr_angle > 180) && (right_ir > ir_threshold))) {
    if (env[1].equals("close")) {
      result[1] = -1;
    } else {
      result[1] = 1;
    }
  } else {
    result[1] = 0;
  }
}

double Compute_Fitness(int left_see_p[], int right_see_p[]) {
  double left_sum = 0;
  double right_sum = 0;
  double left_normalized;
  double right_normalized;

  for (int i = 0; i < eval_steps; i++) {
    left_sum += left_see_p[i];
    right_sum += right_see_p[i];
  }

  left_normalized = (left_sum + eval_steps) / (eval_steps*2);
  right_normalized = (right_sum + eval_steps) / (eval_steps*2);

  return (left_normalized + right_normalized) / 2;
}
