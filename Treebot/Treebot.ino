#include <SPI.h>
#include <math.h>
#include <Servo.h>
#include <String.h>
#include <SD.h>


File genome;
File fitness;
Servo M0;
Servo Ml;
Servo Mr;

// Assumes the dimensions of genome arrays
// 12 entries per row, 14 rows, flattened to 1d array
double weights[168];
int expressions[168];
double joints[3];
double activations[12];

// Evaluation steps
int eval_steps = 0;

// At each specific evaluation time, are the left/right cylinders being seen only when they are supposed to?
// 0 -> not seen
// 1 -> seen, cylinder is far
// -1 -> seen, cylinder is close
// Assumes 200 evaluation steps
int left[200];
int right[200];

// sensor and motor pins
#define L_IR_pin A1
#define R_IR_pin A5
#define M0_pin 2
#define ML_pin 4
#define MR_pin 6

// functions in this project
int* neural_network(double left_IR, double right_IR);
double* IR_reader(int m0, int ml, int mr);
void motor_driver(int m0, int ml, int mr);
double compute_fitness();
void save_fitness(double fit);
void see_cylinder_p(double m0, double ml, double mr);
int base_angle_reader(int m0);
//boolean intersect_circle(double m0, double m, String circle);
//void see_cylinder_p(double m0, double ml, double mr);
//double* get_range(double m0, String cylinder);
//double tanh(double x);

// IR parameters
// used for clamping
double IR_min = 0; //when Treebot cannot see
double Raw_max_range = 450; //the close object

// used for telling cylinder from background noise
double left_threshold = 65;
double right_threshold = 65;

//// Global parameters for forward kinematics
//// use cm as unit
//double main_arm_length = 22.5;
//double cylinder_radius = 8.45;
//double left_close_cylinder_xy[2] = {-37.5, 48.6};
//double left_far_cylinder_xy[2] = {-65.0, 67.3};
//double right_close_cylinder_xy[2] = {37.5, 48.6};
//double right_far_cylinder_xy[2] = {65.0, 67.3};

//Adjust these as environment changes
String left_cylinder = "close"; //"far/close"
String right_cylinder = "far"; //"far/close"

// how much time (in ms) is required for the base servo to make a full swing? (check the servo specifications for this)
double full_swing_time = 1500;
// how much time (in ms) to wait before starting the next eval step?
int post_step_delay = 1500;

void setup() {
  
  Serial.begin(9600);
  // ------------------------------------------------------------------
  // NOTE: COMMENT THIS OUT IN THE ACTUAL RUN IF NOT CONNECTED TO SERIAL!!!
  // ------------------------------------------------------------------
  while (!Serial) {
  }
  // ------------------------------------------------------------------

  // Connect CS to 53, SCK to 52, MOSI to 51, MISO to 50, GND to ground, VCC to 5V
  if (!SD.begin(53)) {
    Serial.println("ERROR: initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  if (SD.exists("genome.txt")) {
    Serial.println("Loading genome file...");
    genome = SD.open("genome.txt");
    
    if (genome) {
      int genome_size = genome.available();
      char* genome_buffer = new char[genome_size];
      genome.read(genome_buffer, genome_size);
      genome.close();
      Serial.println("Loading complete.");

      // ------------------------------------------------------------------
      // NOTE: THIS SECTION ASSUMES DIMENSIONS OF GENOME ARRAYS
      // ------------------------------------------------------------------
      Serial.println("Filling genome arrays...");
      weights[0] = atof(strtok(genome_buffer, " \n"));
      for (int i = 1; i < 168; i++) {
        weights[i] = atof(strtok(NULL, " \n"));
      }
      for (int i = 0; i < 168; i++) {
        expressions[i] = atoi(strtok(NULL, " \n"));
      }
      for (int i = 0; i < 3; i++) {
        joints[i] = atof(strtok(NULL, " \n"));
      }
      free(genome_buffer);
      Serial.println("Filling complete.");
      // ------------------------------------------------------------------
    }
    else {
      Serial.println("ERROR: cannot read genome file!");
    }
  
  } 
  else {
    Serial.println("ERROR: cannot find genome file!");
    return;
  }

  // ------------------------------------------------------------------
  // Print the three genome arrays
  // ------------------------------------------------------------------
  Serial.print("Weights: ");
  for (int i = 0; i < 168; i++) {
    Serial.print(weights[i], 10);
    Serial.print(" ");
  }
  Serial.print("\nExpressions: ");
  for (int i = 0; i < 168; i++) {
    Serial.print(expressions[i]);
    Serial.print(" ");
  }
  Serial.print("\nJoints: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(joints[i], 10);
    Serial.print(" ");
  }
  Serial.println();
  // ------------------------------------------------------------------

  // Zeros out the activation array
  // Assumes dimension
  for (int i = 0; i < 12; i++) {
    activations[i] = 0;
  }

  // Zeros out the left and right arrays
  // Assumes 200 eval steps
  for (int i = 0; i < 200; i++) {
    left[i] = 0;
    right[i] = 0;
  }

  // ------------------------------------------------------------------
  // Print the activation, left, and right arrays
  // Assumes dimension and 200 eval steps
  // ------------------------------------------------------------------
  Serial.print("Activations: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(activations[i], 10);
    Serial.print(" ");
  }
  Serial.print("\nLeft: ");
  for (int i = 0; i < 200; i++) {
    Serial.print(left[i]);
    Serial.print(" ");
  }
  Serial.print("\nRight: ");
  for (int i = 0; i < 200; i++) {
    Serial.print(right[i]);
    Serial.print(" ");
  }
  Serial.println();
  // ------------------------------------------------------------------

  // set sensor and motor pinModes
  pinMode(L_IR_pin, INPUT);
  pinMode(R_IR_pin, INPUT);
  pinMode(M0_pin, OUTPUT);
  pinMode(ML_pin, OUTPUT);
  pinMode(MR_pin, OUTPUT);

  // attach the servos;
  M0.attach(M0_pin);
  Ml.attach(ML_pin);
  Mr.attach(MR_pin);

  // set initial positions
  M0.write(90);
  Ml.write(45);
  Mr.write(135);
  delay(1000);
}


void loop() {
  // Assumes 200 evaluation steps
  while(eval_steps < 200) {
    Serial.print("Step: ");
    Serial.print(eval_steps);
    Serial.print("\n");
    int M0_angle = M0.read();
    M0_angle = base_angle_reader(M0_angle);
    int Ml_angle = Ml.read();
    int Mr_angle = Mr.read();
    double* IR_values = IR_reader(M0_angle, Ml_angle, Mr_angle);
    Serial.print("M0:");
    Serial.print(M0_angle);
    Serial.print(" ML:");
    Serial.print(Ml_angle);
    Serial.print(" MR:");
    Serial.println(Mr_angle);
    Serial.print("Current_fitness: ");
    Serial.println(compute_fitness());
    Serial.print("Left_seen?: ");
    Serial.println(left[eval_steps]);
    Serial.print("Right_seen?: ");
    Serial.println(right[eval_steps]);
    int* motor_commands = neural_network(IR_values[0], IR_values[1]);
    motor_driver(motor_commands[0], motor_commands[1], motor_commands[2]);
    free(IR_values);
    free(motor_commands);
    eval_steps++;
    // Makes sure the motor has moved to the intended position before the next command comes in
    delay(post_step_delay);
  }
  Serial.println("Run complete! Computing fitness...");
  double fit = compute_fitness();
  Serial.print("Fitness: ");
  Serial.print(fit, 10);
  Serial.println();
  Serial.print("Left array:");
  for (int i = 0; i < 200; i++) {
    Serial.print(left[i]);
    Serial.print(" ");
  }
  Serial.print("\nRight array:");
  for (int i = 0; i < 200; i++) {
    Serial.print(right[i]);
    Serial.print(" ");
  }
  save_fitness(fit);
  while(1);
}


// ------------------------------------------------------------------
// Input: IR values clamped between 0 and 10
// Output: A int array of length 3, containing the angles to be fed
//         into the 3 motors, first m0, second mL, third mR
// Side effects: Update the activation array
// ------------------------------------------------------------------
// NOTE: DON'T FORGET TO FREE AFTER CALLING!!!
// ------------------------------------------------------------------
int* neural_network(double left_IR, double right_IR) {
  // Assumes dimensions
  // this will replace the old activations by the end of this function
  double new_activations[12];
  // temporary space for storing column multiplication results
  double pre_sum_array[14];
  // sum of the pre_sum_array, update for each column
  double sum;
  // the motor angles array to be returned as result
  int* result = (int*) malloc(3 * sizeof(int));

  // Assumes dimensions
  // populate new_activations
  for (int col = 0; col < 12; col++) {
    // populate pre_sum_array
    for (int row = 0; row < 12; row++) {
      pre_sum_array[row] = weights[col + row * 12] * expressions[col + row * 12] * activations[row];
    }
    // fill the remaining 2 values by multiplying with IR values
    pre_sum_array[12] = weights[col + 12 * 12] * expressions[col + 12 * 12] * left_IR;
    pre_sum_array[13] = weights[col + 13 * 12] * expressions[col + 13 * 12] * right_IR;

    // sum up the pre_sum_array
    for (int i = 0; i < 14; i++) {
      sum += pre_sum_array[i];
    }

    // set the corresponding new_activation entry to be tanh(sum)
    // then reset sum to 0
    new_activations[col] = tanh(sum);
    sum = 0;
  }

  // Assumes dimensions
  // Overwrite the original activations with new_activations
  for (int i = 0; i < 12; i++) {
    activations[i] = new_activations[i];
  }

  // Assumes dimensions
  // Assumes the max range of servo motors to be 180
  // Calculate results
  for (int i = 0; i < 3; i++) {
    result[i] = (int) round(activations[9 + i] * joints[i] * 90 + 90);
  }

  // The angles for left servo must stay within the range [0, 90]
  // The angles for right servo must stay within the range [90, 180]
  result[1] = result[1] / 2;
  result[2] = result[2] / 2 + 90;

  return result;
}


// ------------------------------------------------------------------
// Read in both IR's, clamp their values down to [0, 10], and return
// the clamped values in a length 2 array first left, second right
// also update left and right arrays by calling see_cylinder_p with
// the passed in motor parameters and averaged IR locally computed
// ------------------------------------------------------------------
// NOTE: DON'T FORGET TO FREE!!!
// ------------------------------------------------------------------
double* IR_reader(int m0, int ml, int mr) {
  // result to be returned
  double* result = (double*) malloc(2 * sizeof(double));
  double LIR = 0;
  double RIR = 0;
  
  // Read in the IR values; this part needs to be consistent with previous testings
  // takes 10 measures and average
  for (int i = 0; i < 10; i++) {
    LIR += analogRead(L_IR_pin);
    RIR += analogRead(R_IR_pin);
  }
  LIR = LIR / 10;
  RIR = RIR / 10;

  Serial.print("LIR:");
  Serial.print(LIR);
  Serial.print(" RIR:");
  Serial.print(RIR);
  Serial.print(" ");

  // update left and right arrays by calling see_cylinder_p
  see_cylinder_p(m0, ml, mr, LIR, RIR);
  
  // Clamp the IR readings down to the [0, 10] range and store into result pointer
  LIR = (LIR - IR_min) / Raw_max_range * 10;
  RIR = (RIR - IR_min) / Raw_max_range * 10;
  result[0] = LIR;
  result[1] = RIR;

  return result;
}


// ------------------------------------------------------------------
// Take in angles for m0, ml, and mr, and move them to corresponding
// positions accordingly
// ------------------------------------------------------------------
void motor_driver(int m0, int ml, int mr) {
  M0.write(m0);
  Ml.write(ml);
  Mr.write(mr);
}


// ------------------------------------------------------------------
// Assuming the left and right arrays have been updated, use these 2
// arrays to compute and return the overall fitness for this evaluation
// ------------------------------------------------------------------
double compute_fitness() {
  double left_sum = 0;
  double right_sum = 0;
  double left_normalized;
  double right_normalized;

  // Assumes 200 eval steps
  // Sums up the left and right arrays
  for (int i = 0; i < 200; i++) {
    left_sum += left[i];
    right_sum += right[i];
  }

  // Normalizes the two sums
  // Both sums are bounded by the [-200, 200] range
  left_normalized = (left_sum + 200) / 400;
  right_normalized = (right_sum + 200) / 400;

  // Returns the average of the two normalized sums
  return (left_normalized + right_normalized) / 2;
}


// ------------------------------------------------------------------
// Given a fitness value, save it in a file named "fitness.txt"
// ------------------------------------------------------------------
void save_fitness(double fit) {
  fitness = SD.open("fitness.txt", FILE_WRITE);
  if (fitness) {
    fitness.println(String(fit));
  }
  else {
    Serial.println("Error: cannot open fitness file!");
  }
}


// ------------------------------------------------------------------
// Wrapper function for intersect_circle
// ------------------------------------------------------------------
// Input: m0, ml, mr are the angles of base, left, and right servo motors
//        currently they are simply taken from the commands sent to servos
//        i.e. we assume servos have moved to the exact positions they
//        are instructed to; in the future we might shift to using servo.read
//        left_IR and right_IR are IR values passed in from IR_reader
// Global parameter: left_cylinder and right_cylinder, which can be either close or far
//                   left_threshold and right_threshold are the smallest IR values that
//                   can be reliably considered to not be due to noise. These values do
//                   NOT distinguish between close and far conditions, since those are already
//                   defined in the experimental setup.
// Side effect: modify the left and right arrays for the current evaluation step
//              with the results returned from intersect_circle
// ------------------------------------------------------------------
void see_cylinder_p(int m0, int ml, int mr, double left_IR, double right_IR) {
  // Is the left cylinder being seen?
  if (((m0 + ml < 180) && (left_IR > left_threshold)) || ((m0 + mr < 180) && (right_IR > right_threshold))) {
    if (left_cylinder.equals("close")) {
      left[eval_steps] = -1;
    } else {
      left[eval_steps] = 1;
    }
  }

  
  // Is the right cylinder being seen?
  if (((m0 + ml > 180) && (left_IR > left_threshold)) || ((m0 + mr > 180) && (right_IR > right_threshold))) {
    if (right_cylinder.equals("close")) {
      right[eval_steps] = -1;
    } else {
      right[eval_steps] = 1;
    }
  }
}

// ------------------------------------------------------------------
// Input:  m0 is the command sent to the base servo
// Global parameters: full_swing_time and post_step_delay, with descriptions at where they were declared
// Output: how much will the base servo actually turn
// ------------------------------------------------------------------
// Note: this function is only intended to be used on the base servo!
// ------------------------------------------------------------------
int base_angle_reader(int m0) {
  // assuming the base servo motor swings at constant velocity, this velocity is then (180 / full_swing_time) (degree/ms)
  // the max angle by which the base servo motor can swing in the period of post_step_delay is therefore (post_step_delay * (180 / full_swing_time))
  // if m0 is smaller than this angle, then the base servo will be able to make the full swing, in which case just return m0
  // otherwise return (post_step_delay * (180 / full_swing_time))
  int swing_speed = (int) (180 / full_swing_time);
  if (m0 <= (post_step_delay * swing_speed)) {
    return m0;
  }
  else {
    return (post_step_delay * swing_speed);
  }
}


// ------------------------------------------------------------------------------------------------------------------------------------
// TRASH YARD
// ------------------------------------------------------------------------------------------------------------------------------------


//// angles of cylinders
//double close_min;
//double close_max;
//double far_min;
//double far_max;

// IR ranges of cylinders
//double close_min = 170;
//double close_max = 250;
//double far_min = 60;
//double far_max = 120;


// ------------------------------------------------------------------
// ORIGINALLY FROM IR_READER
// ------------------------------------------------------------------
//  result[2] = LIR;
//  result[3] = RIR;

//  // Use IR values to identify cylinders
//  if ((LIR >= close_min) && (LIR <= close_max)) {
//    left[eval_steps] = -1;
//  } else if ((LIR >= far_min) && (LIR <= far_max)) {
//    left[eval_steps] = 1;
//  }
//  if ((RIR >= close_min) && (RIR <= close_max)) {
//    right[eval_steps] = -1;
//  } else if ((RIR >= far_min) && (RIR <= far_max)) {
//    right[eval_steps] = 1;
//  }
// ------------------------------------------------------------------


//// tanh
//double tanh(double x) {
//  double x0 = exp(x);
//  double x1 = 1.0 / x0;
//
//  return ((x0 - x1) / (x0 + x1));
//}


// ------------------------------------------------------------------
// ORIGINALLY FROM MOTOR_DRIVER
// ------------------------------------------------------------------
//  // Is the close cylinder being pointed at?
//  // Is the close cylinder being pointed at by left arm?
//  if ((LIR >= close_min) && (LIR <= close_max) && (((m0 - 90) + (c_ml - 90)) < 0)) {
//    left[eval_steps] = -1;
//  } 
//  // Is the close cylinder being pointed at by right arm?
//  // If the left arm is already pointing at the close cylinder, DONNOT modify right[eval_steps]
//  else if ((RIR >= close_min) && (RIR <= close_max) && (((m0 - 90) + (c_mr - 90)) > 0)) {
//    right[eval_steps] = -1;
//  }
//
//  // Is the far cylinder being pointed at?
//  // Is the far cylinder being pointed at by left arm?
//  if ((LIR >= far_min) && (LIR <= far_max) && (((m0 - 90) + (c_ml - 90)) < 0)) {
//    left[eval_steps] = -1;
//  } 
//  // Is the close cylinder being pointed at by right arm?
//  // If the left arm is already pointing at the close cylinder, DONNOT modify right[eval_steps]
//  else if ((RIR >= close_min) && (RIR <= far_max) && (((m0 - 90) + (c_mr - 90)) > 0)) {
//    right[eval_steps] = -1;
//  }

//  // Assumes servo motors to be 0 ~ 180
//  // update left and right arrays
//  // is left arm seeing close cylinder?
//  if (((m0 - 90) + (c_ml - 90)) >= close_min && ((m0 - 90) + (c_ml - 90)) <= close_max) {
//    left[eval_steps] = -1;
//  }
//  // is left arm seeing far cylinder?
//  else if (((m0 - 90) + (c_ml - 90)) >= far_min && ((m0 - 90) + (c_ml - 90)) <= far_max) {
//    left[eval_steps] = 1;
//  }
//  // is right arm seeing close cylinder?
//  if (((m0 - 90) + (c_mr - 90)) >= close_min && ((m0 - 90) + (c_mr - 90)) <= close_max) {
//    right[eval_steps] = -1;
//  }
//  // is right arm seeing far cylinder?
//  else if (((m0 - 90) + (c_mr - 90)) >= far_min && ((m0 - 90) + (c_mr - 90)) <= far_max) {
//    right[eval_steps] = 1;
//  }
// ------------------------------------------------------------------


//// ------------------------------------------------------------------
//// Update the left and right arrays
//// ------------------------------------------------------------------
//void see_cylinder_p(double m0, double ml, double mr) {
//  // Convert the servo angles to radians with 0 rad corresponding to servo.write(90)
//  double a0 = (m0 - 90) * PI / 180;
//  double al = (ml - 90) * PI / 180;
//  double ar = (mr - 90) * PI / 180;
//
//  // filled by calling get_range
//  double* range_left_close = get_range(m0, "left_close");
//  double* range_left_far = get_range(m0, "left_far");
//  double* range_right_close = get_range(m0, "right_close");
//  double* range_right_far = get_range(m0, "right_far");
//
//  // Update the left and right arrays
//  // Is left_close being pointed at?
//  if (((al >= range_left_close[0]) && (al <= range_left_close[1])) || ((ar >= range_left_close[0]) && (ar <= range_left_close[1]))) {
//    left[eval_steps] = -1;
//  }
//  // Is left_far being pointed at?
//  if (((al >= range_left_far[0]) && (al <= range_left_far[1])) || ((ar >= range_left_far[0]) && (ar <= range_left_far[1]))) {
//    left[eval_steps] = 1;
//  }
//  // Is right_close being pointed at?
//  if (((al >= range_right_close[0]) && (al <= range_right_close[1])) || ((ar >= range_right_close[0]) && (ar <= range_right_close[1]))) {
//    right[eval_steps] = -1;
//  }
//  // Is right_far being pointed at?
//  if (((al >= range_right_far[0]) && (al <= range_right_far[1])) || ((ar >= range_right_far[0]) && (ar <= range_right_far[1]))) {
//    right[eval_steps] = 1;
//  }
//
//  free(range_left_close);
//  free(range_left_far);
//  free(range_right_close);
//  free(range_right_far);
//}


//// ------------------------------------------------------------------
//// For a branch, what angle does it have to orientate in order to see a given cylinder
//// calculate using main_arm_lenth, m0, cylinder_radius, and cylinder_xy
//// ------------------------------------------------------------------
//// NOTE: DONT FORGET TO FREE!!!
//// ------------------------------------------------------------------
//double* get_range(double m0, String cylinder) {
//  // result to be returned
//  double* result = (double*) malloc(2 * sizeof(double));
//  
//  // Convert the servo angles to radians with 0 rad corresponding to servo.write(90)
//  double a0 = (m0 - 90) * PI / 180;
//
//  // what are the coordinates of the tip of main arm?
//  double main_tip_x = main_arm_length * sin(a0);
//  double main_tip_y = abs(main_arm_length * cos(a0));
//
//  // what are the coordinates of the cylinder
//  double x;
//  double y;
//  if (cylinder.equals("left_close")) {
//    x = left_close_cylinder_xy[0];
//    y = left_close_cylinder_xy[1];
//  }
//  else if (cylinder.equals("left_far")) {
//    x = left_far_cylinder_xy[0];
//    y = left_far_cylinder_xy[1];
//  }
//  else if (cylinder.equals("right_close")) {
//    x = right_close_cylinder_xy[0];
//    y = right_close_cylinder_xy[1];
//  }
//  else if (cylinder.equals("right_far")) {
//    x = right_far_cylinder_xy[0];
//    y = right_far_cylinder_xy[1];
//  }
//
//  double a = sqrt(sq(x - main_tip_x) + sq(y - main_tip_y));
//  double d = sinh(cylinder_radius / a);
//  double e = abs(sinh((y - main_tip_y) / a));
//  double f = PI / 2 - e - d - a0;
//  double g = f + 2 * d - a0;
//
//  // If (main_tip_y / main_tip_x) < (y / x)
//  // branch arm needs to turn left, i.e. negative f and g
//  if ((main_tip_y / main_tip_x) < (y / x)) {
//    f = -f;
//    g = -g;
//  }
//
//  // fill the result array and return
//  result[0] = min(f, g);
//  result[1] = max(f, g);
//  return result;


//// ------------------------------------------------------------------
//// This function returns true if the extension of the branch driven by m
//// intersects with the base of a given cylinder.
//// The logic of this function took inspiration from this post: and Collin's implementation
//// https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
//// ------------------------------------------------------------------
//// Input: m0 is the angle of the base motor
////        m is the angle of either left or right motor
////        circle can be one of "left_close", "left_far", "right_close", "right_far"
//// Global parameter: left_close_cylinder_xy, left_far_cylinder_xy, right_close_cylinder_xy,
////                   and right_far_cylinder_xy, which are all pre-defined coordinates
////                   relative to the base motor.
////                   main_arm_length, which is the measured length of main arm
////                   cylinder_radius, which is the measured radius of cylinders
////                   These are all in the unit of cm
//// Output: Boolean indicating if there's an intersection
//// ------------------------------------------------------------------
//boolean intersect_circle(double m0, double m, String circle) {
//  // Convert the servo angles to radians with 0 rad corresponding to servo.write(90)
//  double a0 = (m0 - 90) * PI / 180;
//
//  // what are the coordinates of main arm's tip?
//  double main_tip_x = main_arm_length * sin(a0);
//  double main_tip_y = main_arm_length * cos(a0);
//
//  // what are the circle's coordinates relative to main arm's tip?
//  double relative_x;
//  double relative_y;
//  if (circle.equals("left_close")) {
//    relative_x = left_close_cylinder_xy[0] - main_tip_x;
//    relative_y = left_close_cylinder_xy[1] - main_tip_y;
//  }
//  else if (circle.equals("left_far")) {
//    relative_x = left_far_cylinder_xy[0] - main_tip_x;
//    relative_y = left_far_cylinder_xy[1] - main_tip_y;
//  }
//  else if (circle.equals("right_close")) {
//    relative_x = right_close_cylinder_xy[0] - main_tip_x;
//    relative_y = right_close_cylinder_xy[1] - main_tip_y;
//  }
//  else if (circle.equals("right_far")) {
//    relative_x = right_far_cylinder_xy[0] - main_tip_x;
//    relative_y = right_far_cylinder_xy[1] - main_tip_y;
//  }
//
//  // if we imagine the branch driven by m to be a straight line passing
//  // the origin of a coordinate grid centered at (main_tip_x, main_tip_y),
//  // and orientated with positive y pointing towards 0 rad and positive x
//  // pointing towards pi/2 rad, what would its slope be?
//  double slope = -tan((m + m0 - 90) * PI / 180);
//  
//  // what is the shortest distance between the aforementioned line and the
//  // center of circle?
//  double distance = abs((-slope * relative_x + relative_y) / slope);
//
//  // if distance is shorter than radius, then there is an intersectoin
//  return (distance <= cylinder_radius);
//}
//}

