#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Servo.h>

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

// At each specific evaluation time, is the left/right arm behaving correctly?
// Assumes 200 evaluation steps
int left[200];
int right[200];

// sensor and motor pins
#define L_IR_pin A2
#define R_IR_pin A5
#define M0_pin 2
#define ML_pin 4
#define MR_pin 6

// functions in this project
double* neural_network(double left_IR, double right_IR);
double* IR_reader();
//double tanh(double x);
void motor_driver(double m0, double ml, double mr);
void save_fitness(double fit);
double compute_fitness();

// IR parameters
double IR_min = 0; //when Treebot cannot see
double Raw_max_range = 450; //the close object

//// angles of cylinders
//double close_min;
//double close_max;
//double far_min;
//double far_max;

// IR ranges of cylinders
double close_min = 170;
double close_max = 250;
double far_min = 60;
double far_max = 120;


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
  
}


void loop() {
  // Assumes 200 evaluation steps
  while(eval_steps < 200) {
    Serial.print("Step: ");
    Serial.print(eval_steps);
    Serial.print("\n");
    double* IR_values = IR_reader();
    Serial.print("LIR:");
    Serial.print(IR_values[0]);
    Serial.print(" RIR:");
    Serial.print(IR_values[1]);
    Serial.print(" ");
    double* motor_commands = neural_network(IR_values[0], IR_values[1]);
    Serial.print("M0:");
    Serial.print(motor_commands[0]);
    Serial.print(" ML:");
    Serial.print(motor_commands[1]);
    Serial.print(" MR:");
    Serial.print(motor_commands[2]);
    Serial.print("\n");
    // motor_driver(motor_commands[0], motor_commands[1], motor_commands[2]);
    M0.write(motor_commands[0]);
    Ml.write(motor_commands[1]);
    Mr.write(motor_commands[2]);
    free(IR_values);
    free(motor_commands);
    eval_steps++;
    // Makes sure the motor has moved to the intended position before the next command comes in
    delay(1000);
  }
  Serial.println("Run complete! Computing fitness...");
  double fit = compute_fitness();
  Serial.print("Fitness: ");
  Serial.print(fit, 10);
  Serial.println();
  save_fitness(fit);
  while(1);
}


// ------------------------------------------------------------------
// Input: IR values clamped between 0 and 10
// Output: A double array of length 3, containing the angles to be fed
//         into the 3 motors, first m0, second mL, third mR
// Side effects: Update the activation array
// ------------------------------------------------------------------
// NOTE: DON'T FORGET TO FREE AFTER CALLING!!!
// ------------------------------------------------------------------
double* neural_network(double left_IR, double right_IR) {
  // Assumes dimensions
  // this will replace the old activations by the end of this function
  double new_activations[12];
  // temporary space for storing column multiplication results
  double pre_sum_array[14];
  // sum of the pre_sum_array, update for each column
  double sum;
  // the motor angles array to be returned as result
  double* result = (double*) malloc(3 * sizeof(double));

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
    result[i] = activations[9 + i] * joints[i] * 90 + 90;
  }

  return result;
}


// ------------------------------------------------------------------
// Read in both IR's, clamp their values down to [0, 10], and return
// the clamped values in a length 2 array
// first left, second right
// ------------------------------------------------------------------
// NOTE: DON'T FORGET TO FREE!!!
// ------------------------------------------------------------------
double* IR_reader() {
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

  // Use IR values to identify cylinders
  if ((LIR >= close_min) && (LIR <= close_max)) {
    left[eval_steps] = -1;
  } else if ((LIR >= far_min) && (LIR <= far_max)) {
    left[eval_steps] = 1;
  }
  if ((RIR >= close_min) && (RIR <= close_max)) {
    right[eval_steps] = -1;
  } else if ((RIR >= far_min) && (RIR <= far_max)) {
    right[eval_steps] = 1;
  }
  
  // Clamp the IR readings down to the [0, 10] range and store into result pointer
  LIR = (LIR - IR_min) / Raw_max_range * 10;
  RIR = (RIR - IR_min) / Raw_max_range * 10;
  result[0] = LIR;
  result[1] = RIR;

  return result;
}


//// tanh
//double tanh(double x) {
//  double x0 = exp(x);
//  double x1 = 1.0 / x0;
//
//  return ((x0 - x1) / (x0 + x1));
//}


// ------------------------------------------------------------------
// Take in angles for m0, ml, and mr, and move them to corresponding
// positions accordingly
// Also update the left and right arrays according to angles
// ------------------------------------------------------------------
//void motor_driver(double m0, double ml, double mr) {
//  // run m0
//  M0.write(m0);
//
//  // clamp ml down to the range [0, 90] before running
//  double c_ml = ml /2;
//  Ml.write(c_ml);
//
//  // map mr to [90, 180] before running
//  double c_mr = mr / 2 + 90;
//  Mr.write(c_mr);
//
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
//}


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

