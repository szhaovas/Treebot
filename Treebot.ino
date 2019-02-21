#include <SPI.h>
#include <SD.h>
#include <math.h>


File genome;
// Assumes the dimensions of genome arrays
// 12 entries per row, 14 rows, flattened to 1d array
float weights[168];
int expressions[168];
float joints[3];
float activations[12];

// Evaluation steps
int eval_steps = 0;

// At each specific evaluation time, is the left/right arm behaving correctly?
// Assumes 200 evaluation steps
int left[200];
int right[200];


void setup() {
  
  Serial.begin(9600);
  while (!Serial) {
  }

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
}

void loop() {
  // Assumes 200 evaluation steps
  while(eval_steps < 200) {
    
    eval_steps++;
  }
}


// ------------------------------------------------------------------
// Input: IR values clamped between 0 and 10
// Output: A float array of length 3, containing the angles to be fed
//         into the 3 motors, first m0, second mL, third mR
// Side effects: Update the activation array
// ------------------------------------------------------------------
float* neural_network(float left_IR, float right_IR) {
  // Assumes dimensions
  // this will replace the old activations by the end of this function
  float new_activations[12];
  // temporary space for storing column multiplication results
  float pre_sum_array[14];
  // sum of the pre_sum_array, update for each column
  float sum;
  // the motor angles array to be returned as result
  float* result = (float*) malloc(3 * sizeof(float));

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


// tanh
float tanh(float x) {
  float x0 = exp(x);
  float x1 = 1.0 / x0;

  return ((x0 - x1) / (x0 + x1));
}
