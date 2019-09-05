#include <Arduino.h>
#include <SD.h>
#include "NN.h"

NN::NN(String source_name) {
  // Initialize NN with information read from the given file
  File source_file;
  if (SD.exists(source_name)) {
    source_file = SD.open(source_name);
    if (source_file) {
      int source_size = source_file.available();
      char* source_buffer = new char[source_size];
      source_file.read(source_buffer, source_size);
      source_file.close();

      weights[0] = atof(strtok(source_buffer, " \n"));
      for (int i = 1; i < 168; i++) {
        this->weights[i] = atof(strtok(NULL, " \n"));
      }
      for (int i = 0; i < 168; i++) {
        this->expressions[i] = atoi(strtok(NULL, " \n"));
      }
      for (int i = 0; i < 3; i++) {
        this->joints[i] = atof(strtok(NULL, " \n"));
      }
      free(source_buffer);
    }
    else {
      Serial.println("ERROR: cannot read source file!");
      while(1);
    }
  }
  else {
    Serial.println("ERROR: cannot find source file!");
    while(1);
  }

  // NN starts from an undisturbed state
  memset(this->prev_activations, 0, 12*sizeof(double));
}

void NN::StepNN(double left_ir, double right_ir, int result[3]) {
  double new_activations[12];

  // For each neuron, calculate the weighted sum of all incoming connections
  double incoming_connections[14];
  for (int col = 0; col < 12; col++) {
    for (int row = 0; row < 12; row++) {
      incoming_connections[row] = this->weights[col + row * 12] * this->expressions[col + row * 12] * this->prev_activations[row];
    }
    incoming_connections[12] = this->weights[col + 12 * 12] * this->expressions[col + 12 * 12] * left_ir;
    incoming_connections[13] = this->weights[col + 13 * 12] * this->expressions[col + 13 * 12] * right_ir;

    double weighted_sum = NN::Sum(incoming_connections, 14);
    new_activations[col] = tanh(weighted_sum);
  }

  for (int i = 0; i < 12; i++) {
    this->prev_activations[i] = new_activations[i];
  }

  // Assume the max range of servo motors to be 180
  for (int i = 0; i < 3; i++) {
    result[i] = (int) round(new_activations[9 + i] * this->joints[i] * 90 + 90);
  }

  // The angles for left servo must stay within the range [0, 90]
  result[1] = result[1] / 2;
  // The angles for right servo must stay within the range [90, 180]
  result[2] = result[2] / 2 + 90;
}

double NN::Sum(double arr[], int arr_size) {
  double result;

  for (int i = 0; i < arr_size; i++) {
      result += arr[i];
  }

  return result;
}
