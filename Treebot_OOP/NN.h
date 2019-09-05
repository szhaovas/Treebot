#include <Arduino.h>

class NN {

private:
  double weights[168];
  int expressions[168];
  double joints[3];
  double prev_activations[12];

  double Sum(double arr[], int arr_size);

public:
  NN(String source_name);
  void StepNN(double left_ir, double right_ir, int result[3]);
  
};
