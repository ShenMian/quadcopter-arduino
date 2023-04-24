#include "evaluator.hpp"

Evaluator  evaluator;

void setup()
{
}

void loop()
{
  evaluator.update(dt);

  const auto angles = evaluator.get_angles();

  Serial.print("$");
  Serial.print(angles.yaw);
  Serial.print(" ");
  Serial.print(angles.pitch);
  Serial.print(" ");
  Serial.print(angles.roll);
  Serial.print(";");
}
