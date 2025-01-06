idf_component_register(
  SRCS "src/feedback_motor.cc" "src/intpid.cc" "src/mlx90393_sensor.cc" "src/three_wire_motor.cc"
  INCLUDE_DIRS "include"
  PRIV_INCLUDE_DIRS "include/jemotor"
  REQUIRES "arduino" "FixedPointsArduino" "Adafruit_MLX90393_Library"
)