#include "Adafruit_MLX90393.h"
#include "Arduino.h"
#include "FixedPointsCommon.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "jemotor/feedback_motor.h"
#include "jemotor/mlx90393_sensor.h"
#include "jemotor/three_wire_motor.h"

constexpr int pin_pwma = 0;
constexpr int pin_ain1 = 1;
constexpr int pin_ain2 = 8;
constexpr int pin_int_drdy = 15;

Adafruit_MLX90393 magnetometer;
void init_magnetometer();

jemotor::MLX90393Sensor encoder_sensor(&magnetometer);
jemotor::ThreeWireMotor motor(pin_pwma, pin_ain1, pin_ain2);
jemotor::FeedbackMotor fmotor(motor, encoder_sensor);
SemaphoreHandle_t fmotor_lock;

void update_motor_task(jemotor::MLX90393Sensor& encoder,
                       jemotor::FeedbackMotor& motor) {
  while (true) {
    float angle;

    // Wait for the current task notification to be set.
    uint32_t notify_value;
    xTaskNotifyWait(1, 1, &notify_value, portMAX_DELAY);

    xSemaphoreTake(fmotor_lock, portMAX_DELAY);
    const uint64_t t = micros();
    const bool did_update = encoder.Update(t);
    angle = float{encoder.angle()};
    jemotor::FeedbackMotor::UpdateDebug debug;
    if (did_update) {
      debug = motor.UpdateControl(t);
    }
    xSemaphoreGive(fmotor_lock);
    const uint64_t t_end = micros();
    const uint64_t loop_time = t_end - t;
    if (did_update) {
      Serial.printf(
          ">lt:%lld\n>dt:%f\n>setpoint:%f\n>"
          "measurement:%f\n>p:%f\n>i:%f"
          "\n>d:%f\n>output:%f\n>angle:%f\n>err:%f\n",
          loop_time, debug.dt, debug.setpoint, debug.measurement, debug.p,
          debug.i, debug.d, debug.output, angle,
          debug.setpoint - debug.measurement);
    }
  }
}

extern "C" void app_main() {
  initArduino();
  Serial.begin(115200);

  /* Wait for serial on USB platforms. */
  while (!Serial) {
    delay(10);
  }

  pinMode(pin_int_drdy, INPUT_PULLDOWN);

  fmotor_lock = xSemaphoreCreateMutex();

  analogWriteFrequency(pin_pwma, 128);
  analogWriteResolution(pin_pwma, 8);
  motor.Begin();

  init_magnetometer();

  static TaskHandle_t motor_task;
  xTaskCreate(
      +[](void*) { update_motor_task(encoder_sensor, fmotor); }, "motor", 4096,
      NULL, 6, &motor_task);

  xTaskCreate(
      +[](void*) {
        delay(1000);
        std::array<int, 5> speeds = {0, 60, 180, 360, 480};

        bool negate = false;

        xSemaphoreTake(fmotor_lock, portMAX_DELAY);
        fmotor.SetDuty(250);
        xSemaphoreGive(fmotor_lock);
        delay(1000);
        xSemaphoreTake(fmotor_lock, portMAX_DELAY);
        fmotor.Stop(jemotor::kBrake);
        xSemaphoreGive(fmotor_lock);
        delay(200);

        while (true) {
          for (auto s : speeds) {
            xSemaphoreTake(fmotor_lock, portMAX_DELAY);
            fmotor.SetSpeed(negate ? -s : s);
            xSemaphoreGive(fmotor_lock);
            delay(2000);
          }
          negate = !negate;
          if (!negate) {
            xSemaphoreTake(fmotor_lock, portMAX_DELAY);
            fmotor.SetSpeed(0);
            xSemaphoreGive(fmotor_lock);
            delay(50000000);
          }
        }
      },
      "motor", 4096, NULL, 1, NULL);

  attachInterrupt(
      15,
      []() IRAM_ATTR {
        BaseType_t higher_priority_task_woken = false;
        xTaskNotifyFromISR(motor_task, 1, eSetBits,
                           &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
      },
      RISING);
}

void init_magnetometer() {
  Wire.setPins(4, 5);
  constexpr uint8_t i2c_addr = 0x18;
  while (!magnetometer.begin_I2C(i2c_addr)) {
    // if (! magnetometer.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    Serial.println("No magnetometer found ... check your wiring?");
    delay(5000);
  }
  Wire.setClock(400'000);  // Note: can only happen after Wire.begin!
  Serial.println("Found a MLX90393 magnetometer");

  magnetometer.setGain(MLX90393_GAIN_1X);
  magnetometer.setResolution(MLX90393_X, MLX90393_RES_17);
  magnetometer.setResolution(MLX90393_Y, MLX90393_RES_17);
  magnetometer.setOversampling(MLX90393_OSR_2);
  magnetometer.setFilter(MLX90393_FILTER_3);
  magnetometer.setTrigInt(true);
  if (!magnetometer.exitMode()) {
    Serial.print("Failed to exit mode.");
  }
  if (!magnetometer.setBurstRate(0)) {
    Serial.println("Failed to set burst rate");
  };
  if (!magnetometer.startBurstMode(MLX90393_X | MLX90393_Y)) {
    Serial.println("Failed to start burst mode");
  }
}