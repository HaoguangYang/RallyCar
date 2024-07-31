/**
 * The config data structure containing actuation limits, and the class that
 * contains publishers/subscribers, IMU driver, and a throttle watchdog (as a
 * timer) for safety concerns.
 */

#ifndef _RALLYCAR_DRIVER_MCU_NODE_HPP_
#define _RALLYCAR_DRIVER_MCU_NODE_HPP_

#include "transport_layer/hdlc.h"
#include "transport_layer/arduino_sensor_network_node.h"
#include "hardware_abstraction_layer/ArduinoHardware.h"
#include "rallycar_msgs/msg/imu_raw.h"
#include "std_msgs/msg/float32.h"

#include "mpu6050/MPU6050_6Axis_MotionApps612.h"
#include <Servo.h>

// hardware pin definitions
#define SPEED_LIMITER_PIN 2
#define STEER_PIN 6
#define ACCEL_BRAKE_PIN 7
#define LED_PIN 13

// parameters
namespace rallycar_config {
    const float accPedalRange = 2048.0;
    const float escMin = 1000.0;
    const float escMax = 2000.0;
    const float steerRange = 2048.0;
    const float steerServoMin = 1000.0;
    const float steerServoMax = 2000.0;

    // if speed limiter is on the "trim" knob
    // const float spdLimiterMin = 1380.0;  // without safety margin
    const float spdLimiterMin = 1500.0;     // with safety margin (only use upper half)
    const float spdLimiterMax = 1620.0;
    // if speed limiter is the actual "throttle" trigger
    //const float spdLimiterMin = 1500.0;
    //const float spdLimiterMax = 2000.0;

    const float steerServoZero = (steerServoMin + steerServoMax) * 0.5;
    const float escZero = (escMin + escMax) * 0.5;
}

// The time at beginning of the pulse (on rising edge) in us
volatile uint16_t spdLimiterPulseHighEdge  = 0;
// The width of pulse in us(microseconds)
volatile float escLimit = 0;

// reader of PWM from RC control (speed limiter)
void trimPulseTimer() {
    if (digitalRead(SPEED_LIMITER_PIN) == HIGH) {
      spdLimiterPulseHighEdge = micros() & 0xFFFF;
    } else {
      float spdLimiterPulseWidth = (float)((micros() & 0xFFFF) - spdLimiterPulseHighEdge);
      escLimit = (constrain(spdLimiterPulseWidth,
                            rallycar_config::spdLimiterMin, rallycar_config::spdLimiterMax)
        - rallycar_config::spdLimiterMin)
        / (rallycar_config::spdLimiterMax - rallycar_config::spdLimiterMin)
        * rallycar_config::accPedalRange;
    }
}

// main driver class
class RallycarDriverMCUNode : public sensor_network::Node<HdlcBase> {
using ImuRaw = rallycar_msgs__msg::ImuRaw;
using Float32 = std_msgs__msg::Float32;
protected:
    ImuRaw imu_data;

    MPU6050 imu;
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Servo eSC, steeringServo;
    sensor_network::Publisher<ImuRaw, RallycarDriverMCUNode> imu_pub;
    sensor_network::Subscription<Float32, RallycarDriverMCUNode> accel_brake_sub, steer_sub;
    sensor_network::WallTimer<RallycarDriverMCUNode::ClockT, RallycarDriverMCUNode> accel_watchdog;

    inline static float fmap (
        const float& in,
        const float& in_min,
        const float& in_max,
        const float& out_min,
        const float& out_max)
    {
        return (in - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

public:
    RallycarDriverMCUNode(ArduinoHardware* hw, HdlcBase* fr) :
      Node<HdlcBase>(hw, fr),
      imu_pub(this),
      accel_brake_sub(&RallycarDriverMCUNode::accelBrakeCb, this),
      steer_sub(&RallycarDriverMCUNode::steerCb, this),
      accel_watchdog(this->get_clock(), 2000000UL, &RallycarDriverMCUNode::acceleratorCmdWatchdog, this)
    {
        registerTimer(&accel_watchdog);
        /* IMPORTANT -- Make sure the registered IDs (second argument) match with the server side. */
        registerEndpoint(&imu_pub, 0);
        registerEndpoint(&accel_brake_sub, 1);
        registerEndpoint(&steer_sub, 2);
    }

    void acceleratorCmdWatchdog() {
        this->eSC.writeMicroseconds((int)rallycar_config::escZero);
    }

    virtual void onReset() override final {
        this->eSC.writeMicroseconds((int)rallycar_config::escZero);
    }

    void steerCb(const Float32& steerCmd) {
        float cmd = constrain(-steerCmd.data,
                  -rallycar_config::steerRange, rallycar_config::steerRange);
        cmd = fmap(cmd,
                  -rallycar_config::steerRange, rallycar_config::steerRange,
                  rallycar_config::steerServoMin, rallycar_config::steerServoMax);
        this->steeringServo.writeMicroseconds((int)cmd);
    }

    void accelBrakeCb(const Float32& accelBrakeCmd) {
        float cmd = constrain(accelBrakeCmd.data,
                  -escLimit, escLimit);
        cmd = fmap(cmd,
                  -rallycar_config::accPedalRange, rallycar_config::accPedalRange,
                  rallycar_config::escMin, rallycar_config::escMax);
        this->eSC.writeMicroseconds((int)cmd);
        this->accel_watchdog.reset();
    }

    void imu_init() {
        // power-on settling time is 30 ms
        delay(30);
        // Try to initialize IMU to highest sensitivity for calibration
        imu.initialize();
        while (!imu.testConnection()) {
            delay(50);
            imu.initialize();
        }
        int16_t gyroOffset[3], accelOffset[3];
        imu.CoarseCalibrate(&gyroOffset[0], &accelOffset[0]);
        // this resets the MPU6050, therefore the previous coarse calibration
        // results need to be stored locally, and re-upload.
        // imu.dmpInitialize() returns 0 when success
        while (imu.dmpInitialize()) {
            delay(50);
        }
        // power-on settling time is 30 ms
        delay(30);
        // re-upload calibration results
        imu.setXGyroOffset(gyroOffset[0]);
        imu.setYGyroOffset(gyroOffset[1]);
        imu.setZGyroOffset(gyroOffset[2]);
        imu.setXAccelOffset(accelOffset[0]);
        imu.setYAccelOffset(accelOffset[1]);
        imu.setZAccelOffset(accelOffset[2]);
        // fine-tune
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        imu.setDMPEnabled(true);
    }

    void drive_by_wire_init() {
        // Initialize drive by wire
        steeringServo.attach(STEER_PIN);      // Steer servo connected
        eSC.attach(ACCEL_BRAKE_PIN);          // ESC connected
        // centering the servo and ESC
        steeringServo.writeMicroseconds((int)rallycar_config::steerServoZero);
        eSC.writeMicroseconds((int)rallycar_config::escZero);
        //Setup the interrput pin to read the speed limiter (trim) signal;
        pinMode(SPEED_LIMITER_PIN, INPUT_PULLUP);
        // External hardware interrupt is attached to the pin and the interrupt is set to trigger on 'CHANGE'
        // in state of pin. On trigger the Interrupt service routine trimPulseTimer is called.
        attachInterrupt(digitalPinToInterrupt(SPEED_LIMITER_PIN), trimPulseTimer, CHANGE);
    }

    void publish_imu_when_ready(){
        const uint16_t mpuDmpPacketSize = imu.dmpGetFIFOPacketSize();
        uint16_t fifoCount = imu.getFIFOCount();// get current FIFO count
        // packets not ready yet
        if (fifoCount < mpuDmpPacketSize) return;

        imu_data.stamp = this->get_clock()->now();
        uint8_t mpuIntStatus = imu.getIntStatus();
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & (1U << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024U) {
            // reset so we can continue cleanly
            imu.resetFIFO();
            // Serial.println(F("FIFO overflow!"));
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & (1U << MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // read all available packets from FIFO
            while (fifoCount >= mpuDmpPacketSize) {
              // Lets catch up to NOW, in case someone is using the dreaded delay()!
              imu.getFIFOBytes(fifoBuffer, mpuDmpPacketSize);
              // track FIFO count here in case there is > 1 packet available
              // (this lets us immediately read more without waiting for an interrupt)
              fifoCount -= mpuDmpPacketSize;
            }
            // global_fifo_count = imu.getFIFOCount(); //should be zero here

            bool has_update = false;

            // get orientation in quaternion
            int16_t outRaw[4];
            if (!imu.dmpGetQuaternion(outRaw, fifoBuffer)) {
                const float scalingFactor = 1./16384.0f;
                imu_data.orientation.w = (float)outRaw[0] * scalingFactor;
                imu_data.orientation.x = (float)outRaw[1] * scalingFactor;
                imu_data.orientation.y = (float)outRaw[2] * scalingFactor;
                imu_data.orientation.z = (float)outRaw[3] * scalingFactor;
                has_update = true;
            }

            // get acceleration vector
            if (!imu.dmpGetAccel(outRaw, fifoBuffer)) {
                const float scalingFactor = 9.80665f / 16384.0f;
                imu_data.linear_acceleration.x = outRaw[0] * scalingFactor;
                imu_data.linear_acceleration.y = outRaw[1] * scalingFactor;
                imu_data.linear_acceleration.z = outRaw[2] * scalingFactor;
                has_update = true;
            }

            // get angular velocity vector
            if (!imu.dmpGetGyro(outRaw, fifoBuffer)) {
                const float scalingFactor = M_PI / (16.384 * 180.0);
                imu_data.angular_velocity.x = outRaw[0] * scalingFactor;
                imu_data.angular_velocity.y = outRaw[1] * scalingFactor;
                imu_data.angular_velocity.z = outRaw[2] * scalingFactor;
                has_update = true;
            }

            // if we have new data and time is synced, send updated message over serial
            if (has_update && state == NodeState::RUNNING)
                imu_pub.publish(imu_data);
        }
    }
};

#endif  // _RALLYCAR_DRIVER_MCU_NODE_HPP_
