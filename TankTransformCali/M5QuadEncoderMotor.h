#include <Wire.h>
#include <Arduino.h>

#define M4E_ADDR_DEFAULT 0x26


class M5_4EncoderMotor {
    public:
        enum MotorMode : uint8_t {
            NORMAL_MODE   = 0,   // open loop PWM
            POSITION_MODE = 1,   // position lock
            SPEED_MODE    = 2    // speed lock
        };
        M5_4EncoderMotor(uint8_t addr = M4E_ADDR_DEFAULT);
        bool begin(TwoWire& w = Wire);
        bool ping();
        bool setAllDuty(const int8_t duty[4]);
        bool setDuty(uint8_t motor, int8_t duty);
        bool readEncoders(int32_t counts[4]);
        bool setEncoder(uint8_t motor, int32_t value);
        bool readSpeeds(int8_t speed[4]);
        bool setMode(uint8_t motor, MotorMode mode);
        bool setPositionPID(uint8_t motor, int8_t p, int8_t i, int8_t d);
        bool setPositionPoint(uint8_t motor, int32_t point);
        bool setPositionMaxSpeed(uint8_t motor, int8_t maxSpeed);
        bool setSpeedPID(uint8_t motor, int8_t p, int8_t i, int8_t d);
        bool setSpeedPoint(uint8_t motor, int8_t speedPoint);
        bool getCurrentX100(int32_t& current_x100);
        bool getCurrentA(float& amps);
        bool getVin8(uint8_t& adc8);
        bool getVin12(uint16_t& adc12);
        bool getVoltage(float& vin);
        bool setEncoderMode(bool ba);
        bool enableSoftStartStop(uint8_t mask);
        bool getFirmwareVersion(uint8_t& ver);
        bool setI2CAddress(uint8_t newAddr);
        void printEncoders();
        void printTelemetry();
    private:
        uint8_t _addr;
        TwoWire* _wire;
        uint8_t modeBase(uint8_t motor);
        bool writeUint8(uint8_t reg, uint8_t val);
        bool writeInt8(uint8_t reg, int8_t val);
        bool writeBytes(uint8_t reg, const uint8_t* data, size_t len);
        bool readBytes(uint8_t reg, uint8_t* data, size_t len);
};