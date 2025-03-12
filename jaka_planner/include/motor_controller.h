// motor_controller.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

class MotorController {
public:
    // Constructor with default port and baudrate parameters
    MotorController(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);
    ~MotorController();

    // Initialization functions
    bool init();
    
    // Position control related functions
    bool setTorque(float torque);       // Set motor torque
    bool setAcceleration(float acc);    // Set motor acceleration
    bool setVelocity(float vel);        // Set motor velocity
    bool setPosition(float pos);        // Set target position
    float getCurrentPosition();         // Get current position
    
    // IO control functions
    bool setIOStop(bool stop);          // Set IO stop state

private:
    serial::Serial serial_;             // Serial port object
    uint8_t slave_id_;                 // Slave address
    
    // Helper functions
    std::vector<uint8_t> buildModbusMessage(uint8_t functionCode, 
                                          uint16_t address, 
                                          const std::vector<uint8_t>& data);
    uint16_t calculateCRC(const std::vector<uint8_t>& message);
    void printMessage(const std::vector<uint8_t>& message, bool isSend);
    bool sendAndReceive(const std::vector<uint8_t>& sendMsg);
    std::vector<uint8_t> floatToModbus(float value);
};

#endif // MOTOR_CONTROLLER_H