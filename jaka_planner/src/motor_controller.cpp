// motor_controller.cpp
#include "motor_controller.h"

MotorController::MotorController(const std::string& port, int baudrate) : slave_id_(0x01) {
    try {
        // Configure serial port parameters
        serial_.setPort(port);
        serial_.setBaudrate(baudrate);
        serial_.setBytesize(serial::eightbits);
        serial_.setParity(serial::parity_none);
        serial_.setStopbits(serial::stopbits_one);
        serial_.setFlowcontrol(serial::flowcontrol_none);
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port: %s", e.what());
    }
}

MotorController::~MotorController() {
    if (serial_.isOpen())
        serial_.close();
}

bool MotorController::init() {
    try {
        if (!serial_.isOpen())
            serial_.open();
            
        // Set timeout parameters
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100);  // 100ms timeout
        serial_.setTimeout(timeout);
        
        // Flush buffers
        serial_.flushInput();
        serial_.flushOutput();
        
        // Wait for serial port to stabilize
        ros::Duration(0.1).sleep();
        
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port: %s", e.what());
        return false;
    }
    return true;
}

bool MotorController::setIOStop(bool stop) {
    // Build Modbus message
    uint16_t address = 0x0011;
    std::vector<uint8_t> data = {static_cast<uint8_t>(stop ? 0xFF : 0x00), 0x00};
    auto message = buildModbusMessage(0x05, address, data);
    
    ROS_INFO("Setting IO Stop state: %s", stop ? "STOP" : "RUN");
    return sendAndReceive(message);
}

bool MotorController::setTorque(float torque) {
    // Build Modbus message - Address 0x08F2
    uint16_t address = 0x08F2;
    auto data = floatToModbus(torque);
    auto message = buildModbusMessage(0x10, address, data);
    
    ROS_INFO("Setting torque: %.2f", torque);
    return sendAndReceive(message);
}

bool MotorController::setAcceleration(float acc) {
    // Build Modbus message - Address 0x08F0
    uint16_t address = 0x08F0;
    auto data = floatToModbus(acc);
    auto message = buildModbusMessage(0x10, address, data);
    
    ROS_INFO("Setting acceleration: %.2f", acc);
    return sendAndReceive(message);
}

bool MotorController::setVelocity(float vel) {
    // Build Modbus message - Address 0x08EE
    uint16_t address = 0x08EE;
    auto data = floatToModbus(vel);
    auto message = buildModbusMessage(0x10, address, data);
    
    ROS_INFO("Setting velocity: %.2f", vel);
    return sendAndReceive(message);
}

bool MotorController::setPosition(float pos) {
    // Build Modbus message - Address 0x08EC
    uint16_t address = 0x08EC;
    auto data = floatToModbus(pos);
    auto message = buildModbusMessage(0x10, address, data);
    
    ROS_INFO("Setting position: %.2f", pos);
    return sendAndReceive(message);
}

float MotorController::getCurrentPosition() {
    // Build Modbus message - Address 0x0000
    uint16_t address = 0x0000;
    std::vector<uint8_t> data = {0x00, 0x02};  // Read 2 registers
    auto message = buildModbusMessage(0x04, address, data);
    
    if (sendAndReceive(message)) {
        // Parse returned position data
        // TODO: Implement actual position data parsing
        return 0.0;
    }
    return -1.0;
}

std::vector<uint8_t> MotorController::buildModbusMessage(uint8_t functionCode, 
                                                        uint16_t address, 
                                                        const std::vector<uint8_t>& data) {
    std::vector<uint8_t> message;
    
    // Build basic message structure
    message.push_back(slave_id_);           // Slave address (0x01)
    message.push_back(functionCode);        // Function code (0x10)
    message.push_back(address >> 8);        // Address high byte
    message.push_back(address & 0xFF);      // Address low byte
    
    // Add data
    message.insert(message.end(), data.begin(), data.end());
    
    // Calculate and add CRC
    uint16_t crc = calculateCRC(message);
    message.push_back(crc & 0xFF);         // CRC low byte
    message.push_back(crc >> 8);           // CRC high byte
    
    return message;
}

uint16_t MotorController::calculateCRC(const std::vector<uint8_t>& message) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < message.size(); pos++) {
        crc ^= (uint16_t)message[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


void MotorController::printMessage(const std::vector<uint8_t>& message, bool isSend) {
    std::stringstream ss;
    ss << (isSend ? "Send data: " : "Receive data: ");
    for (uint8_t byte : message) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(byte) << " ";
    }
    ROS_INFO("%s", ss.str().c_str());
}

bool MotorController::sendAndReceive(const std::vector<uint8_t>& sendMsg) {
    try {
        // Print and send data
        printMessage(sendMsg, true);
        serial_.write(sendMsg);
        
        // Set timeout for reading
        serial_.setTimeout(serial::Timeout::max(), 50, 0, 50, 0);
        
        // Read response based on function code
        std::vector<uint8_t> response;
        size_t expectedBytes = 0;
        
        // Calculate expected response length based on function code
        uint8_t functionCode = sendMsg[1];
        switch(functionCode) {
            case 0x05:  // Write Single Coil
                expectedBytes = 8;
                break;
            case 0x10:  // Write Multiple Registers
                expectedBytes = 8;
                break;
            case 0x04:  // Read Input Registers
                expectedBytes = 7;
                break;
            default:
                expectedBytes = 8;
                break;
        }
        
        // Read with timeout
        size_t bytesRead = serial_.read(response, expectedBytes);
        
        if (bytesRead == expectedBytes) {
            printMessage(response, false);
            // Verify CRC
            if (bytesRead >= 2) {
                std::vector<uint8_t> messageWithoutCRC(response.begin(), response.end() - 2);
                uint16_t calculatedCRC = calculateCRC(messageWithoutCRC);
                uint16_t receivedCRC = (response[bytesRead-1] << 8) | response[bytesRead-2];
                
                if (calculatedCRC != receivedCRC) {
                    ROS_ERROR("CRC check failed. Calculated: 0x%04X, Received: 0x%04X", 
                             calculatedCRC, receivedCRC);
                    return false;
                }
            }
            return true;
        } else {
            ROS_ERROR("Read timeout. Expected %zu bytes, got %zu bytes", 
                     expectedBytes, bytesRead);
            return false;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("Communication error: %s", e.what());
        return false;
    }
}

// std::vector<uint8_t> MotorController::floatToModbus(float value) {
//     std::vector<uint8_t> result;
//     result.push_back(0x00);  // Register count high byte
//     result.push_back(0x02);  // Register count low byte
//     result.push_back(0x04);  // Byte count
    
//     // Convert float to 4-byte data
//     uint32_t raw = *reinterpret_cast<uint32_t*>(&value);
//     result.push_back((raw >> 24) & 0xFF);
//     result.push_back((raw >> 16) & 0xFF);
//     result.push_back((raw >> 8) & 0xFF);
//     result.push_back(raw & 0xFF);
    
//     return result;
// }
std::vector<uint8_t> MotorController::floatToModbus(float value) {
    std::vector<uint8_t> result;
    
    // First add register count (2 registers)
    result.push_back(0x00);  // Register count high byte
    result.push_back(0x02);  // Register count low byte
    
    // Add byte count
    result.push_back(0x04);  // Byte count (4 bytes for float)
    
    // Convert float to IEEE-754 representation
    union {
        float f;
        uint32_t i;
    } converter;
    
    converter.f = value;
    uint32_t raw = converter.i;
    
    // Add float bytes in the correct order (00 00 followed by upper bytes of float)
    result.push_back((raw >> 8) & 0xFF);                  // First byte: always 0x00
    result.push_back(raw & 0xFF);                  // Second byte: always 0x00
    result.push_back((raw >> 24) & 0xFF);    // Third byte: MSB of IEEE-754
    result.push_back((raw >> 16) & 0xFF);    // Fourth byte: Next byte of IEEE-754
    
    return result;
}