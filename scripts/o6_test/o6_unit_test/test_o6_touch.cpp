// Test O6 touch sensor with LinkerHandApi
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "LinkerHandApi.h"

int main(int argc, char** argv) {
    std::cout << "=== Testing O6 Hand Touch Sensor ===" << std::endl;
    
    // Parse arguments
    std::string can_interface = "can0";
    std::string hand_type_str = "right";
    
    if (argc > 1) {
        can_interface = argv[1];
    }
    if (argc > 2) {
        hand_type_str = argv[2];
    }
    
    // Map CAN interface
    COMM_TYPE channel = (can_interface == "can0") ? COMM_TYPE::COMM_CAN_0 : COMM_TYPE::COMM_CAN_1;
    
    // Map hand type
    HAND_TYPE hand_type = (hand_type_str == "left") ? HAND_TYPE::LEFT : HAND_TYPE::RIGHT;
    
    std::cout << "CAN: " << can_interface << ", Hand: " << hand_type_str << std::endl;
    
    try {
        // Create O6 hand instance
        auto hand_api = std::make_unique<LinkerHandApi>(LINKER_HAND::O6, hand_type, channel);
        std::cout << "✓ LinkerHandApi created successfully" << std::endl;
        
        // Enable hand
        hand_api->setEnable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "✓ Hand enabled" << std::endl;
        
        // Get version
        std::string version = hand_api->getVersion();
        std::cout << "✓ Version: " << version << std::endl;
        
        // Test getForce() 
        std::cout << "\n=== Testing getForce() ===" << std::endl;
        
        for (int i = 0; i < 5; i++) {
            std::cout << "\nAttempt " << (i+1) << ":" << std::endl;
            
            try {
                auto force_data = hand_api->getForce();
                
                std::cout << "  getForce() returned: " << force_data.size() << " fingers" << std::endl;
                
                if (force_data.empty()) {
                    std::cout << "  ⚠ WARNING: force_data is EMPTY!" << std::endl;
                } else {
                    // Print structure
                    for (size_t f = 0; f < force_data.size(); ++f) {
                        std::cout << "  Finger[" << f << "]: " << force_data[f].size() << " rows" << std::endl;
                        
                        if (!force_data[f].empty()) {
                            std::cout << "    Row[0]: " << force_data[f][0].size() << " columns" << std::endl;
                            
                            // Print first 10 values of first row
                            std::cout << "    First values: ";
                            for (size_t c = 0; c < std::min(size_t(10), force_data[f][0].size()); ++c) {
                                std::cout << static_cast<int>(force_data[f][0][c]) << " ";
                            }
                            std::cout << std::endl;
                            
                            // Calculate total force for this finger
                            uint32_t total = 0;
                            for (const auto& row : force_data[f]) {
                                for (const auto& val : row) {
                                    total += val;
                                }
                            }
                            std::cout << "    Total force: " << total << std::endl;
                        }
                    }
                }
                
            } catch (const std::exception& e) {
                std::cout << "  ✗ Exception: " << e.what() << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // Disable hand
        hand_api->setDisable();
        std::cout << "\n✓ Hand disabled" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "✗ Error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    return 0;
}
