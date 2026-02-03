#!/usr/bin/env python3
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
LEAP Hand Serial Port Test Tool

Usage:
    ros2 run openarm_hardware test_leap_serial_port.py
    ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0
    ros2 run openarm_hardware test_leap_serial_port.py --scan
"""

import argparse
import glob
import sys
import time
import serial
import serial.tools.list_ports


class LeapHandSerialTester:
    """Test tool for LEAP Hand serial port communication"""
    
    BAUDRATE = 4000000  # 4 Mbps for Dynamixel
    TIMEOUT = 1.0
    
    def __init__(self):
        self.ser = None
        
    def scan_serial_ports(self):
        """Scan for available serial ports"""
        print("\n" + "="*60)
        print("Scanning for available serial ports...")
        print("="*60)
        
        # Method 1: Using serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if ports:
            print("\n✓ Found ports using serial.tools.list_ports:")
            for port in ports:
                print(f"  • {port.device}")
                print(f"    Description: {port.description}")
                print(f"    Hardware ID: {port.hwid}")
        else:
            print("\n✗ No ports found using serial.tools.list_ports")
        
        # Method 2: Using glob pattern
        print("\n✓ Checking common USB serial ports:")
        common_ports = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/ttyS*',
            '/dev/serial/by-id/*'
        ]
        
        found_any = False
        for pattern in common_ports:
            matching_ports = glob.glob(pattern)
            if matching_ports:
                found_any = True
                print(f"\n  Pattern: {pattern}")
                for port in matching_ports:
                    print(f"    • {port}")
        
        if not found_any:
            print("\n  ✗ No USB serial ports found")
        
        print("\n" + "="*60)
        
    def test_connect(self, port):
        """Test connecting to serial port"""
        print(f"\n{'='*60}")
        print(f"Testing connection to: {port}")
        print(f"Baudrate: {self.BAUDRATE}")
        print(f"Timeout: {self.TIMEOUT}s")
        print(f"{'='*60}\n")
        
        try:
            print(f"[1/3] Opening serial port {port}...")
            self.ser = serial.Serial(
                port=port,
                baudrate=self.BAUDRATE,
                timeout=self.TIMEOUT,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            print(f"✓ Serial port opened successfully")
            print(f"  Port: {self.ser.port}")
            print(f"  Baudrate: {self.ser.baudrate}")
            print(f"  Is open: {self.ser.is_open}")
            
            # Test if port is readable
            print(f"\n[2/3] Testing port readability...")
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                print(f"✓ Received {len(data)} bytes")
            else:
                print(f"✓ Port readable (no data waiting)")
            
            # Test if port is writable
            print(f"\n[3/3] Testing port writability...")
            test_data = b'\x00\x01\x02'
            bytes_written = self.ser.write(test_data)
            print(f"✓ Wrote {bytes_written} bytes")
            
            print(f"\n{'='*60}")
            print(f"✅ CONNECTION TEST PASSED")
            print(f"{'='*60}\n")
            
            return True
            
        except serial.SerialException as e:
            print(f"\n{'='*60}")
            print(f"❌ CONNECTION TEST FAILED")
            print(f"{'='*60}")
            print(f"Error: {e}")
            print(f"\nPossible issues:")
            print(f"  • Port {port} does not exist")
            print(f"  • Port is already in use by another process")
            print(f"  • Insufficient permissions (try: sudo chmod 666 {port})")
            print(f"  • Device not connected")
            print()
            return False
            
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            return False
    
    def test_disconnect(self):
        """Test disconnecting from serial port"""
        print(f"\n{'='*60}")
        print(f"Testing disconnection...")
        print(f"{'='*60}\n")
        
        if self.ser and self.ser.is_open:
            try:
                port_name = self.ser.port
                print(f"[1/2] Closing port {port_name}...")
                self.ser.close()
                print(f"✓ Port closed")
                
                print(f"\n[2/2] Verifying port is closed...")
                if not self.ser.is_open:
                    print(f"✓ Port is confirmed closed")
                    print(f"\n{'='*60}")
                    print(f"✅ DISCONNECTION TEST PASSED")
                    print(f"{'='*60}\n")
                    return True
                else:
                    print(f"✗ Port still appears open")
                    return False
                    
            except Exception as e:
                print(f"❌ Error during disconnect: {e}")
                return False
        else:
            print(f"⚠ No port is currently open")
            return False
    
    def test_reconnect(self, port, iterations=3):
        """Test multiple connect/disconnect cycles"""
        print(f"\n{'='*60}")
        print(f"Testing reconnection cycles (x{iterations})...")
        print(f"{'='*60}\n")
        
        success_count = 0
        for i in range(iterations):
            print(f"\n--- Cycle {i+1}/{iterations} ---")
            
            # Connect
            if self.test_connect(port):
                time.sleep(0.5)
                
                # Disconnect
                if self.test_disconnect():
                    success_count += 1
                    time.sleep(0.5)
                else:
                    print(f"✗ Cycle {i+1} failed at disconnect")
                    break
            else:
                print(f"✗ Cycle {i+1} failed at connect")
                break
        
        print(f"\n{'='*60}")
        print(f"Reconnection Test Summary:")
        print(f"  Success: {success_count}/{iterations}")
        print(f"  Result: {'✅ PASSED' if success_count == iterations else '❌ FAILED'}")
        print(f"{'='*60}\n")
        
        return success_count == iterations


def main():
    parser = argparse.ArgumentParser(
        description='LEAP Hand Serial Port Test Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Scan for available ports
  ros2 run openarm_hardware test_leap_serial_port.py --scan
  
  # Test specific port
  ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0
  
  # Test with reconnect cycles
  ros2 run openarm_hardware test_leap_serial_port.py --port /dev/ttyUSB0 --reconnect 5
        '''
    )
    
    parser.add_argument('--port', '-p', 
                       default='/dev/ttyUSB0',
                       help='Serial port to test (default: /dev/ttyUSB0)')
    
    parser.add_argument('--scan', '-s',
                       action='store_true',
                       help='Scan for available serial ports')
    
    parser.add_argument('--reconnect', '-r',
                       type=int,
                       metavar='N',
                       help='Test N reconnection cycles')
    
    parser.add_argument('--auto-find', '-a',
                       action='store_true',
                       help='Auto-find and test first available port')
    
    args = parser.parse_args()
    
    tester = LeapHandSerialTester()
    
    print("\n" + "="*60)
    print("LEAP Hand Serial Port Test Tool")
    print("="*60)
    
    # Scan mode
    if args.scan:
        tester.scan_serial_ports()
        return 0
    
    # Auto-find mode
    if args.auto_find:
        print("\nAuto-finding available ports...")
        ports = glob.glob('/dev/ttyUSB*')
        if not ports:
            ports = glob.glob('/dev/ttyACM*')
        
        if not ports:
            print("❌ No USB serial ports found")
            print("Run with --scan to see all available ports")
            return 1
        
        args.port = ports[0]
        print(f"✓ Found port: {args.port}")
    
    # Test mode
    if args.reconnect:
        # Reconnection test
        success = tester.test_reconnect(args.port, args.reconnect)
    else:
        # Single connection test
        if tester.test_connect(args.port):
            input("\nPress Enter to disconnect...")
            tester.test_disconnect()
            success = True
        else:
            success = False
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
