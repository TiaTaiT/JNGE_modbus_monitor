"""
Real-time monitoring script for JNGE MPPT Controller
"""

import time
import sys
from datetime import datetime
from jnge_controller import JNGEController, ChargeState, LoadStatus


def clear_lines(num_lines):
    """Clear previous lines (for updating display in place)"""
    for _ in range(num_lines):
        sys.stdout.write('\033[F\033[K')


def format_timestamp():
    """Get formatted timestamp"""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def display_monitoring(params, clear_previous=False):
    """Display monitoring data in simple format"""
    if clear_previous:
        clear_lines(12)
    
    # Calculate totals
    total_power = params['pv_power'] + params['fan_power']
    total_energy = params['pv_total_energy'] + params['fan_total_energy']
    
    # Charge state indicator
    charge_indicators = {
        ChargeState.UNCHARGED: "Not Charging",
        ChargeState.CONSTANT_CURRENT: "Constant Current",
        ChargeState.BOOST_CHARGING: "Boost Charging",
        ChargeState.FLOAT_CHARGING: "Float Charging"
    }
    charge_str = charge_indicators.get(params['charge_state'], "Unknown")
    
    # Load status
    load_str = "ON" if params['load_status'] == LoadStatus.BOOT_UP else "OFF"
    
    print(f"\n[{format_timestamp()}]")
    print(f"Battery:  {params['battery_voltage']:.1f}V  |  {charge_str}")
    print(f"Solar:    {params['pv_voltage']:.1f}V  {params['pv_current']:.1f}A  {params['pv_power']}W")
    print(f"Wind:     {params['fan_voltage']:.1f}V  {params['fan_current']:.1f}A  {params['fan_power']}W")
    print(f"Total:    {total_power}W  |  Energy: {total_energy:.1f}kWh  |  Load: {load_str}")
    
    # Show faults if any
    if params['error_code'] != 0:
        faults = []
        fault_codes = {
            0: "PV overcurrent", 1: "Short circuit", 3: "Battery overvoltage",
            4: "PV overvoltage", 5: "Fan overvoltage", 6: "Fan overcurrent",
            12: "Battery undervoltage", 14: "PV undervoltage"
        }
        for bit, desc in fault_codes.items():
            if params['error_code'] & (1 << bit):
                faults.append(desc)
        print(f"Faults:   {', '.join(faults)}")
    else:
        print("Faults:   None")
    
    print("-" * 70)


def show_system_info(controller):
    """Display system information"""
    print("\n" + "=" * 70)
    print("JNGE MPPT CONTROLLER - SYSTEM INFO")
    print("=" * 70)
    
    device_info = controller.read_device_info()
    if device_info:
        print(f"\nDevice Type: 0x{device_info['device_type']:04X}")
        print(f"Device Number: {device_info['device_number']}")
    
    settings = controller.read_system_settings()
    if settings:
        print(f"\nBattery Type: {settings['battery_type'].name}")
        print(f"Overvoltage: {settings['overvoltage']:.1f}V  |  Undervoltage: {settings['undervoltage']:.1f}V")
        print(f"Boost: {settings['boost_voltage']:.1f}V  |  Float: {settings['float_voltage']:.1f}V")
        print(f"Operating Mode: {settings['operating_mode'].name}")
    
    params = controller.read_operating_parameters()
    if params:
        print(f"\nPV Rating: {params['pv_rating']}W  |  Fan Rating: {params['fan_rating']}W")
        print(f"Firmware Version: {params['version']}")
    
    print("=" * 70)


def monitor_loop(controller, interval=2.0):
    """Main monitoring loop"""
    print("\nStarting real-time monitoring (Ctrl+C to stop)...")
    print("=" * 70)
    
    iteration = 0
    try:
        while True:
            params = controller.read_operating_parameters()
            
            if params:
                display_monitoring(params, clear_previous=(iteration > 0))
                iteration += 1
            else:
                print(f"[{format_timestamp()}] Failed to read parameters")
            
            time.sleep(interval)
            
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped")


def main():
    """Main function"""
    # Configuration
    PORT = 'COM4'  # Change to your port (COM3 for Windows)
    ADDRESS = 0x06
    UPDATE_INTERVAL = 2.0  # seconds
    
    print("JNGE MPPT Controller Monitor")
    print(f"Port: {PORT}  |  Address: 0x{ADDRESS:02X}")
    
    # Initialize controller
    controller = JNGEController(
        port=PORT,
        address=ADDRESS,
        timeout=1.0,
        inter_byte_timeout=0.1
    )
    
    # Connect
    print("\nConnecting...")
    if not controller.connect():
        print("Failed to connect to controller")
        return 1
    
    print("Connected successfully!")
    
    try:
        # Test communication
        test = controller.read_registers(0x1000, 0x01)
        if not test:
            print("Communication test failed")
            return 1
        
        # Show system info
        show_system_info(controller)
        
        # Wait for user
        input("\nPress Enter to start monitoring...")
        
        # Start monitoring
        monitor_loop(controller, interval=UPDATE_INTERVAL)
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        controller.disconnect()
        print("Disconnected")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())