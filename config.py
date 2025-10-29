"""
Configuration tool for JNGE MPPT Controller
"""

import sys
from jnge_controller import JNGEController, BatteryType


def show_current_settings(controller):
    """Display current settings"""
    print("\n" + "=" * 70)
    print("CURRENT SETTINGS")
    print("=" * 70)
    
    settings = controller.read_system_settings()
    if not settings:
        print("Failed to read settings")
        return False
    
    print(f"\nBattery Configuration:")
    print(f"  Type: {settings['battery_type'].name}")
    print(f"  Overvoltage Protection: {settings['overvoltage']:.1f}V")
    print(f"  Overvoltage Recovery: {settings['overvoltage_recovery']:.1f}V")
    print(f"  Boost Charging Voltage: {settings['boost_voltage']:.1f}V")
    print(f"  Boost Return Voltage: {settings['boost_return_voltage']:.1f}V")
    print(f"  Float Charging Voltage: {settings['float_voltage']:.1f}V")
    print(f"  Float Return Voltage: {settings['float_return_voltage']:.1f}V")
    print(f"  Undervoltage Protection: {settings['undervoltage']:.1f}V")
    print(f"  Undervoltage Recovery: {settings['undervoltage_recovery']:.1f}V")
    
    print(f"\nController Settings:")
    print(f"  Modbus Address: {settings['modbus_address']}")
    print(f"  Operating Mode: {settings['operating_mode'].name}")
    print(f"  Light Control On: {settings['light_ctrl_on_voltage']:.1f}V")
    print(f"  Light Control Off: {settings['light_ctrl_off_voltage']:.1f}V")
    
    print(f"\nSwitches:")
    print(f"  Charging: {'Enabled' if settings['charging_switch'] else 'Disabled'}")
    print(f"  Load: {'Enabled' if settings['load_switch'] else 'Disabled'}")
    
    print("=" * 70)
    return True


def change_battery_type(controller):
    """Interactive battery type change"""
    print("\nBattery Types:")
    print("  1. Lead-Acid")
    print("  2. LiFePO4")
    print("  3. Ternary Lithium")
    print("  4. Custom")
    
    choice = input("\nSelect battery type (1-4): ").strip()
    
    type_map = {
        '1': BatteryType.LEAD_ACID,
        '2': BatteryType.LIFEPO4,
        '3': BatteryType.TERNARY_LITHIUM,
        '4': BatteryType.CUSTOMIZE
    }
    
    if choice not in type_map:
        print("Invalid choice")
        return False
    
    battery_type = type_map[choice]
    print(f"\nSetting battery type to {battery_type.name}...")
    
    if controller.set_battery_type(battery_type):
        print("Success!")
        return True
    else:
        print("Failed to set battery type")
        return False


def change_voltage_settings(controller):
    """Interactive voltage settings change"""
    print("\nVoltage Settings:")
    print("  1. Overvoltage Protection")
    print("  2. Undervoltage Protection")
    print("  3. Boost Charging Voltage")
    print("  4. Float Charging Voltage")
    
    choice = input("\nSelect setting to change (1-4): ").strip()
    
    if choice not in ['1', '2', '3', '4']:
        print("Invalid choice")
        return False
    
    try:
        voltage = float(input("Enter voltage value (V): ").strip())
    except ValueError:
        print("Invalid voltage value")
        return False
    
    if choice == '1':
        print(f"\nSetting overvoltage to {voltage:.1f}V...")
        success = controller.set_overvoltage(voltage)
    elif choice == '2':
        print(f"\nSetting undervoltage to {voltage:.1f}V...")
        success = controller.set_undervoltage(voltage)
    elif choice == '3':
        print(f"\nSetting boost voltage to {voltage:.1f}V...")
        success = controller.write_register(0x1026, int(voltage * 10))
    elif choice == '4':
        print(f"\nSetting float voltage to {voltage:.1f}V...")
        success = controller.write_register(0x1028, int(voltage * 10))
    
    if success:
        print("Success!")
        return True
    else:
        print("Failed to set voltage")
        return False


def toggle_load(controller):
    """Toggle load output"""
    params = controller.read_operating_parameters()
    if not params:
        print("Failed to read current status")
        return False
    
    current_state = params['load_switch']
    new_state = not current_state
    
    print(f"\nLoad is currently: {'ON' if current_state else 'OFF'}")
    print(f"Switching to: {'ON' if new_state else 'OFF'}...")
    
    if controller.enable_load(new_state):
        print("Success!")
        return True
    else:
        print("Failed to toggle load")
        return False


def interactive_menu(controller):
    """Interactive configuration menu"""
    while True:
        print("\n" + "=" * 70)
        print("JNGE CONTROLLER CONFIGURATION")
        print("=" * 70)
        print("\n1. Show Current Settings")
        print("2. Change Battery Type")
        print("3. Change Voltage Settings")
        print("4. Toggle Load Output")
        print("5. Exit")
        
        choice = input("\nSelect option (1-5): ").strip()
        
        if choice == '1':
            show_current_settings(controller)
        elif choice == '2':
            change_battery_type(controller)
        elif choice == '3':
            change_voltage_settings(controller)
        elif choice == '4':
            toggle_load(controller)
        elif choice == '5':
            print("\nExiting...")
            break
        else:
            print("\nInvalid option")


def main():
    """Main function"""
    # Configuration
    PORT = 'COM4'  # Change to your port
    ADDRESS = 0x06
    
    print("JNGE MPPT Controller Configuration Tool")
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
        
        # Show device info
        device_info = controller.read_device_info()
        if device_info:
            print(f"\nDevice Type: 0x{device_info['device_type']:04X}")
            print(f"Device Number: {device_info['device_number']}")
        
        # Run interactive menu
        interactive_menu(controller)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
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