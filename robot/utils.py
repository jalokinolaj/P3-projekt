
from robot.ina219_driver import INA219  # Import from ina219 battery driver found on waveshare wiki.

def get_voltage():
    """Finds the battery voltage on adress 0x41."""
    try:
        # Initialize with i2c bus 1 and adress 0x41 for batteries
        ina = INA219(i2c_bus=1, addr=0x41)
        
        # Get bus voltage and shut voltage
        bus_voltage = ina.getBusVoltage_V()
        shunt_voltage = ina.getShuntVoltage_mV() / 1000.0
        
        # Total voltage shunt + bus
        total_voltage = bus_voltage + shunt_voltage
        return float(total_voltage)
        
    except Exception as e:
        print(f"[[Error] could not find voltage] {e}")
        return 12.6  # Fallback-standard value