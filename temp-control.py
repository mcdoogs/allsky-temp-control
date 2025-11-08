import time
import sys
import math
import board
import digitalio
import adafruit_sht31d

sys.stdout.reconfigure(line_buffering=True)

# Format timestamp
print("Dew heater circuit running")

# Initialize heater control pin
heater_pin = digitalio.DigitalInOut(board.D18)
heater_pin.direction = digitalio.Direction.OUTPUT

# Initialize SHT30 sensor on I2C
i2c = board.I2C()
sht30_device = adafruit_sht31d.SHT31D(i2c)

# Configuration
DEW_POINT_THRESHOLD = 5.0  # Turn on heater when within 5°C of dew point
HYSTERESIS = 1.0  # Turn off when 6°C above dew point (5 + 1)
SENSOR_READ_INTERVAL = 30  # Read sensor every 30 seconds
DEFROST_CYCLE_INTERVAL = 360 # SHT30 heater cycle interval in seconds
DEFROST_CYCLE_LENGTH = 60 # SHT30 heater cycle length in seconds
# DEFROST_CYCLE_COOLDOWN = 90 # Time before temperature readings are valid after defrost
DEFROST_CYCLE_COOLDOWN = 90 # Time before temperature readings are valid after defrost
DEFROST_CYCLE_MARGIN = 7.5 # Only run defrost cycle if dew point margin is below this value

def calculate_dew_point(temp_c, humidity):
    """
    Calculate dew point using Magnus formula
    Returns dew point in Celsius, or None if calculation fails
    """
    try:
        # Validate inputs
        if not isinstance(temp_c, (int, float)) or not isinstance(humidity, (int, float)):
            return None
        
        if humidity <= 0 or humidity > 100:
            return None
            
        a = 17.27
        b = 237.7
        
        alpha = ((a * temp_c) / (b + temp_c)) + math.log(humidity / 100.0)
        dew_point = (b * alpha) / (a - alpha)
        
        return dew_point
    except (ZeroDivisionError, ValueError, TypeError):
        return None

heater_state = False
last_read_time = 0
last_defrost_time = 0

while True:
    try:
        current_time = time.time()
        
        # Read sensor at specified interval
        if current_time - last_read_time >= SENSOR_READ_INTERVAL:
            # Read temperature and humidity
            temp_raw = sht30_device.temperature
            humidity_raw = sht30_device.relative_humidity
            
            # Handle different return types and validate readings
            if temp_raw is None or humidity_raw is None:
                print("Sensor read failed - skipping this cycle")
                last_read_time = current_time
                continue
            
            # Extract single values if lists are returned
            if isinstance(temp_raw, list):
                if len(temp_raw) == 0:
                    print("Empty temperature list - skipping this cycle")
                    last_read_time = current_time
                    continue
                temperature_c = temp_raw[0]  # Use first reading
            else:
                temperature_c = temp_raw
            
            if isinstance(humidity_raw, list):
                if len(humidity_raw) == 0:
                    print("Empty humidity list - skipping this cycle")
                    last_read_time = current_time
                    continue
                humidity = humidity_raw[0]  # Use first reading
            else:
                humidity = humidity_raw
            
            # Validate final values are numbers
            if not isinstance(temperature_c, (int, float)) or not isinstance(humidity, (int, float)):
                print("Invalid sensor data types - skipping this cycle")
                last_read_time = current_time
                continue

            # Calculate dew point
            dew_point = calculate_dew_point(temperature_c, humidity)
            
            # Validate dew point calculation
            if dew_point is None or not isinstance(dew_point, (int, float)):
                print("Dew point calculation failed - skipping this cycle")
                last_read_time = current_time
                continue

            dew_point_margin = temperature_c - dew_point

            # Format timestamp
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(current_time))
            
            # Print status
            print(f"[{timestamp}] Temp: {temperature_c:.1f}°C, Humidity: {humidity:.1f}%, "
                  f"Dew Point: {dew_point:.1f}°C, Margin: {dew_point_margin:.1f}°C")
            
            # Control heater with hysteresis - use absolute margin
            if not heater_state and dew_point_margin < DEW_POINT_THRESHOLD:
                heater_state = True
                heater_pin.value = True
                print("→ Dome Heater ON")
            elif heater_state and dew_point_margin > (DEW_POINT_THRESHOLD + HYSTERESIS):
                heater_state = False
                heater_pin.value = False
                print("→ Dome Heater OFF")
            else:
                print(f"→ Dome Heater {'ON' if heater_state else 'OFF'} (no change)")
            
            last_read_time = current_time

            if (current_time - last_defrost_time >= DEFROST_CYCLE_INTERVAL):
                if (dew_point_margin < DEFROST_CYCLE_MARGIN):
                    # Format timestamp
                    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(current_time))
                    print(f"[{timestamp}] Starting SHT30 defrost cycle")
                    sht30_device.heater = True
                    time.sleep(DEFROST_CYCLE_LENGTH)
                    sht30_device.heater = False
                    last_defrost_time = current_time
                    print(f"[{timestamp}] SHT30 defrost cycle complete, cooling down..")
                    # Wait for cooldown period before taking readings
                    time.sleep(DEFROST_CYCLE_COOLDOWN)
                last_defrost_time = current_time
            last_read_time = current_time  # Reset read timer after defrost
        
        time.sleep(1)  # Small delay to prevent CPU spinning
        
    except OSError as error:
        # I2C communication errors
        print(f"Sensor read error: {error}")
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        heater_pin.value = False
        break
        
    except Exception as error:
        print(f"Unexpected error: {error}")
        heater_pin.value = False
        raise
