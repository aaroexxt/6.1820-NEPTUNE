import serial
import serial.tools.list_ports
import time

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port for port in ports if port.device]

def choose_serial_port():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return None

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i + 1}: {port.device} - {port.description}")

    choice = input("Select a port by number: ")
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(ports):
            return ports[idx].device
        else:
            print("Invalid choice.")
            return None
    except ValueError:
        print("Invalid input.")
        return None

def log_serial_data(port, baudrate=115200):
    output_file = input("Enter the filename to save data (e.g., data_log.txt): ").strip()
    if not output_file:
        output_file = "serial_output.txt"
        print("No filename entered. Defaulting to 'serial_output.txt'.")

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"\nLogging data from {port} at {baudrate} baud to '{output_file}'...\n")

        with open(output_file, 'a') as f:
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    print(line)
                    f.write(line + '\n')
                    f.flush()
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    port = choose_serial_port()
    if port:
        log_serial_data(port)
