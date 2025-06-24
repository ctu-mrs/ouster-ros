#!/usr/bin/env python3

"""
Flexible Ouster discovery with human-readable and machine-readable output
"""

import subprocess
import re
import sys
import argparse

# Global flag for output mode
QUIET_MODE = False

def log(message):
    """Print message only if not in quiet mode"""
    if not QUIET_MODE:
        print(message)

def find_link_local_interface():
    """Find interface with link-local IP (169.254.x.x)"""
    try:
        result = subprocess.run(['ifconfig'], capture_output=True, text=True, timeout=3)
        current_iface = None
        
        for line in result.stdout.split('\n'):
            if line and not line.startswith(' ') and not line.startswith('\t'):
                current_iface = line.split(':')[0]
            elif 'inet 169.254.' in line and current_iface:
                # Found link-local IP
                ip_match = re.search(r'inet (169\.254\.[0-9]+\.[0-9]+)', line)
                if ip_match:
                    return current_iface, ip_match.group(1)
    except:
        pass
    return None, None

def discover_ouster():
    """Discover Ouster sensor using nmap (any IP)"""
    try:
        result = subprocess.run([
            'nmap', '--script=broadcast-dns-service-discovery'
        ], capture_output=True, text=True, timeout=10)
        
        # Parse for Ouster sensor at any IP
        for line in result.stdout.split('\n'):
            if 'roger' in line and '/tcp' in line:
                lines = result.stdout.split('\n')
                idx = lines.index(line)
                
                sensor = {}
                for i in range(idx + 1, min(idx + 10, len(lines))):
                    detail = lines[i].strip()
                    
                    if 'sn=' in detail:
                        sensor['serial'] = re.search(r'sn=([^\s]+)', detail).group(1)
                    elif 'Address=' in detail:
                        # Accept any IP address
                        match = re.search(r'Address=([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)', detail)
                        if match:
                            sensor['ip'] = match.group(1)
                
                if 'ip' in sensor:
                    return sensor
    except:
        pass
    return None

def setup_routing_if_needed(interface, sensor_ip):
    """Set up routing if sensor is not on link-local network"""
    
    if sensor_ip.startswith('169.254.'):
        # Link-local, no routing needed
        log("✓ Sensor on link-local network, no routing needed")
        return True
    
    # Non-link-local IP, need to add route via link-local interface
    ip_parts = sensor_ip.split('.')
    network = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.0/24"
    
    log(f"Setting up route to {network} via {interface}...")
    
    # Add route
    try:
        subprocess.run([
            'sudo', 'route', 'add', '-net', network, 'dev', interface
        ], capture_output=True, timeout=5)
        log(f"✓ Added route: {network} -> {interface}")
        return True
    except:
        # Try alias IP method
        try:
            alias_ip = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.1"
            subprocess.run([
                'sudo', 'ifconfig', f'{interface}:0', alias_ip, 'netmask', '255.255.255.0'
            ], capture_output=True, timeout=5)
            log(f"✓ Added alias IP: {alias_ip} -> {interface}")
            return True
        except:
            log(f"✗ Failed to set up routing to {sensor_ip}")
            return False

def test_sensor(sensor_ip):
    """Test if sensor is reachable"""
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((sensor_ip, 80))
        sock.close()
        return result == 0
    except:
        return False

def main():
    parser = argparse.ArgumentParser(description='Ouster sensor discovery')
    parser.add_argument('serial', nargs='?', default='any', 
                       help='Target serial number (or "any" for any sensor)')
    parser.add_argument('-q', '--quiet', action='store_true',
                       help='Machine readable output: only "serial_number ip_address"')
    parser.add_argument('-m', '--machine', action='store_true',
                       help='Alias for --quiet')
    
    args = parser.parse_args()
    
    # Set global quiet mode
    global QUIET_MODE
    QUIET_MODE = args.quiet or args.machine
    
    target_serial = None if args.serial == 'any' else args.serial
    
    log("Link-Local Ouster Discovery")
    log("===========================")
    
    # 1. Find link-local interface
    interface, interface_ip = find_link_local_interface()
    if not interface:
        log("✗ No link-local interface found (169.254.x.x)")
        log("Make sure your Ouster is connected and link-local is configured")
        sys.exit(1)
    
    log(f"✓ Found link-local interface: {interface} ({interface_ip})")
    
    # 2. Discover sensor (any IP)
    log("Discovering sensor...")
    sensor = discover_ouster()
    
    if not sensor:
        log("✗ No Ouster sensor found")
        log("Make sure sensor is powered and broadcasting")
        sys.exit(1)
    
    sensor_ip = sensor['ip']
    sensor_serial = sensor.get('serial', 'unknown')
    
    # Check if target serial matches
    if target_serial and sensor_serial != target_serial:
        log(f"✗ Found sensor {sensor_serial}, but looking for {target_serial}")
        sys.exit(1)
    
    log(f"✓ Found sensor: {sensor_serial} at {sensor_ip}")
    
    # 3. Set up routing if needed
    if setup_routing_if_needed(interface, sensor_ip):
        log("✓ Network access configured")
    else:
        log("⚠ Network setup failed")
    
    # 4. Test connectivity
    if test_sensor(sensor_ip):
        log("✓ Sensor is accessible!")
    else:
        log("⚠ Sensor not accessible")
    
    # 5. Output result
    if QUIET_MODE:
        # Machine readable format
        print(f"{sensor_serial} {sensor_ip} {interface_ip}")
    else:
        # Human readable format
        log(f"\nResult:")
        log(f"Interface: {interface}")
        log(f"Interface IP: {interface_ip}")
        log(f"Sensor IP: {sensor_ip}")
        log(f"Serial: {sensor_serial}")
        log(f"\nTest command: curl http://{sensor_ip}/api/v1/sensor/config")

if __name__ == '__main__':
    main()