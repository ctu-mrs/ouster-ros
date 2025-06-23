#!/usr/bin/env python3

"""
Silent Ouster discovery - machine readable output only
Output format: "serial_number ip_address"
"""

import subprocess
import re
import sys

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
        # print("✓ Sensor on link-local network, no routing needed")
        return True
    
    # Non-link-local IP, need to add route via link-local interface
    ip_parts = sensor_ip.split('.')
    network = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.0/24"
    
    # print(f"Setting up route to {network} via {interface}...")
    
    # Add route
    try:
        subprocess.run([
            'sudo', 'route', 'add', '-net', network, 'dev', interface
        ], capture_output=True, timeout=5)
        # print(f"✓ Added route: {network} -> {interface}")
        return True
    except:
        # Try alias IP method
        try:
            alias_ip = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.1"
            subprocess.run([
                'sudo', 'ifconfig', f'{interface}:0', alias_ip, 'netmask', '255.255.255.0'
            ], capture_output=True, timeout=5)
            # print(f"✓ Added alias IP: {alias_ip} -> {interface}")
            return True
        except:
            # print(f"✗ Failed to set up routing to {sensor_ip}")
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
    target_serial = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] != 'any' else None
    
    # print("Link-Local Ouster Discovery")
    # print("===========================")
    
    # 1. Find link-local interface
    interface, interface_ip = find_link_local_interface()
    if not interface:
        # print("✗ No link-local interface found (169.254.x.x)")
        # print("Make sure your Ouster is connected and link-local is configured")
        sys.exit(1)
    
    # print(f"✓ Found link-local interface: {interface} ({interface_ip})")
    
    # 2. Discover sensor (any IP)
    # print("Discovering sensor...")
    sensor = discover_ouster()
    
    if not sensor:
        # print("✗ No Ouster sensor found")
        # print("Make sure sensor is powered and broadcasting")
        sys.exit(1)
    
    sensor_ip = sensor['ip']
    sensor_serial = sensor.get('serial', 'unknown')
    
    # Check if target serial matches
    if target_serial and sensor_serial != target_serial:
        # print(f"✗ Found sensor {sensor_serial}, but looking for {target_serial}")
        sys.exit(1)
    
    # print(f"✓ Found sensor: {sensor_serial} at {sensor_ip}")
    
    # 3. Set up routing if needed
    if setup_routing_if_needed(interface, sensor_ip):
        # print("✓ Network access configured")
        pass
    else:
        # print("⚠ Network setup failed")
        pass
    
    # 4. Test connectivity
    if test_sensor(sensor_ip):
        # print("✓ Sensor is accessible!")
        pass
    else:
        # print("⚠ Sensor not accessible")
        pass
    
    # 5. Output result in machine readable format
    print(f"{sensor_serial} {sensor_ip}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        # print("Usage: python3 silent_ouster.py <serial_number|any>")
        # print("Example: python3 silent_ouster.py any")
        # print("         python3 silent_ouster.py 992040000160")
        # print()
        # print("Output format: serial_number ip_address")
        # print("Assumes: System has link-local interface (169.254.x.x)")
        # print("Sensor can be at any IP (link-local, static, etc.)")
        sys.exit(1)
    
    main()