#!/usr/bin/env python3

"""
Simple mDNS debug tool to understand what's happening
"""

import socket
import struct
import time
import sys

def test_basic_connectivity():
    """Test basic network connectivity"""
    print("=== Basic Network Test ===")
    
    try:
        # Test if we can create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("✓ UDP socket creation works")
        
        # Test if we can bind to a port
        sock.bind(('', 0))
        port = sock.getsockname()[1]
        print(f"✓ Can bind to port {port}")
        
        sock.close()
        
    except Exception as e:
        print(f"✗ Basic network test failed: {e}")
        return False
    
    return True

def get_interfaces_simple():
    """Get interfaces using simple methods"""
    print("\n=== Interface Detection ===")
    
    interfaces = []
    
    # Method 1: Try subprocess
    try:
        import subprocess
        result = subprocess.run(['ifconfig'], capture_output=True, text=True, timeout=3)
        if result.returncode == 0:
            print("✓ ifconfig command works")
            
            current_iface = None
            for line in result.stdout.split('\n'):
                if line and not line.startswith(' '):
                    # Interface line
                    current_iface = line.split()[0].rstrip(':')
                    if current_iface != 'lo':
                        print(f"  Found interface: {current_iface}")
                elif 'inet ' in line and current_iface:
                    # IP line
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == 'inet' and i + 1 < len(parts):
                            ip = parts[i + 1].replace('addr:', '')
                            if not ip.startswith('127.'):
                                interfaces.append((current_iface, ip))
                                print(f"    IP: {ip}")
                            break
        else:
            print("✗ ifconfig command failed")
    except Exception as e:
        print(f"✗ ifconfig method failed: {e}")
    
    # Method 2: Try /proc/net/dev
    try:
        with open('/proc/net/dev', 'r') as f:
            content = f.read()
            print("✓ /proc/net/dev accessible")
            
            for line in content.split('\n'):
                if ':' in line and 'lo:' not in line:
                    iface = line.split(':')[0].strip()
                    if iface:
                        print(f"  /proc interface: {iface}")
    except Exception as e:
        print(f"✗ /proc/net/dev method failed: {e}")
    
    print(f"Total interfaces found: {len(interfaces)}")
    return interfaces

def test_multicast_join():
    """Test if we can join multicast group"""
    print("\n=== Multicast Test ===")
    
    MDNS_ADDR = '224.0.0.251'
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Try to bind to any address
        sock.bind(('', 0))
        print("✓ Socket bind successful")
        
        # Try to join multicast group
        mreq = struct.pack('4sl', socket.inet_aton(MDNS_ADDR), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        print("✓ Multicast group join successful")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ Multicast test failed: {e}")
        return False

def test_mdns_send():
    """Test sending mDNS query"""
    print("\n=== mDNS Send Test ===")
    
    MDNS_ADDR = '224.0.0.251'
    MDNS_PORT = 5353
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', 0))
        
        # Create simple mDNS query
        query = create_simple_query()
        
        # Send query
        sock.sendto(query, (MDNS_ADDR, MDNS_PORT))
        print("✓ mDNS query sent successfully")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ mDNS send failed: {e}")
        return False

def create_simple_query():
    """Create a simple mDNS query packet"""
    # Header: ID=0, flags=0, 1 question, 0 answers
    header = struct.pack('!HHHHHH', 0, 0, 1, 0, 0, 0)
    
    # Question: _roger._tcp.local PTR IN
    question = b''
    for part in ['_roger', '_tcp', 'local']:
        question += struct.pack('!B', len(part)) + part.encode()
    question += b'\x00'  # End of name
    question += struct.pack('!HH', 12, 1)  # PTR, IN
    
    return header + question

def listen_for_responses():
    """Listen for any mDNS responses"""
    print("\n=== mDNS Listen Test ===")
    
    MDNS_ADDR = '224.0.0.251'
    MDNS_PORT = 5353
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind to mDNS port
        try:
            sock.bind(('', MDNS_PORT))
            print("✓ Bound to mDNS port 5353")
        except Exception as e:
            print(f"⚠ Cannot bind to port 5353: {e}")
            sock.bind(('', 0))
            print(f"✓ Bound to random port {sock.getsockname()[1]}")
        
        # Join multicast group
        mreq = struct.pack('4sl', socket.inet_aton(MDNS_ADDR), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        print("✓ Joined multicast group")
        
        # Listen for a few seconds
        sock.settimeout(5.0)
        print("Listening for mDNS traffic for 5 seconds...")
        
        packet_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 5:
            try:
                data, addr = sock.recvfrom(4096)
                packet_count += 1
                print(f"  Received packet {packet_count} from {addr[0]} ({len(data)} bytes)")
                
                # Quick check if it might be an Ouster response
                if b'ouster' in data.lower() or b'roger' in data.lower():
                    print(f"    ★ Potential Ouster packet!")
                    print(f"    Data preview: {data[:100]}")
                
            except socket.timeout:
                continue
        
        print(f"Total packets received: {packet_count}")
        sock.close()
        
        return packet_count > 0
        
    except Exception as e:
        print(f"✗ mDNS listen failed: {e}")
        return False

def test_known_sensor():
    """Test connection to known sensor IP"""
    print("\n=== Known Sensor Test ===")
    
    known_ips = ['10.10.20.90', '169.254.168.90', '192.168.1.90']
    
    for ip in known_ips:
        print(f"Testing {ip}...")
        
        try:
            # Test TCP connection
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((ip, 80))
            sock.close()
            
            if result == 0:
                print(f"  ✓ {ip} port 80 is open")
                
                # Test HTTP request
                try:
                    import urllib.request
                    response = urllib.request.urlopen(f"http://{ip}/api/v1/sensor/info", timeout=3)
                    data = response.read().decode('utf-8')
                    print(f"  ✓ {ip} API responds: {data[:50]}...")
                    return ip
                except Exception as e:
                    print(f"  ⚠ {ip} HTTP failed: {e}")
            else:
                print(f"  ✗ {ip} port 80 closed")
                
        except Exception as e:
            print(f"  ✗ {ip} connection failed: {e}")
    
    return None

def main():
    print("Simple mDNS Debug Tool")
    print("=====================")
    
    # Run all tests
    tests = [
        ("Basic Connectivity", test_basic_connectivity),
        ("Interface Detection", lambda: len(get_interfaces_simple()) > 0),
        ("Multicast Join", test_multicast_join),
        ("mDNS Send", test_mdns_send),
        ("mDNS Listen", listen_for_responses),
    ]
    
    results = {}
    for test_name, test_func in tests:
        print(f"\n{'='*50}")
        try:
            result = test_func()
            results[test_name] = result
            print(f"Result: {'PASS' if result else 'FAIL'}")
        except Exception as e:
            results[test_name] = False
            print(f"Result: ERROR - {e}")
    
    # Test known sensor
    print(f"\n{'='*50}")
    sensor_ip = test_known_sensor()
    
    # Summary
    print(f"\n{'='*50}")
    print("SUMMARY")
    print("="*50)
    
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{test_name:20} {status}")
    
    if sensor_ip:
        print(f"\n★ Found working sensor at: {sensor_ip}")
        print("The sensor is reachable but mDNS discovery may have issues.")
    else:
        print(f"\n⚠ No working sensor found at common IPs")
    
    print(f"\nNext steps:")
    if not results.get("Multicast Join", False):
        print("- Multicast not working - may need root privileges")
        print("- Try: sudo python3 script.py")
    
    if not results.get("mDNS Listen", False):
        print("- No mDNS traffic detected")
        print("- Sensor may not be broadcasting")
        print("- Network may be filtering multicast")
    
    if sensor_ip:
        print(f"- Direct connection works, use: curl http://{sensor_ip}/api/v1/sensor/info")

if __name__ == '__main__':
    main()