#!/usr/bin/env python3

"""
Standalone mDNS discovery tool - recreates avahi-browse functionality
Discovers Ouster sensors and automatically determines the correct interface
"""

import socket
import struct
import time
import threading
import sys
import subprocess
from collections import defaultdict
import select

class MDNSQuery:
    """Creates mDNS query packets"""
    
    def __init__(self):
        self.MDNS_ADDR = '224.0.0.251'
        self.MDNS_PORT = 5353
    
    def create_query_packet(self, service_name="_roger._tcp.local"):
        """Create mDNS query packet for PTR records"""
        # Transaction ID (0 for mDNS)
        tid = 0x0000
        
        # Flags: Standard query
        flags = 0x0000
        
        # Counts
        questions = 1
        answers = authority = additional = 0
        
        # Header
        header = struct.pack('!HHHHHH', tid, flags, questions, answers, authority, additional)
        
        # Question section
        question = self._encode_name(service_name)
        question += struct.pack('!HH', 12, 1)  # PTR query, IN class
        
        return header + question
    
    def _encode_name(self, name):
        """Encode domain name for DNS packet"""
        encoded = b''
        for part in name.split('.'):
            encoded += struct.pack('!B', len(part)) + part.encode('utf-8')
        encoded += b'\x00'  # End of name
        return encoded

class MDNSParser:
    """Parses mDNS response packets"""
    
    def __init__(self):
        self.sensors = defaultdict(dict)
    
    def parse_response(self, data, sender_addr, interface_name=None):
        """Parse mDNS response packet"""
        if len(data) < 12:
            return
        
        try:
            # Parse header
            tid, flags, questions, answers, authority, additional = struct.unpack('!HHHHHH', data[:12])
            
            offset = 12
            
            # Skip questions
            for _ in range(questions):
                name, offset = self._parse_name(data, offset)
                offset += 4  # Skip QTYPE and QCLASS
            
            # Parse all resource records
            total_records = answers + authority + additional
            
            for _ in range(total_records):
                if offset >= len(data):
                    break
                
                name, offset = self._parse_name(data, offset)
                
                if offset + 10 > len(data):
                    break
                
                rtype, rclass, ttl, rdlength = struct.unpack('!HHIH', data[offset:offset + 10])
                offset += 10
                
                if offset + rdlength > len(data):
                    break
                
                rdata = data[offset:offset + rdlength]
                offset += rdlength
                
                self._process_record(name, rtype, rdata, sender_addr, interface_name)
        
        except Exception as e:
            # Silently ignore malformed packets
            pass
    
    def _parse_name(self, data, offset):
        """Parse DNS name with compression support"""
        name_parts = []
        original_offset = offset
        jumped = False
        
        while offset < len(data):
            length = data[offset]
            
            if length == 0:
                offset += 1
                break
            elif (length & 0xC0) == 0xC0:  # Compression pointer
                if not jumped:
                    original_offset = offset + 2
                    jumped = True
                offset = ((length & 0x3F) << 8) | data[offset + 1]
                continue
            else:
                offset += 1
                if offset + length > len(data):
                    break
                name_parts.append(data[offset:offset + length].decode('utf-8', errors='ignore'))
                offset += length
        
        if jumped:
            offset = original_offset
        
        return '.'.join(name_parts), offset
    
    def _process_record(self, name, rtype, rdata, sender_addr, interface_name):
        """Process individual DNS records"""
        
        # Look for Ouster-related records
        if '_roger._tcp.local' in name and any(keyword in name.lower() for keyword in ['ouster', 'sensor']):
            
            if rtype == 12:  # PTR record
                target_name, _ = self._parse_name(rdata, 0)
                sensor_id = self._extract_sensor_id(name, target_name)
                if sensor_id:
                    self.sensors[sensor_id]['ptr_target'] = target_name
                    self.sensors[sensor_id]['interface'] = interface_name
            
            elif rtype == 16:  # TXT record
                txt_data = self._parse_txt_record(rdata)
                sensor_id = self._extract_sensor_id(name)
                if sensor_id:
                    self.sensors[sensor_id]['txt'] = txt_data
                    if 'sn' in txt_data:
                        self.sensors[sensor_id]['serial'] = txt_data['sn']
                    self.sensors[sensor_id]['interface'] = interface_name
        
        # A records for sensor hostnames
        elif rtype == 1 and len(rdata) == 4:  # A record
            ip = socket.inet_ntoa(rdata)
            
            if name.startswith('os-') and name.endswith('.local'):
                # Extract serial from hostname like "os-992040000160.local"
                serial = name[3:].replace('.local', '')
                sensor_id = f"ouster-{serial}"
                self.sensors[sensor_id]['ip'] = ip
                self.sensors[sensor_id]['hostname'] = name
                self.sensors[sensor_id]['serial'] = serial
                self.sensors[sensor_id]['interface'] = interface_name
            
            # Also check if this IP belongs to any sensor we've seen
            for sensor_id in self.sensors:
                if 'hostname' in self.sensors[sensor_id]:
                    if self.sensors[sensor_id]['hostname'] == name:
                        self.sensors[sensor_id]['ip'] = ip
                        if interface_name:
                            self.sensors[sensor_id]['interface'] = interface_name
    
    def _extract_sensor_id(self, name, target_name=None):
        """Extract sensor identifier from DNS name"""
        if 'ouster' in name.lower():
            # Try to extract from various name formats
            parts = name.split('.')
            for part in parts:
                if 'ouster' in part.lower() and len(part) > 6:
                    return part
        
        if target_name and 'os-' in target_name:
            serial = target_name.split('.')[0][3:]  # Remove 'os-' prefix
            return f"ouster-{serial}"
        
        return None
    
    def _parse_txt_record(self, data):
        """Parse TXT record data"""
        txt_data = {}
        offset = 0
        
        while offset < len(data):
            if offset >= len(data):
                break
            
            length = data[offset]
            if length == 0:
                break
            
            offset += 1
            if offset + length > len(data):
                break
            
            txt_string = data[offset:offset + length].decode('utf-8', errors='ignore')
            offset += length
            
            if '=' in txt_string:
                key, value = txt_string.split('=', 1)
                txt_data[key] = value
        
        return txt_data

class NetworkInterfaceManager:
    """Manages network interfaces for mDNS discovery"""
    
    @staticmethod
    def get_interfaces():
        """Get all network interfaces with their IPs"""
        interfaces = []
        
        try:
            # Try using socket to get interface info
            import subprocess
            
            # Method 1: Use 'ip' command if available
            try:
                result = subprocess.run(['ip', 'addr', 'show'], capture_output=True, text=True, timeout=100)
                if result.returncode == 0:
                    return NetworkInterfaceManager._parse_ip_addr_output(result.stdout)
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
            
            # Method 2: Use 'ifconfig' command
            try:
                result = subprocess.run(['ifconfig', '-a'], capture_output=True, text=True, timeout=100)
                if result.returncode == 0:
                    return NetworkInterfaceManager._parse_ifconfig_output(result.stdout)
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass
            
            # Method 3: Parse /proc/net/dev and try to get IPs
            try:
                with open('/proc/net/dev', 'r') as f:
                    for line in f:
                        if ':' in line and not line.strip().startswith('lo:'):
                            iface = line.split(':')[0].strip()
                            if iface not in ['lo']:
                                # Try to get IP for this interface
                                ip = NetworkInterfaceManager._get_interface_ip_proc(iface)
                                if ip:
                                    interfaces.append((iface, ip))
            except:
                pass
            
            # Method 4: Fallback - try common interface names
            if not interfaces:
                common_interfaces = ['eth0', 'eth1', 'wlan0', 'enp0s3', 'ens33']
                for iface in common_interfaces:
                    ip = NetworkInterfaceManager._get_interface_ip_socket(iface)
                    if ip:
                        interfaces.append((iface, ip))
        
        except Exception:
            pass
        
        return interfaces
    
    @staticmethod
    def _parse_ip_addr_output(output):
        """Parse 'ip addr show' output"""
        interfaces = []
        current_iface = None
        
        for line in output.split('\n'):
            line = line.strip()
            if line.startswith(('1:', '2:', '3:', '4:', '5:', '6:', '7:', '8:', '9:')):
                # Interface line
                parts = line.split()
                if len(parts) >= 2:
                    current_iface = parts[1].rstrip(':').split('@')[0]
            elif line.startswith('inet ') and current_iface and current_iface != 'lo':
                # IP address line
                parts = line.split()
                if len(parts) >= 2:
                    ip = parts[1].split('/')[0]
                    if not ip.startswith('127.'):
                        interfaces.append((current_iface, ip))
        
        return interfaces
    
    @staticmethod
    def _parse_ifconfig_output(output):
        """Parse 'ifconfig -a' output"""
        interfaces = []
        current_iface = None
        
        for line in output.split('\n'):
            if line and not line.startswith(' ') and not line.startswith('\t'):
                # Interface name line
                current_iface = line.split()[0].rstrip(':')
            elif 'inet ' in line and current_iface and current_iface != 'lo':
                # IP address line
                parts = line.split()
                for i, part in enumerate(parts):
                    if part == 'inet' and i + 1 < len(parts):
                        ip = parts[i + 1].replace('addr:', '')
                        if not ip.startswith('127.'):
                            interfaces.append((current_iface, ip))
                        break
        
        return interfaces
    
    @staticmethod
    def _get_interface_ip_proc(interface):
        """Get interface IP using /proc filesystem"""
        try:
            with open(f'/proc/net/fib_trie', 'r') as f:
                content = f.read()
                # This is a simplified approach
                # In practice, parsing fib_trie is complex
                return None
        except:
            return None
    
    @staticmethod
    def _get_interface_ip_socket(interface):
        """Get interface IP using socket"""
        try:
            import fcntl
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            packed_iface = struct.pack('256s', interface.encode('utf-8'))
            packed_addr = fcntl.ioctl(sock.fileno(), 0x8915, packed_iface)  # SIOCGIFADDR
            addr = socket.inet_ntoa(packed_addr[20:24])
            sock.close()
            return addr
        except:
            return None

class OusterMDNSDiscovery:
    """Main discovery class that coordinates mDNS query and response handling"""
    
    def __init__(self):
        self.query = MDNSQuery()
        self.parser = MDNSParser()
        self.running = False
        self.sockets = []
    
    def discover(self, timeout=100, target_serial=None):
        """Discover Ouster sensors via mDNS"""
        
        print(f"Discovering Ouster sensors via mDNS (timeout: {timeout}s)...")
        
        # Get network interfaces
        interfaces = NetworkInterfaceManager.get_interfaces()
        
        if not interfaces:
            print("No network interfaces found")
            return []
        
        print(f"Scanning {len(interfaces)} interfaces:")
        for iface, ip in interfaces:
            print(f"  {iface}: {ip}")
        
        self.running = True
        
        # Start listeners and send queries
        threads = []
        
        for interface_name, interface_ip in interfaces:
            # Start listener thread
            thread = threading.Thread(
                target=self._listen_on_interface, 
                args=(interface_ip, interface_name),
                daemon=True
            )
            thread.start()
            threads.append(thread)
            
            # Send query on this interface
            self._send_query_on_interface(interface_ip, interface_name)
        
        # Wait for responses
        print(f"Listening for responses...")
        time.sleep(timeout)
        
        self.running = False
        
        # Close all sockets
        for sock in self.sockets:
            try:
                sock.close()
            except:
                pass
        
        # Wait for threads to finish
        for thread in threads:
            thread.join(timeout=100)
        
        # Process results
        sensors = self._extract_sensor_info(target_serial)
        
        return sensors
    
    def _listen_on_interface(self, interface_ip, interface_name):
        """Listen for mDNS responses on a specific interface"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to the interface
            sock.bind((interface_ip, 0))
            
            # Join multicast group
            mreq = struct.pack('4s4s', 
                             socket.inet_aton(self.query.MDNS_ADDR), 
                             socket.inet_aton(interface_ip))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
            sock.settimeout(1.0)
            self.sockets.append(sock)
            
            while self.running:
                try:
                    # Use select for better timeout handling
                    ready = select.select([sock], [], [], 0.5)
                    if ready[0]:
                        data, addr = sock.recvfrom(4096)
                        self.parser.parse_response(data, addr[0], interface_name)
                except socket.timeout:
                    continue
                except Exception:
                    break
        
        except Exception as e:
            print(f"Error listening on {interface_name} ({interface_ip}): {e}")
    
    def _send_query_on_interface(self, interface_ip, interface_name):
        """Send mDNS query on a specific interface"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Bind to the interface
            sock.bind((interface_ip, 0))
            
            # Send query
            query_packet = self.query.create_query_packet("_roger._tcp.local")
            sock.sendto(query_packet, (self.query.MDNS_ADDR, self.query.MDNS_PORT))
            
            # Also send a general services query
            services_packet = self.query.create_query_packet("_services._dns-sd._udp.local")
            sock.sendto(services_packet, (self.query.MDNS_ADDR, self.query.MDNS_PORT))
            
            sock.close()
            
        except Exception as e:
            print(f"Error sending query on {interface_name} ({interface_ip}): {e}")
    
    def _extract_sensor_info(self, target_serial=None):
        """Extract and format sensor information"""
        sensors = []
        
        for sensor_id, data in self.parser.sensors.items():
            if 'ip' in data:
                sensor_info = {
                    'name': sensor_id,
                    'ip': data['ip'],
                    'interface': data.get('interface', 'unknown'),
                    'hostname': data.get('hostname', ''),
                    'serial': data.get('serial', ''),
                    'txt_records': data.get('txt', {})
                }
                
                # Extract additional info from TXT records
                txt = data.get('txt', {})
                sensor_info['firmware'] = txt.get('fw', '')
                sensor_info['part_number'] = txt.get('pn', '')
                
                # Filter by serial if specified
                if target_serial is None or sensor_info['serial'] == target_serial:
                    sensors.append(sensor_info)
        
        return sensors

def setup_network_route(sensor_info):
    """Set up network routing to reach the sensor"""
    interface = sensor_info['interface']
    ip = sensor_info['ip']
    
    if interface == 'unknown':
        print("⚠ Could not determine interface")
        return False
    
    # Determine network
    if ip.startswith('169.254.'):
        network = '169.254.0.0/16'
    else:
        ip_parts = ip.split('.')
        network = f"{ip_parts[0]}.{ip_parts[1]}.{ip_parts[2]}.0/24"
    
    print(f"Setting up route: {network} via {interface}")
    
    try:
        # Try different route command formats
        route_commands = [
            ['ip', 'route', 'add', network, 'dev', interface],
            ['route', 'add', '-net', network, 'dev', interface],
            ['route', 'add', '-net', network.split('/')[0], 'netmask', '255.255.255.0', 'dev', interface]
        ]
        
        for cmd in route_commands:
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=100)
                if result.returncode == 0:
                    print(f"✓ Route added: {' '.join(cmd)}")
                    return True
                elif 'exists' in result.stderr.lower():
                    print("Route already exists")
                    return True
            except (subprocess.TimeoutExpired, FileNotFoundError):
                continue
        
        print("✗ Failed to add route with all methods")
        return False
    
    except Exception as e:
        print(f"✗ Route setup failed: {e}")
        return False

def test_sensor_connectivity(sensor_info):
    """Test connectivity to the sensor"""
    ip = sensor_info['ip']
    
    print(f"Testing connectivity to {ip}...")
    
    # Test TCP connection
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        result = sock.connect_ex((ip, 80))
        sock.close()
        
        if result == 0:
            print("✓ HTTP port accessible")
            
            # Test API
            try:
                import urllib.request
                response = urllib.request.urlopen(f"http://{ip}/api/v1/sensor/info", timeout=100)
                data = response.read().decode('utf-8')
                print("✓ Ouster API accessible")
                print(f"API response preview: {data[:100]}...")
                return True
            except:
                print("⚠ HTTP accessible but API not responding")
                return True
        else:
            print("✗ HTTP port not accessible")
            return False
    
    except Exception as e:
        print(f"✗ Connection test failed: {e}")
        return False

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Standalone mDNS Ouster Discovery Tool')
    parser.add_argument('serial', nargs='?', help='Target serial number (or "any" for any sensor)')
    parser.add_argument('--timeout', '-t', type=int, default=8, help='Discovery timeout in seconds')
    parser.add_argument('--no-route', action='store_true', help='Skip automatic route setup')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if not args.serial:
        print("Usage: python3 mdns_discovery.py <serial_number|any> [--timeout 8] [--no-route]")
        print("Example: python3 mdns_discovery.py 992040000160")
        print("         python3 mdns_discovery.py any")
        sys.exit(1)
    
    target_serial = None if args.serial == 'any' else args.serial
    
    # Create discovery instance
    discovery = OusterMDNSDiscovery()
    
    # Discover sensors
    sensors = discovery.discover(timeout=args.timeout, target_serial=target_serial)
    
    if not sensors:
        print("No Ouster sensors found")
        sys.exit(1)
    
    print(f"\nFound {len(sensors)} sensor(s):")
    
    for i, sensor in enumerate(sensors):
        print(f"\nSensor {i+1}:")
        print(f"  IP: {sensor['ip']}")
        print(f"  Interface: {sensor['interface']}")
        print(f"  Serial: {sensor['serial']}")
        print(f"  Hostname: {sensor['hostname']}")
        print(f"  Firmware: {sensor['firmware']}")
        print(f"  Part Number: {sensor['part_number']}")
        
        if args.verbose and sensor['txt_records']:
            print(f"  TXT Records: {sensor['txt_records']}")
        
        # Set up routing and test connectivity
        if not args.no_route:
            print("\nSetting up network access...")
            if setup_network_route(sensor):
                test_sensor_connectivity(sensor)
        
        print("-" * 50)
    
    # Output in avahi-browse compatible format
    print(f"\nAvahi-browse compatible output:")
    for sensor in sensors:
        print(f"+ {sensor['interface']} IPv4 Ouster Sensor {sensor['serial']} _roger._tcp local")
        print(f"= {sensor['interface']} IPv4 Ouster Sensor {sensor['serial']} _roger._tcp local")
        print(f"   hostname = [{sensor['hostname']}]")
        print(f"   address = [{sensor['ip']}]")
        print(f"   port = [7501]")
        
        txt_items = []
        if sensor['firmware']:
            txt_items.append(f'"fw={sensor["firmware"]}"')
        if sensor['serial']:
            txt_items.append(f'"sn={sensor["serial"]}"')
        if sensor['part_number']:
            txt_items.append(f'"pn={sensor["part_number"]}"')
        
        if txt_items:
            print(f"   txt = [{' '.join(txt_items)}]")

if __name__ == '__main__':
    main()