#!/bin/bash

# Minimal Ouster discovery using only basic tools
# Works with: bash, curl, ifconfig, route, cat, grep, awk

test_tcp_connection() {
    local host="$1"
    local port="${2:-80}"
    local timeout="${3:-3}"
    
    # Test TCP connection using bash built-in
    if timeout "$timeout" bash -c "</dev/tcp/$host/$port" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

get_interfaces() {
    # Try multiple methods to get interface list
    if command -v ifconfig >/dev/null 2>&1; then
        # Use ifconfig
        ifconfig -a | grep -E "^[a-zA-Z0-9]" | awk '{print $1}' | grep -v "lo"
    elif [ -d /sys/class/net ]; then
        # Use /sys filesystem
        ls /sys/class/net | grep -v lo
    elif [ -f /proc/net/dev ]; then
        # Use /proc filesystem
        cat /proc/net/dev | grep -E "^ *[a-zA-Z0-9]" | awk -F: '{print $1}' | tr -d ' ' | grep -v lo
    else
        echo "Cannot determine network interfaces"
        return 1
    fi
}

get_interface_ip() {
    local interface="$1"
    
    if command -v ifconfig >/dev/null 2>&1; then
        # Use ifconfig
        ifconfig "$interface" 2>/dev/null | grep 'inet ' | awk '{print $2}' | sed 's/addr://'
    elif [ -f "/sys/class/net/$interface/operstate" ]; then
        # Try to read from /proc or alternative methods
        # This is trickier without ip command, but we can check if interface exists
        if [ -f "/proc/net/fib_trie" ]; then
            # Look for interface in routing table
            grep -A 10 "$interface" /proc/net/fib_trie 2>/dev/null | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1
        fi
    fi
}

test_ouster_api() {
    local ip="$1"
    
    echo "Testing Ouster API at $ip..."
    
    # Test HTTP connection first
    if ! test_tcp_connection "$ip" 80 3; then
        echo "  ✗ HTTP port not accessible"
        return 1
    fi
    
    echo "  ✓ HTTP port accessible"
    
    # Test Ouster API endpoints
    local api_paths=("/api/v1/sensor/info" "/sensor/info" "/api/sensor/info")
    
    for path in "${api_paths[@]}"; do
        local response=$(timeout 5 curl -s --connect-timeout 2 "http://$ip$path" 2>/dev/null)
        
        if [ -n "$response" ] && [[ "$response" =~ ("prod_sn"|"serial"|"firmware"|"lidar") ]]; then
            echo "  ✓ Ouster API responds at $path"
            
            # Extract serial number
            local serial=""
            serial=$(echo "$response" | grep -o '"prod_sn"[^"]*"[^"]*"' | cut -d'"' -f4)
            if [ -z "$serial" ]; then
                serial=$(echo "$response" | grep -o '"serial_number"[^"]*"[^"]*"' | cut -d'"' -f4)
            fi
            if [ -z "$serial" ]; then
                serial=$(echo "$response" | grep -o '"sn"[^"]*"[^"]*"' | cut -d'"' -f4)
            fi
            
            echo "  Serial: ${serial:-unknown}"
            echo "  Response preview: $(echo "$response" | head -c 80)..."
            
            return 0
        fi
    done
    
    echo "  ⚠ HTTP responds but no valid Ouster API found"
    return 1
}

test_common_ips() {
    echo "=== Testing Common Ouster IP Addresses ==="
    
    local common_ips=(
        "169.254.168.90"
        "10.10.20.90" 
        "192.168.1.90"
        "169.254.168.1"
        "10.10.20.1"
        "192.168.1.1"
        "10.10.20.100"
    )
    
    for ip in "${common_ips[@]}"; do
        if test_ouster_api "$ip"; then
            echo "✓ Found Ouster sensor at $ip"
            return 0
        fi
    done
    
    echo "No sensors found at common IP addresses"
    return 1
}

scan_interface_range() {
    local interface="$1"
    local ip_base="$2"  # e.g., "169.254.168" or "10.10.20"
    
    echo "Scanning ${ip_base}.0/24 on interface $interface..."
    
    # Test most common Ouster IPs in this range first
    local priority_ips=(90 1 10 100 50 2)
    
    for suffix in "${priority_ips[@]}"; do
        local test_ip="${ip_base}.${suffix}"
        echo "  Testing ${test_ip}..."
        
        if test_ouster_api "$test_ip"; then
            echo "✓ Found Ouster sensor at $test_ip on $interface"
            return 0
        fi
    done
    
    return 1
}

scan_all_interfaces() {
    echo "=== Scanning All Network Interfaces ==="
    
    local interfaces=$(get_interfaces)
    
    if [ -z "$interfaces" ]; then
        echo "Cannot detect network interfaces"
        return 1
    fi
    
    echo "Found interfaces: $interfaces"
    echo
    
    for interface in $interfaces; do
        echo "Checking interface: $interface"
        
        # Check if interface appears to be USB ethernet (common pattern)
        if [[ "$interface" =~ ^enx[0-9a-f]{12}$ ]]; then
            echo "  This appears to be a USB ethernet interface"
        fi
        
        # Get interface IP if possible
        local interface_ip=$(get_interface_ip "$interface")
        
        if [ -n "$interface_ip" ]; then
            echo "  Interface IP: $interface_ip"
            
            # Determine what network ranges to scan based on interface IP
            if [[ $interface_ip =~ ^169\.254\. ]]; then
                scan_interface_range "$interface" "169.254.168"
            elif [[ $interface_ip =~ ^10\.10\.20\. ]]; then
                scan_interface_range "$interface" "10.10.20"
            elif [[ $interface_ip =~ ^192\.168\.1\. ]]; then
                scan_interface_range "$interface" "192.168.1"
            elif [[ $interface_ip =~ ^([0-9]+\.[0-9]+\.[0-9]+)\.[0-9]+$ ]]; then
                scan_interface_range "$interface" "${BASH_REMATCH[1]}"
            fi
        else
            echo "  No IP address detected"
            
            # If it's a USB ethernet interface, try common Ouster ranges anyway
            if [[ "$interface" =~ ^enx[0-9a-f]{12}$ ]]; then
                echo "  Trying common ranges on USB interface..."
                scan_interface_range "$interface" "169.254.168" || \
                scan_interface_range "$interface" "10.10.20"
            fi
        fi
        
        echo
    done
    
    return 1
}

check_tools() {
    echo "=== Checking Available Tools ==="
    
    # Check for essential tools
    local required="curl bash"
    local missing=""
    
    for tool in $required; do
        if command -v "$tool" >/dev/null 2>&1; then
            echo "✓ $tool available"
        else
            missing="$missing $tool"
        fi
    done
    
    # Check for network tools
    echo "Network tools:"
    for tool in ifconfig ip route netstat; do
        if command -v "$tool" >/dev/null 2>&1; then
            echo "✓ $tool available"
        else
            echo "✗ $tool not available"
        fi
    done
    
    if [ -n "$missing" ]; then
        echo "✗ Missing essential tools:$missing"
        echo "Cannot proceed without these tools"
        exit 1
    fi
    
    echo
}

show_network_info() {
    echo "=== Network Information ==="
    
    echo "Network interfaces:"
    local interfaces=$(get_interfaces)
    for interface in $interfaces; do
        echo "  $interface"
        local ip=$(get_interface_ip "$interface")
        if [ -n "$ip" ]; then
            echo "    IP: $ip"
        else
            echo "    No IP"
        fi
    done
    echo
    
    # Show routing table if available
    if command -v route >/dev/null 2>&1; then
        echo "Routing table:"
        route -n 2>/dev/null | head -10
    elif [ -f /proc/net/route ]; then
        echo "Route information:"
        cat /proc/net/route | head -5
    fi
    echo
}

manual_test() {
    local ip="$1"
    
    echo "=== Manual IP Test ==="
    echo "Testing specific IP: $ip"
    
    if test_ouster_api "$ip"; then
        echo "✓ Found Ouster sensor at $ip"
        return 0
    else
        echo "✗ No Ouster sensor found at $ip"
        return 1
    fi
}

main() {
    echo "Minimal Ouster Discovery Tool"
    echo "============================="
    echo "Uses only: bash, curl, ifconfig/proc, basic utilities"
    echo
    
    check_tools
    show_network_info
    
    # If IP provided as argument, test it directly
    if [ $# -gt 0 ] && [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        manual_test "$1"
        exit $?
    fi
    
    echo "Starting automatic discovery..."
    echo
    
    # Try common IPs first (fastest)
    if test_common_ips; then
        echo "✓ Discovery successful!"
        exit 0
    fi
    
    echo
    # Scan all interfaces
    if scan_all_interfaces; then
        echo "✓ Discovery successful!"
        exit 0
    fi
    
    echo
    echo "=== Discovery Failed ==="
    echo "No Ouster sensors found automatically."
    echo
    echo "Manual steps to try:"
    echo "1. List your network interfaces:"
    if command -v ifconfig >/dev/null 2>&1; then
        echo "   ifconfig -a"
    else
        echo "   ls /sys/class/net/"
    fi
    echo
    echo "2. Configure your USB ethernet interface (replace enxXXX):"
    if command -v ifconfig >/dev/null 2>&1; then
        echo "   sudo ifconfig enxXXXXXXXXXXXX up"
        echo "   sudo ifconfig enxXXXXXXXXXXXX 169.254.168.1"
    else
        echo "   echo 1 | sudo tee /sys/class/net/enxXXXXXXXXXXXX/carrier"
    fi
    echo
    echo "3. Test the known IP directly:"
    echo "   $0 10.10.20.90"
    echo "   curl http://10.10.20.90/api/v1/sensor/info"
    echo
    echo "4. Check if interface shows up:"
    echo "   ls /sys/class/net/ | grep enx"
    
    exit 1
}

main "$@"