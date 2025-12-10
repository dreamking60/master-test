#!/bin/bash
# Simple script to configure gateway to MITM router
# Can be run on any machine (robot or controller)

echo "=========================================="
echo "Configure Gateway to MITM Router"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script requires root privileges"
    echo "   Run with: sudo $0"
    exit 1
fi

# Get current gateway
CURRENT_GATEWAY=$(ip route | grep default | awk '{print $3}')
CURRENT_INTERFACE=$(ip route | grep default | awk '{print $5}')

echo "Current network configuration:"
echo "  Current Gateway: ${CURRENT_GATEWAY:-none}"
echo "  Current Interface: ${CURRENT_INTERFACE:-unknown}"
echo ""

# Get network interfaces
echo "Available network interfaces:"
ip link show | grep -E "^[0-9]+:" | grep -v lo | awk -F': ' '{print "  " $2}'
echo ""

# Get router IP
read -p "Enter MITM router IP address: " ROUTER_IP

if [ -z "$ROUTER_IP" ]; then
    echo "❌ Router IP is required"
    exit 1
fi

# Get interface
read -p "Enter network interface name (e.g., eth0, ens33) [default: $CURRENT_INTERFACE]: " INTERFACE
INTERFACE=${INTERFACE:-$CURRENT_INTERFACE}

if [ -z "$INTERFACE" ]; then
    echo "❌ Interface name is required"
    exit 1
fi

# Verify interface exists
if ! ip link show "$INTERFACE" &>/dev/null; then
    echo "❌ Interface $INTERFACE does not exist"
    exit 1
fi

echo ""
echo "Configuration:"
echo "  Interface: $INTERFACE"
echo "  New Gateway: $ROUTER_IP"
echo "  Old Gateway: ${CURRENT_GATEWAY:-none}"
echo ""

read -p "Change gateway to $ROUTER_IP? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Configuration cancelled"
    exit 0
fi

echo ""
echo "Configuring gateway..."

# Remove old default route
ip route del default 2>/dev/null

# Add new default route through MITM router
ip route add default via "$ROUTER_IP" dev "$INTERFACE"

if [ $? -eq 0 ]; then
    echo "✅ Gateway configured successfully!"
    echo ""
    echo "New gateway: $ROUTER_IP"
    echo "All traffic will now route through MITM router"
    echo ""
    
    # Test connectivity
    echo "Testing connectivity to router..."
    if ping -c 2 "$ROUTER_IP" &>/dev/null; then
        echo "✅ Router is reachable"
    else
        echo "⚠️  Cannot ping router (may be normal if firewall blocks ICMP)"
    fi
else
    echo "❌ Failed to configure gateway"
    exit 1
fi

echo ""
echo "To restore original gateway:"
echo "  sudo ip route del default"
echo "  sudo ip route add default via <original_gateway_ip> dev $INTERFACE"
echo ""

