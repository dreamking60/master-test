#!/bin/bash
# Setup attacker machine as MITM router
# This enables IP forwarding and configures the machine to act as a router

echo "=========================================="
echo "MITM Router Setup (Attacker Machine)"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script requires root privileges"
    echo "   Run with: sudo $0"
    exit 1
fi

echo "This script configures this machine as a MITM router."
echo "All traffic between robot and controller will pass through this machine."
echo ""
echo "⚠️  WARNING: This modifies network configuration!"
echo ""

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Setup cancelled"
    exit 0
fi

echo ""
echo "Step 1: Enabling IP forwarding..."
echo ""

# Enable IP forwarding
echo 1 > /proc/sys/net/ipv4/ip_forward
sysctl -w net.ipv4.ip_forward=1

# Make it persistent
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf; then
    echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf
fi

echo "✅ IP forwarding enabled"
echo ""

echo "Step 2: Getting network interface information..."
echo ""

# Get network interfaces
INTERFACES=$(ip link show | grep -E "^[0-9]+:" | grep -v lo | awk -F': ' '{print $2}')
echo "Available interfaces:"
echo "$INTERFACES" | nl
echo ""

read -p "Enter interface name for MITM router (e.g., eth0, ens33): " ROUTER_IFACE

if [ -z "$ROUTER_IFACE" ] || ! echo "$INTERFACES" | grep -q "^$ROUTER_IFACE$"; then
    echo "❌ Invalid interface name"
    exit 1
fi

# Get current IP
ROUTER_IP=$(ip addr show "$ROUTER_IFACE" | grep "inet " | awk '{print $2}' | cut -d'/' -f1)

if [ -z "$ROUTER_IP" ]; then
    echo "❌ Interface $ROUTER_IFACE has no IP address"
    echo "   Please configure IP address first"
    exit 1
fi

echo "✅ Router IP: $ROUTER_IP"
echo ""

echo "Step 3: Configuring iptables for traffic forwarding..."
echo ""

# Configure NAT for forwarding
iptables -t nat -A POSTROUTING -o "$ROUTER_IFACE" -j MASQUERADE
iptables -A FORWARD -i "$ROUTER_IFACE" -o "$ROUTER_IFACE" -j ACCEPT
iptables -A FORWARD -i "$ROUTER_IFACE" -o "$ROUTER_IFACE" -m state --state RELATED,ESTABLISHED -j ACCEPT

echo "✅ iptables configured"
echo ""

echo "Step 4: Saving configuration..."
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"

mkdir -p "$CONFIG_DIR"

cat > "$CONFIG_DIR/mitm_router_config.txt" << EOF
# MITM Router Configuration
ROUTER_IP=$ROUTER_IP
ROUTER_INTERFACE=$ROUTER_IFACE
ROUTER_NETWORK=$(echo "$ROUTER_IP" | cut -d'.' -f1-3).0/24
SETUP_TIME=$(date '+%Y-%m-%d %H:%M:%S')
EOF

echo "✅ Configuration saved to: $CONFIG_DIR/mitm_router_config.txt"
echo ""

echo "=========================================="
echo "MITM Router Setup Complete!"
echo "=========================================="
echo ""
echo "Router Information:"
echo "  IP Address: $ROUTER_IP"
echo "  Interface: $ROUTER_IFACE"
echo "  Network: $(echo "$ROUTER_IP" | cut -d'.' -f1-3).0/24"
echo ""
echo "Next steps:"
echo "  1. Configure robot machine to use $ROUTER_IP as gateway"
echo "  2. Configure controller machine to use $ROUTER_IP as gateway"
echo "  3. Run: ./intercept_and_modify.sh to start intercepting"
echo ""
echo "To disable MITM router:"
echo "  echo 0 > /proc/sys/net/ipv4/ip_forward"
echo "  iptables -t nat -F"
echo ""

