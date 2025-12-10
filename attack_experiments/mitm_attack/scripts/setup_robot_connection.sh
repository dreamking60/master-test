#!/bin/bash
# Configure robot machine to use MITM router as gateway

echo "=========================================="
echo "Robot Machine Configuration (Target)"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"

# Check if router config exists
if [ ! -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    echo "❌ MITM router not configured yet"
    echo "   Please run setup_mitm_router.sh on attacker machine first"
    exit 1
fi

# Load router config
source "$CONFIG_DIR/mitm_router_config.txt"

echo "This script configures the robot to use MITM router as gateway."
echo ""
echo "Router IP: $ROUTER_IP"
echo ""

read -p "Enter robot machine's network interface (e.g., eth0, ens33): " ROBOT_IFACE

if [ -z "$ROUTER_IP" ]; then
    read -p "Enter MITM router IP address: " ROUTER_IP
fi

# Get current gateway
CURRENT_GATEWAY=$(ip route | grep default | awk '{print $3}')

echo ""
echo "Current configuration:"
echo "  Interface: $ROUTER_IFACE"
echo "  Current Gateway: ${CURRENT_GATEWAY:-none}"
echo "  New Gateway: $ROUTER_IP"
echo ""

read -p "Change gateway to MITM router? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Configuration cancelled"
    exit 0
fi

echo ""
echo "Configuring gateway..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "⚠️  Root privileges required for network configuration"
    echo "   Run with: sudo $0"
    echo ""
    echo "Manual configuration:"
    echo "  sudo ip route del default"
    echo "  sudo ip route add default via $ROUTER_IP dev $ROUTER_IFACE"
    exit 1
fi

# Remove old default route
ip route del default 2>/dev/null

# Add new default route through MITM router
ip route add default via "$ROUTER_IP" dev "$ROUTER_IFACE"

echo "✅ Gateway configured: $ROUTER_IP"
echo ""

# Save configuration
cat > "$CONFIG_DIR/robot_config.txt" << EOF
# Robot Machine Configuration
ROBOT_IP=$(hostname -I | awk '{print $1}')
ROBOT_INTERFACE=$ROUTER_IFACE
ROBOT_GATEWAY=$ROUTER_IP
MITM_ROUTER_IP=$ROUTER_IP
CONFIG_TIME=$(date '+%Y-%m-%d %H:%M:%S')
EOF

echo "✅ Configuration saved"
echo ""
echo "Robot is now routing through MITM router"
echo "All traffic will pass through: $ROUTER_IP"
echo ""

