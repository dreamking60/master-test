#!/bin/bash
# Configure controller machine to use MITM router as gateway

echo "=========================================="
echo "Controller Machine Configuration"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"

# Try to load router config if exists
ROUTER_IP=""
if [ -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    source "$CONFIG_DIR/mitm_router_config.txt"
    echo "Found router configuration"
    echo "Router IP: $ROUTER_IP"
    echo ""
else
    echo "Router configuration not found locally"
    echo "This is normal if running on a different machine"
    echo ""
fi

echo "This script configures the controller to use MITM router as gateway."
echo ""

if [ -z "$ROUTER_IP" ]; then
    read -p "Enter MITM router IP address: " ROUTER_IP
    if [ -z "$ROUTER_IP" ]; then
        echo "❌ Router IP is required"
        exit 1
    fi
fi

read -p "Enter controller machine's network interface (e.g., eth0, ens33): " CONTROLLER_IFACE

# Get current gateway
CURRENT_GATEWAY=$(ip route | grep default | awk '{print $3}')

echo ""
echo "Current configuration:"
echo "  Interface: $CONTROLLER_IFACE"
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
    echo "  sudo ip route add default via $ROUTER_IP dev $CONTROLLER_IFACE"
    exit 1
fi

# Remove old default route
ip route del default 2>/dev/null

# Add new default route through MITM router
ip route add default via "$ROUTER_IP" dev "$CONTROLLER_IFACE"

echo "✅ Gateway configured: $ROUTER_IP"
echo ""

# Save configuration
mkdir -p "$CONFIG_DIR"
cat > "$CONFIG_DIR/controller_config.txt" << EOF
# Controller Machine Configuration
CONTROLLER_IP=$(hostname -I | awk '{print $1}')
CONTROLLER_INTERFACE=$CONTROLLER_IFACE
CONTROLLER_GATEWAY=$ROUTER_IP
MITM_ROUTER_IP=$ROUTER_IP
CONFIG_TIME=$(date '+%Y-%m-%d %H:%M:%S')
EOF

echo "✅ Configuration saved"
echo ""
echo "Controller is now routing through MITM router"
echo "All traffic will pass through: $ROUTER_IP"
echo ""

