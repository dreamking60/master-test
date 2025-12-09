#!/bin/bash
# Test network interception attack
# Attempts to intercept and analyze encrypted DDS traffic

echo "=========================================="
echo "Network Interception Attack Test"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

echo "This test attempts to intercept encrypted DDS traffic."
echo ""
echo "Expected result: Traffic is encrypted, cannot be decrypted"
echo ""

# Check if required tools are available
if ! command -v tcpdump &> /dev/null && ! command -v wireshark &> /dev/null; then
    echo "⚠️  Network analysis tools not found"
    echo "   Install with: sudo apt install tcpdump wireshark"
    echo ""
    echo "This test requires network analysis tools to capture packets."
    echo "Even with tools, encrypted traffic should not be readable."
    exit 0
fi

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Test cancelled"
    exit 0
fi

echo ""
echo "Step 1: Capturing DDS traffic..."
echo ""
echo "⚠️  Note: This requires root privileges"
echo "   SROS2 traffic is encrypted with TLS/DTLS"
echo "   Even if captured, packets should be unreadable"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "⚠️  This test requires root privileges for packet capture"
    echo "   Run with: sudo $0"
    echo ""
    echo "However, even with packet capture, encrypted traffic"
    echo "should not be readable without the private keys."
    exit 0
fi

echo "Capturing DDS traffic on ports 7400-7500..."
echo "Press Ctrl+C to stop capture"
echo ""

# Capture DDS traffic (encrypted)
CAPTURE_FILE="/tmp/dds_capture.pcap"
timeout 10 tcpdump -i any -w "$CAPTURE_FILE" \
    'udp portrange 7400-7500' 2>/dev/null &

CAPTURE_PID=$!

# Wait a bit for capture
sleep 5

# Try to generate some traffic
if ros2 topic list 2>/dev/null | grep -q cmd_vel; then
    echo "Generating test traffic..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
        > /dev/null 2>&1
fi

sleep 3
kill $CAPTURE_PID 2>/dev/null
wait $CAPTURE_PID 2>/dev/null

echo ""
echo "Step 2: Analyzing captured traffic..."
echo ""

if [ -f "$CAPTURE_FILE" ]; then
    PACKET_COUNT=$(tcpdump -r "$CAPTURE_FILE" 2>/dev/null | wc -l)
    echo "Captured $PACKET_COUNT packets"
    echo ""
    
    echo "Attempting to read packet contents..."
    echo ""
    
    # Try to read packet data (should be encrypted/unreadable)
    if tcpdump -r "$CAPTURE_FILE" -A 2>/dev/null | grep -q "cmd_vel\|Twist\|linear\|angular"; then
        echo "⚠️  WARNING: Plaintext data found in captured packets!"
        echo "   This suggests encryption may not be working"
    else
        echo "✅ SUCCESS (for security): Packet contents are encrypted/unreadable"
        echo "   Cannot extract meaningful data from captured packets"
        echo "   This is the CORRECT behavior"
    fi
    
    echo ""
    echo "Packet capture saved to: $CAPTURE_FILE"
    echo "Analyze with: tcpdump -r $CAPTURE_FILE -A"
    echo "Or: wireshark $CAPTURE_FILE"
else
    echo "⚠️  No packets captured"
    echo "   This may indicate:"
    echo "   - No traffic on DDS ports"
    echo "   - Firewall blocking"
    echo "   - SROS2 not enabled"
fi

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="
echo ""
echo "Conclusion:"
echo "  - If packets are encrypted: SROS2 encryption is working"
echo "  - If plaintext found: Encryption may not be enabled"
echo ""

