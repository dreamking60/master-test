#!/bin/bash
# Test CA forgery attack
# Attempts to use forged CA to create valid certificates

echo "=========================================="
echo "CA Forgery Attack Test"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

echo "This test demonstrates CA forgery attack."
echo "It creates a forged CA and attempts to use it."
echo ""
echo "Expected result: Should FAIL (CA verification should reject)"
echo ""

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Test cancelled"
    exit 0
fi

echo ""
echo "Step 1: Creating forged CA..."
echo ""

FORGED_CA_DIR="$MITM_DIR/forged_ca"
mkdir -p "$FORGED_CA_DIR"

# Create forged CA with same name as legitimate CA (but different key)
openssl genrsa -out "$FORGED_CA_DIR/ca_key.pem" 2048
openssl req -new -x509 -days 365 -key "$FORGED_CA_DIR/ca_key.pem" \
    -out "$FORGED_CA_DIR/ca_cert.pem" \
    -subj "/CN=ROS2_CA/O=ROS2_Organization"

echo "✅ Forged CA created (same name, different key)"
echo ""

echo "Step 2: Creating certificate with forged CA..."
echo ""

FORGED_NODE_DIR="$MITM_DIR/forged_node"
mkdir -p "$FORGED_NODE_DIR"

# Generate node key
openssl genrsa -out "$FORGED_NODE_DIR/key.pem" 2048

# Create certificate request
openssl req -new -key "$FORGED_NODE_DIR/key.pem" \
    -out "$FORGED_NODE_DIR/req.pem" \
    -subj "/CN=robot_controller"

# Sign with forged CA
openssl x509 -req -in "$FORGED_NODE_DIR/req.pem" \
    -CA "$FORGED_CA_DIR/ca_cert.pem" \
    -CAkey "$FORGED_CA_DIR/ca_key.pem" \
    -CAcreateserial \
    -out "$FORGED_NODE_DIR/cert.pem" \
    -days 365

cp "$FORGED_CA_DIR/ca_cert.pem" "$FORGED_NODE_DIR/"

rm -f "$FORGED_NODE_DIR/req.pem"

echo "✅ Certificate signed with forged CA created"
echo ""

echo "Step 3: Attempting to use forged certificate..."
echo ""

# Create enclaves
mkdir -p "$FORGED_NODE_DIR/enclaves"
if command -v ros2 &> /dev/null; then
    ros2 security create_keystore "$FORGED_NODE_DIR" 2>/dev/null || true
    ros2 security create_enclave "$FORGED_NODE_DIR" "/" 2>/dev/null || true
fi

# Try to use forged certificate
export ROS_SECURITY_KEYSTORE="$FORGED_NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
sleep 3

echo "Checking if forged certificate is accepted..."
echo ""

NODES=$(ros2 node list 2>/dev/null)
TOPICS=$(ros2 topic list 2>/dev/null)

echo "=========================================="
echo "Test Results"
echo "=========================================="
echo ""

if [ -z "$NODES" ]; then
    echo "✅ SUCCESS (for security): Forged certificate REJECTED"
    echo "   CA verification detected the forgery"
    echo "   This is the CORRECT behavior"
else
    echo "⚠️  WARNING: Forged certificate ACCEPTED!"
    echo "   This indicates a critical security vulnerability"
    echo "   CA verification may not be working"
fi

echo ""
echo "Conclusion:"
echo "  - If rejected: SROS2 CA verification is working correctly"
echo "  - If accepted: CA verification is not properly configured"
echo ""

# Clean up
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
unset ROS_SECURITY_STRATEGY

