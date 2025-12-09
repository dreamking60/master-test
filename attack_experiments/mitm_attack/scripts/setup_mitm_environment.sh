#!/bin/bash
# Setup MITM attack testing environment

echo "=========================================="
echo "MITM Attack Testing Environment Setup"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MITM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Setup ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

echo "This script sets up the environment for MITM attack testing."
echo ""
echo "⚠️  WARNING: These are security tests only!"
echo "   - Do NOT use on production systems"
echo "   - For research and education purposes"
echo ""

read -p "Continue? [y/N]: " confirm
if [ "$confirm" != "y" ]; then
    echo "Setup cancelled"
    exit 0
fi

echo ""
echo "Step 1: Creating attacker CA and certificates..."
echo ""

# Create attacker's CA
ATTACKER_KEYS_DIR="$MITM_DIR/attacker_keys"
mkdir -p "$ATTACKER_KEYS_DIR/ca"

if [ ! -f "$ATTACKER_KEYS_DIR/ca/ca_key.pem" ]; then
    echo "Creating attacker CA..."
    openssl genrsa -out "$ATTACKER_KEYS_DIR/ca/ca_key.pem" 2048
    openssl req -new -x509 -days 365 -key "$ATTACKER_KEYS_DIR/ca/ca_key.pem" \
        -out "$ATTACKER_KEYS_DIR/ca/ca_cert.pem" \
        -subj "/CN=AttackerCA/O=AttackerOrg"
    echo "✅ Attacker CA created"
else
    echo "✅ Attacker CA already exists"
fi

# Create attacker's node certificate
ATTACKER_NODE_DIR="$ATTACKER_KEYS_DIR/mitm_attacker"
mkdir -p "$ATTACKER_NODE_DIR"

if [ ! -f "$ATTACKER_NODE_DIR/cert.pem" ]; then
    echo "Creating attacker node certificate..."
    
    # Generate private key
    openssl genrsa -out "$ATTACKER_NODE_DIR/key.pem" 2048
    
    # Create certificate request
    openssl req -new -key "$ATTACKER_NODE_DIR/key.pem" \
        -out "$ATTACKER_NODE_DIR/req.pem" \
        -subj "/CN=mitm_attacker"
    
    # Sign with attacker's CA
    openssl x509 -req -in "$ATTACKER_NODE_DIR/req.pem" \
        -CA "$ATTACKER_KEYS_DIR/ca/ca_cert.pem" \
        -CAkey "$ATTACKER_KEYS_DIR/ca/ca_key.pem" \
        -CAcreateserial \
        -out "$ATTACKER_NODE_DIR/cert.pem" \
        -days 365
    
    # Copy CA cert
    cp "$ATTACKER_KEYS_DIR/ca/ca_cert.pem" "$ATTACKER_NODE_DIR/"
    
    # Clean up
    rm -f "$ATTACKER_NODE_DIR/req.pem"
    
    echo "✅ Attacker node certificate created"
else
    echo "✅ Attacker node certificate already exists"
fi

# Create enclaves directory
mkdir -p "$ATTACKER_NODE_DIR/enclaves"
if command -v ros2 &> /dev/null; then
    ros2 security create_keystore "$ATTACKER_NODE_DIR" 2>/dev/null || true
    ros2 security create_enclave "$ATTACKER_NODE_DIR" "/" 2>/dev/null || true
fi

echo ""
echo "Step 2: Creating test configuration..."
echo ""

# Save configuration
cat > "$MITM_DIR/mitm_config.txt" << EOF
# MITM Attack Configuration
ATTACKER_KEYS_DIR=$ATTACKER_KEYS_DIR
ATTACKER_NODE_DIR=$ATTACKER_NODE_DIR
ATTACKER_CA_CERT=$ATTACKER_KEYS_DIR/ca/ca_cert.pem
ATTACKER_NODE_CERT=$ATTACKER_NODE_DIR/cert.pem
EOF

echo "✅ Configuration saved to: $MITM_DIR/mitm_config.txt"
echo ""

echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Attacker credentials created:"
echo "  CA: $ATTACKER_KEYS_DIR/ca/"
echo "  Node: $ATTACKER_NODE_DIR/"
echo ""
echo "⚠️  Note: These certificates are from a DIFFERENT CA"
echo "   They should be REJECTED by properly configured SROS2"
echo ""
echo "Next steps:"
echo "  1. Run: ./test_certificate_replacement.sh"
echo "  2. Run: ./test_ca_forgery.sh"
echo "  3. Check results in: ../results/"
echo ""

