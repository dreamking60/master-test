#!/bin/bash
# Generate certificates for ROS2 nodes

echo "=========================================="
echo "SROS2 Node Certificate Generation"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"
CA_DIR="$KEYS_DIR/ca"

# Check if CA exists
if [ ! -f "$CA_DIR/ca_key.pem" ]; then
    echo "Error: CA not found!"
    echo "Please run setup_ca.sh first"
    exit 1
fi

echo "This script generates certificates for ROS2 nodes."
echo ""

# Get node name
read -p "Enter node name (e.g., 'robot_controller'): " node_name
if [ -z "$node_name" ]; then
    echo "Error: Node name required"
    exit 1
fi

NODE_DIR="$KEYS_DIR/$node_name"
mkdir -p "$NODE_DIR"

echo ""
echo "Generating certificate for: $node_name"
echo ""

# Generate node private key
openssl genrsa -out "$NODE_DIR/key.pem" 2048

# Generate certificate signing request
openssl req -new -key "$NODE_DIR/key.pem" -out "$NODE_DIR/req.pem" \
    -subj "/C=US/ST=State/L=City/O=ROS2/CN=$node_name"

# Sign certificate with CA
openssl x509 -req -in "$NODE_DIR/req.pem" -CA "$CA_DIR/ca_cert.pem" \
    -CAkey "$CA_DIR/ca_key.pem" -CAcreateserial \
    -out "$NODE_DIR/cert.pem" -days 365

# Copy CA certificate (nodes need it to verify other nodes)
cp "$CA_DIR/ca_cert.pem" "$NODE_DIR/"

echo ""
echo "=========================================="
echo "Certificate Generated!"
echo "=========================================="
echo ""
echo "Node: $node_name"
echo "Files:"
echo "  Private key: $NODE_DIR/key.pem"
echo "  Certificate: $NODE_DIR/cert.pem"
echo "  CA cert: $NODE_DIR/ca_cert.pem"
echo ""
echo "To use this certificate:"
echo "  export ROS_SECURITY_KEYSTORE=$NODE_DIR"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
echo ""

