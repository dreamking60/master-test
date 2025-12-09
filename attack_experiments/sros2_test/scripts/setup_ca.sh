#!/bin/bash
# Setup Certificate Authority (CA) for SROS2

echo "=========================================="
echo "SROS2 Certificate Authority Setup"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"
CA_DIR="$KEYS_DIR/ca"

# Create directories
mkdir -p "$CA_DIR"

echo "This script sets up a Certificate Authority (CA) for SROS2."
echo "The CA will be used to sign all node certificates."
echo ""

# Check if CA already exists
if [ -f "$CA_DIR/ca_key.pem" ]; then
    echo "CA already exists at $CA_DIR"
    read -p "Regenerate CA? [y/N]: " regenerate
    if [ "$regenerate" != "y" ]; then
        echo "Using existing CA"
        exit 0
    fi
    rm -rf "$CA_DIR"/*
fi

echo "Generating CA key and certificate..."
echo ""

# Generate CA private key
openssl genrsa -out "$CA_DIR/ca_key.pem" 4096

# Generate CA certificate
openssl req -new -x509 -key "$CA_DIR/ca_key.pem" -out "$CA_DIR/ca_cert.pem" \
    -days 365 \
    -subj "/C=US/ST=State/L=City/O=ROS2/CN=CA"

echo ""
echo "=========================================="
echo "CA Setup Complete!"
echo "=========================================="
echo ""
echo "CA files created:"
echo "  Private key: $CA_DIR/ca_key.pem"
echo "  Certificate: $CA_DIR/ca_cert.pem"
echo ""
echo "⚠️  Keep the CA private key secure!"
echo "   Anyone with this key can create valid certificates"
echo ""

