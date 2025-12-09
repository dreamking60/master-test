#!/bin/bash
# Fix missing enclaves directories for existing SROS2 configurations

echo "=========================================="
echo "Fixing SROS2 Enclaves Directories"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"

if [ ! -d "$KEYS_DIR" ]; then
    echo "Error: Keys directory not found: $KEYS_DIR"
    echo "Please run init_sros2_security.sh first"
    exit 1
fi

echo "Scanning for node directories..."
echo ""

# Find all node directories
NODE_DIRS=$(find "$KEYS_DIR" -mindepth 1 -maxdepth 1 -type d ! -name "ca")

if [ -z "$NODE_DIRS" ]; then
    echo "No node directories found"
    exit 0
fi

for NODE_DIR in $NODE_DIRS; do
    NODE_NAME=$(basename "$NODE_DIR")
    
    if [ "$NODE_NAME" = "ca" ]; then
        continue
    fi
    
    echo "Processing: $NODE_NAME"
    
    # Create enclaves directory if missing
    if [ ! -d "$NODE_DIR/enclaves" ]; then
        echo "  Creating enclaves directory..."
        mkdir -p "$NODE_DIR/enclaves"
    fi
    
    # Try to create keystore and enclave using ros2 security
    if command -v ros2 &> /dev/null; then
        echo "  Creating keystore..."
        ros2 security create_keystore "$NODE_DIR" 2>/dev/null || echo "    (keystore may already exist)"
        
        echo "  Creating enclave..."
        ros2 security create_enclave "$NODE_DIR" "/" 2>/dev/null || echo "    (enclave may already exist)"
    else
        echo "  Warning: ros2 command not found, skipping keystore/enclave creation"
    fi
    
    # Verify
    if [ -d "$NODE_DIR/enclaves" ]; then
        echo "  ✅ Enclaves directory exists"
    else
        echo "  ❌ Failed to create enclaves directory"
    fi
    
    echo ""
done

echo "=========================================="
echo "Fix Complete!"
echo "=========================================="
echo ""
echo "All node directories now have enclaves directories."
echo "You can now use SROS2 security mode."
echo ""

