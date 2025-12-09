#!/bin/bash
# One-click SROS2 security initialization script
# This script sets up complete SROS2 security configuration

set -e  # Exit on error

echo "=========================================="
echo "SROS2 Security One-Click Initialization"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"
CA_DIR="$KEYS_DIR/ca"

# Create keys directory
mkdir -p "$KEYS_DIR"

echo "This script will:"
echo "  1. Create Certificate Authority (CA)"
echo "  2. Generate certificates for nodes"
echo "  3. Setup security policies"
echo "  4. Create environment setup script"
echo ""

# Check if OpenSSL is installed
if ! command -v openssl &> /dev/null; then
    echo "Error: OpenSSL is not installed"
    echo "Install it with: sudo apt install openssl"
    exit 1
fi

# Check if ROS2 is available
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
fi

echo "ROS2 environment: $ROS_DISTRO"
echo ""

# ==========================================
# Step 1: Setup Certificate Authority
# ==========================================

echo "=========================================="
echo "Step 1: Creating Certificate Authority"
echo "=========================================="
echo ""

if [ -f "$CA_DIR/ca_key.pem" ]; then
    echo "CA already exists at $CA_DIR"
    read -p "Regenerate CA? [y/N]: " regenerate
    if [ "$regenerate" = "y" ]; then
        rm -rf "$CA_DIR"/*
        echo "Regenerating CA..."
    else
        echo "Using existing CA"
    fi
fi

if [ ! -f "$CA_DIR/ca_key.pem" ]; then
    echo "Generating CA private key..."
    openssl genrsa -out "$CA_DIR/ca_key.pem" 4096 2>/dev/null
    
    echo "Generating CA certificate..."
    openssl req -new -x509 -key "$CA_DIR/ca_key.pem" -out "$CA_DIR/ca_cert.pem" \
        -days 365 \
        -subj "/C=US/ST=State/L=City/O=ROS2/CN=CA" 2>/dev/null
    
    echo "✅ CA created successfully"
else
    echo "✅ CA already exists"
fi

echo ""

# ==========================================
# Step 2: Generate Node Certificates
# ==========================================

echo "=========================================="
echo "Step 2: Generating Node Certificates"
echo "=========================================="
echo ""

# Default nodes
DEFAULT_NODES=("robot_controller" "normal_operator" "gazebo_sim")

echo "Default nodes to create:"
for node in "${DEFAULT_NODES[@]}"; do
    echo "  - $node"
done
echo ""

read -p "Use default nodes? [Y/n]: " use_default
if [ "$use_default" != "n" ]; then
    NODES=("${DEFAULT_NODES[@]}")
else
    echo ""
    echo "Enter node names (one per line, empty line to finish):"
    NODES=()
    while true; do
        read -p "Node name: " node_name
        if [ -z "$node_name" ]; then
            break
        fi
        NODES+=("$node_name")
    done
fi

echo ""
echo "Generating certificates for ${#NODES[@]} nodes..."

for node_name in "${NODES[@]}"; do
    NODE_DIR="$KEYS_DIR/$node_name"
    mkdir -p "$NODE_DIR"
    
    if [ -f "$NODE_DIR/cert.pem" ]; then
        echo "  ⚠️  Certificate for $node_name already exists, skipping..."
        continue
    fi
    
    echo "  Generating certificate for: $node_name"
    
    # Generate private key
    openssl genrsa -out "$NODE_DIR/key.pem" 2048 2>/dev/null
    
    # Generate certificate signing request
    openssl req -new -key "$NODE_DIR/key.pem" -out "$NODE_DIR/req.pem" \
        -subj "/C=US/ST=State/L=City/O=ROS2/CN=$node_name" 2>/dev/null
    
    # Sign certificate with CA
    openssl x509 -req -in "$NODE_DIR/req.pem" -CA "$CA_DIR/ca_cert.pem" \
        -CAkey "$CA_DIR/ca_key.pem" -CAcreateserial \
        -out "$NODE_DIR/cert.pem" -days 365 2>/dev/null
    
    # Copy CA certificate
    cp "$CA_DIR/ca_cert.pem" "$NODE_DIR/" 2>/dev/null
    
    # Clean up CSR
    rm -f "$NODE_DIR/req.pem"
    
    # Create enclaves directory (required by SROS2)
    mkdir -p "$NODE_DIR/enclaves"
    
    # Create enclave using ros2 security command
    if command -v ros2 &> /dev/null; then
        # Create keystore if it doesn't exist
        if [ ! -f "$NODE_DIR/enclaves/.keystore" ]; then
            ros2 security create_keystore "$NODE_DIR" 2>/dev/null || true
        fi
        # Create enclave
        ros2 security create_enclave "$NODE_DIR" "/" 2>/dev/null || true
    fi
    
    echo "    ✅ Certificate created"
done

echo "✅ All certificates generated"
echo ""

# ==========================================
# Step 3: Setup Security Policies
# ==========================================

echo "=========================================="
echo "Step 3: Creating Security Policies"
echo "=========================================="
echo ""

for node_name in "${NODES[@]}"; do
    NODE_DIR="$KEYS_DIR/$node_name"
    POLICY_FILE="$NODE_DIR/policy.xml"
    
    if [ -f "$POLICY_FILE" ]; then
        echo "  ⚠️  Policy for $node_name already exists, skipping..."
        continue
    fi
    
    echo "  Creating policy for: $node_name"
    
    # Create policy file
    cat > "$POLICY_FILE" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="$node_name" ns="/" />
      </profiles>
      <topics>
        <topic name="/cmd_vel">
          <publish>
            <allow_rule>
              <node>$node_name</node>
            </allow_rule>
          </publish>
          <subscribe>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </subscribe>
        </topic>
        <topic name="/odom">
          <publish>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </publish>
          <subscribe>
            <allow_rule>
              <node>$node_name</node>
            </allow_rule>
          </subscribe>
        </topic>
        <topic name="/scan">
          <publish>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </publish>
          <subscribe>
            <allow_rule>
              <node>$node_name</node>
            </allow_rule>
          </subscribe>
        </topic>
        <topic name="/imu">
          <publish>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </publish>
          <subscribe>
            <allow_rule>
              <node>$node_name</node>
            </allow_rule>
          </subscribe>
        </topic>
      </topics>
      <services>
        <service name="*">
          <request>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </request>
          <reply>
            <allow_rule>
              <node>*</node>
            </allow_rule>
          </reply>
        </service>
      </services>
    </enclave>
  </enclaves>
</policy>
EOF
    
    echo "    ✅ Policy created"
done

echo "✅ All policies created"
echo ""

# ==========================================
# Step 4: Create Environment Setup Scripts
# ==========================================

echo "=========================================="
echo "Step 4: Creating Environment Scripts"
echo "=========================================="
echo ""

# Create main setup script
SETUP_SCRIPT="$SCRIPT_DIR/../setup_sros2_env.sh"
cat > "$SETUP_SCRIPT" << 'EOF'
#!/bin/bash
# SROS2 Environment Setup Script
# Source this file to enable SROS2 security

if [ -z "$1" ]; then
    echo "Usage: source setup_sros2_env.sh <node_name>"
    echo ""
    echo "Available nodes:"
    ls -d keys/*/ 2>/dev/null | sed 's|keys/||' | sed 's|/||' | sed 's/^/  - /'
    return 1
fi

NODE_NAME="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/keys"
NODE_DIR="$KEYS_DIR/$NODE_NAME"

if [ ! -d "$NODE_DIR" ]; then
    echo "Error: Node '$NODE_NAME' not found"
    echo "Available nodes:"
    ls -d keys/*/ 2>/dev/null | sed 's|keys/||' | sed 's|/||' | sed 's/^/  - /'
    return 1
fi

if [ ! -f "$NODE_DIR/cert.pem" ]; then
    echo "Error: Certificate not found for '$NODE_NAME'"
    return 1
fi

# Ensure enclaves directory exists
if [ ! -d "$NODE_DIR/enclaves" ]; then
    mkdir -p "$NODE_DIR/enclaves"
    # Try to create keystore and enclave
    if command -v ros2 &> /dev/null; then
        ros2 security create_keystore "$NODE_DIR" 2>/dev/null || true
        ros2 security create_enclave "$NODE_DIR" "/" 2>/dev/null || true
    fi
fi

export ROS_SECURITY_KEYSTORE="$NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo "SROS2 security enabled for: $NODE_NAME"
echo "  Keystore: $ROS_SECURITY_KEYSTORE"
echo "  Security: Enabled"
echo "  Strategy: Enforce"
EOF

chmod +x "$SETUP_SCRIPT"

# Create per-node setup scripts
for node_name in "${NODES[@]}"; do
    NODE_SETUP="$SCRIPT_DIR/../setup_${node_name}_env.sh"
    cat > "$NODE_SETUP" << EOF
#!/bin/bash
# Auto-generated SROS2 environment for $node_name
# Source this file: source setup_${node_name}_env.sh

SCRIPT_DIR="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
NODE_DIR="\$SCRIPT_DIR/keys/$node_name"

# Ensure enclaves directory exists (required by SROS2)
if [ ! -d "\$NODE_DIR/enclaves" ]; then
    mkdir -p "\$NODE_DIR/enclaves"
    # Try to create keystore and enclave using ros2 security
    if command -v ros2 &> /dev/null; then
        ros2 security create_keystore "\$NODE_DIR" 2>/dev/null || true
        ros2 security create_enclave "\$NODE_DIR" "/" 2>/dev/null || true
    fi
fi

export ROS_SECURITY_KEYSTORE="\$NODE_DIR"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

echo "SROS2 security enabled for: $node_name"
echo "  Keystore: \$ROS_SECURITY_KEYSTORE"
echo "  Security: Enabled"
echo "  Strategy: Enforce"
EOF
    chmod +x "$NODE_SETUP"
    echo "  Created: setup_${node_name}_env.sh"
done

echo "✅ Environment scripts created"
echo ""

# ==========================================
# Step 5: Create Quick Reference
# ==========================================

echo "=========================================="
echo "Step 5: Creating Quick Reference"
echo "=========================================="
echo ""

QUICK_REF="$SCRIPT_DIR/../QUICK_REFERENCE.md"
cat > "$QUICK_REF" << EOF
# SROS2 Security Quick Reference

## Setup Complete! ✅

All certificates and policies have been created.

## How to Use

### Method 1: Use Node-Specific Script

\`\`\`bash
source setup_robot_controller_env.sh
cd ../..
./scripts/setup/run_empty_world.sh
\`\`\`

### Method 2: Use Generic Script

\`\`\`bash
source setup_sros2_env.sh robot_controller
cd ../..
./scripts/setup/run_empty_world.sh
\`\`\`

### Method 3: Manual Setup

\`\`\`bash
export ROS_SECURITY_KEYSTORE="\$(pwd)/keys/robot_controller"
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
cd ../..
./scripts/setup/run_empty_world.sh
\`\`\`

## Available Nodes

EOF

for node_name in "${NODES[@]}"; do
    echo "- \`$node_name\`" >> "$QUICK_REF"
done

cat >> "$QUICK_REF" << 'EOF'

## Testing Security

### Test 1: Legitimate Access (Should Work)

```bash
# Terminal 1: Start Gazebo with security
source setup_robot_controller_env.sh
cd ../..
./scripts/setup/run_empty_world.sh

# Terminal 2: Run controller with security
source setup_robot_controller_env.sh
python3 normal_controller.py
```

### Test 2: Unauthorized Access (Should Fail)

```bash
# Terminal 3: Try attack without credentials
unset ROS_SECURITY_KEYSTORE
unset ROS_SECURITY_ENABLE
python3 injection_attack.py --attack-type turn_left
# Should fail - cannot discover nodes
```

## Files Created

- `keys/ca/` - Certificate Authority
- `keys/<node_name>/` - Node certificates and policies
- `setup_<node_name>_env.sh` - Per-node environment scripts
- `setup_sros2_env.sh` - Generic environment script

## Important Notes

- Keep CA private key (`keys/ca/ca_key.pem`) secure!
- All nodes using same CA can communicate
- Policies control what each node can do
- Security is enforced when `ROS_SECURITY_STRATEGY=Enforce`
EOF

echo "✅ Quick reference created: QUICK_REFERENCE.md"
echo ""

# ==========================================
# Summary
# ==========================================

echo "=========================================="
echo "Initialization Complete! ✅"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - CA created: $CA_DIR"
echo "  - Nodes configured: ${#NODES[@]}"
for node_name in "${NODES[@]}"; do
    echo "    • $node_name"
done
echo "  - Environment scripts: Created"
echo "  - Quick reference: QUICK_REFERENCE.md"
echo ""
echo "Next steps:"
echo ""
echo "1. Enable security for a node:"
echo "   source setup_robot_controller_env.sh"
echo ""
echo "2. Start Gazebo with security:"
echo "   source setup_robot_controller_env.sh"
echo "   cd ../..
./scripts/setup/run_empty_world.sh"
echo ""
echo "3. Test security:"
echo "   See QUICK_REFERENCE.md for testing instructions"
echo ""
echo "=========================================="

