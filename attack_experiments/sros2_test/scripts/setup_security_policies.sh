#!/bin/bash
# Setup SROS2 security policies

echo "=========================================="
echo "SROS2 Security Policy Setup"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYS_DIR="$SCRIPT_DIR/../keys"

echo "This script creates security policies for nodes."
echo "Policies define what each node is allowed to do."
echo ""

# Get node name
read -p "Enter node name: " node_name
if [ -z "$node_name" ]; then
    echo "Error: Node name required"
    exit 1
fi

NODE_DIR="$KEYS_DIR/$node_name"

if [ ! -f "$NODE_DIR/cert.pem" ]; then
    echo "Error: Certificate not found for $node_name"
    echo "Please run generate_certificates.sh first"
    exit 1
fi

POLICY_FILE="$NODE_DIR/policy.xml"

echo ""
echo "Creating policy for: $node_name"
echo ""

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

echo "Policy file created: $POLICY_FILE"
echo ""
echo "This policy allows:"
echo "  - $node_name to publish to /cmd_vel"
echo "  - Any node to subscribe to /cmd_vel"
echo "  - Any node to publish to /odom"
echo "  - $node_name to subscribe to /odom"
echo ""
echo "To restrict access, modify the policy file."
echo ""

