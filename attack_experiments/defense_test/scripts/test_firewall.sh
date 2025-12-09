#!/bin/bash
# Test firewall defense mechanism
# This test verifies that firewall rules can block DDS communication

echo "=========================================="
echo "Defense Test: Firewall Protection"
echo "=========================================="
echo ""

# Check if running as root for firewall commands
if [ "$EUID" -ne 0 ]; then
    echo "Note: Some firewall commands require root privileges"
    echo "You may need to run: sudo ./test_firewall.sh"
    echo ""
fi

echo "This test demonstrates firewall protection against network attacks."
echo ""
echo "DDS uses UDP ports 7400-7500 for discovery and communication."
echo "Blocking these ports prevents network-based attacks."
echo ""

read -p "Press Enter to continue..."

echo "=========================================="
echo "Current Firewall Status"
echo "=========================================="
echo ""

if command -v ufw &> /dev/null; then
    echo "UFW (Uncomplicated Firewall) status:"
    sudo ufw status | head -5
    echo ""
elif command -v firewall-cmd &> /dev/null; then
    echo "firewalld status:"
    sudo firewall-cmd --list-all
    echo ""
else
    echo "No common firewall tool detected (ufw or firewalld)"
    echo ""
fi

echo "=========================================="
echo "Test: Block DDS Ports"
echo "=========================================="
echo ""

read -p "Do you want to test blocking DDS ports? [y/N]: " test_block

if [ "$test_block" = "y" ]; then
    echo ""
    echo "Blocking DDS ports (7400-7500/udp)..."
    
    if command -v ufw &> /dev/null; then
        echo "Using UFW..."
        sudo ufw deny 7400:7500/udp
        echo "✅ DDS ports blocked"
        echo ""
        echo "Test: Try to discover ROS2 nodes from another machine"
        echo "      They should NOT be discoverable now"
        echo ""
        read -p "Press Enter when done testing..."
        echo ""
        read -p "Restore firewall rules? [y/N]: " restore
        if [ "$restore" = "y" ]; then
            sudo ufw delete deny 7400:7500/udp
            echo "✅ Firewall rules restored"
        fi
    elif command -v firewall-cmd &> /dev/null; then
        echo "Using firewalld..."
        sudo firewall-cmd --permanent --add-rich-rule='rule protocol value="udp" port port="7400-7500" reject'
        sudo firewall-cmd --reload
        echo "✅ DDS ports blocked"
        echo ""
        echo "Test: Try to discover ROS2 nodes from another machine"
        echo "      They should NOT be discoverable now"
        echo ""
        read -p "Press Enter when done testing..."
        echo ""
        read -p "Restore firewall rules? [y/N]: " restore
        if [ "$restore" = "y" ]; then
            sudo firewall-cmd --permanent --remove-rich-rule='rule protocol value="udp" port port="7400-7500" reject'
            sudo firewall-cmd --reload
            echo "✅ Firewall rules restored"
        fi
    else
        echo "❌ No firewall tool available"
        echo "Install UFW: sudo apt install ufw"
    fi
fi

echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo ""
echo "Firewall protection:"
echo "  - Blocks DDS ports (7400-7500/udp)"
echo "  - Prevents network-based discovery and attacks"
echo "  - Effective against external attackers"
echo "  - Does NOT protect against local attackers"
echo ""
echo "Defense effectiveness: ⭐⭐⭐⭐ (4/5)"
echo "  - Very effective against network attacks"
echo "  - Easy to implement"
echo "  - May block legitimate remote access"
echo "  - Should be combined with VPN for remote access"
echo ""

