# SROS2 Security Testing

This experiment tests ROS2 security mechanisms (SROS2) and attempts to attack secured systems.

## Overview

SROS2 (Secure ROS2) provides:
- **Encryption**: All messages encrypted
- **Authentication**: Nodes need certificates
- **Access Control**: Permission-based policies

## Experiment Goals

1. **Setup SROS2**: Configure secure ROS2 environment
2. **Test Legitimate Access**: Verify authorized nodes work
3. **Test Unauthorized Access**: Attempt attacks without proper credentials
4. **Evaluate Security**: See if SROS2 actually prevents attacks

## Quick Start

### Step 1: Setup Certificate Authority (CA)

```bash
cd sros2_test/scripts
./setup_ca.sh
```

### Step 2: Generate Node Certificates

```bash
./generate_certificates.sh
```

### Step 3: Configure Security Policies

```bash
./setup_security_policies.sh
```

### Step 4: Test Secure Communication

```bash
./test_secure_communication.sh
```

### Step 5: Attempt Attack (Should Fail)

```bash
./test_unauthorized_attack.sh
```

## Files

- `setup_ca.sh` - Setup Certificate Authority
- `generate_certificates.sh` - Generate node certificates
- `setup_security_policies.sh` - Configure access policies
- `test_secure_communication.sh` - Test legitimate access
- `test_unauthorized_attack.sh` - Test attack attempts
- `bypass_test.sh` - Test if security can be bypassed

## Prerequisites

- ROS2 with SROS2 support
- OpenSSL (for certificate generation)
- Python 3

## Notes

- SROS2 configuration is complex
- Requires certificate management
- Performance overhead compared to unsecured ROS2
- This tests if security actually works

