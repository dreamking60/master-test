# Man-in-the-Middle (MITM) Attack on SROS2

## Overview

This experiment tests whether Man-in-the-Middle (MITM) attacks can bypass SROS2 security mechanisms.

## Important Note

SROS2 uses **TLS/DTLS encryption** with certificate-based authentication. In theory, MITM attacks should be **blocked** because:
- Certificate validation detects MITM
- Encrypted communication cannot be easily decrypted
- CA (Certificate Authority) verification prevents unauthorized certificates

However, this experiment tests various MITM scenarios to verify SROS2's actual security.

## MITM Attack Scenarios

### 1. Certificate Replacement Attack
Attempt to replace legitimate certificates with attacker's certificates.

### 2. CA Forgery Attack
Attempt to use a forged CA certificate to sign malicious certificates.

### 3. Certificate Validation Bypass
Test if certificate validation can be bypassed (should fail with proper SROS2).

### 4. Network Interception
Attempt to intercept and modify encrypted messages (should fail).

## Prerequisites

1. **SROS2 configured** - Target system must have SROS2 enabled
2. **Network access** - Attacker must be on the same network
3. **Root/Admin access** - Some attacks require elevated privileges

## Attack Methods

### Method 1: Certificate Replacement

If attacker gains access to keystore directory, they might try to replace certificates.

### Method 2: CA Compromise

If CA private key is stolen, attacker can create valid certificates.

### Method 3: Network Interception

Attempt to intercept DDS packets (encrypted, should fail).

## Expected Results

With **properly configured SROS2**:
- ✅ MITM attacks should **FAIL**
- ✅ Certificate validation should **REJECT** unauthorized certificates
- ✅ Encrypted communication should **PREVENT** message interception
- ✅ CA verification should **BLOCK** forged certificates

## Files

- `scripts/test_certificate_replacement.sh` - Test certificate replacement attack
- `scripts/test_ca_forgery.sh` - Test CA forgery attack
- `scripts/test_network_interception.sh` - Test network interception
- `scripts/setup_mitm_environment.sh` - Setup MITM testing environment
- `docs/MITM_ATTACK_THEORY.md` - Detailed theory and explanation
- `docs/SROS2_MITM_DEFENSE.md` - How SROS2 defends against MITM

## Quick Start

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
./setup_mitm_environment.sh
./test_certificate_replacement.sh
```

## Security Implications

If MITM attacks succeed, it means:
- SROS2 configuration is incomplete
- Certificates are not properly validated
- CA private key may be compromised
- Security policies are misconfigured

If MITM attacks fail (expected), it confirms:
- SROS2 is working correctly
- Certificate validation is effective
- Encryption is protecting communication
- Security mechanisms are functioning

