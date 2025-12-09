# Man-in-the-Middle (MITM) Attack Theory

## What is MITM Attack?

A Man-in-the-Middle (MITM) attack is where an attacker intercepts and potentially modifies communication between two parties without their knowledge.

```
Legitimate Communication:
[Node A] <---> [Node B]

MITM Attack:
[Node A] <---> [Attacker] <---> [Node B]
```

## MITM Attack Vectors on ROS2

### 1. Certificate Replacement

**Attack**: Attacker replaces legitimate certificates with their own.

**How it works**:
- Attacker gains access to keystore directory
- Replaces `cert.pem` and `key.pem` with attacker's certificates
- Attempts to connect using attacker's credentials

**Defense**: Certificate validation should reject certificates from unknown CA.

### 2. CA Forgery

**Attack**: Attacker creates a forged CA certificate.

**How it works**:
- Attacker creates a CA with same name as legitimate CA
- Signs certificates with forged CA
- Attempts to use forged certificates

**Defense**: CA verification checks the CA's public key, not just the name.

### 3. Network Interception

**Attack**: Attacker intercepts network traffic.

**How it works**:
- Attacker uses packet capture tools (tcpdump, wireshark)
- Intercepts DDS packets on network
- Attempts to read or modify messages

**Defense**: TLS/DTLS encryption makes packets unreadable without private keys.

### 4. Certificate Validation Bypass

**Attack**: Attempts to bypass certificate validation.

**How it works**:
- Exploits misconfiguration
- Uses "Permissive" security strategy instead of "Enforce"
- Attempts to connect without proper validation

**Defense**: Use `ROS_SECURITY_STRATEGY=Enforce` for strict validation.

## SROS2 Defense Mechanisms

### 1. Certificate Chain Validation

SROS2 validates the entire certificate chain:
- Node certificate → CA certificate
- CA certificate must be trusted
- Certificate must not be expired
- Certificate must match the node name

### 2. CA Verification

SROS2 verifies CA using cryptographic signatures:
- CA public key is used to verify certificates
- Cannot be forged without CA private key
- Each system has its own CA

### 3. TLS/DTLS Encryption

All communication is encrypted:
- Messages cannot be read without decryption keys
- Keys are derived from certificates
- Each session has unique encryption keys

### 4. Mutual Authentication

Both parties authenticate:
- Node A verifies Node B's certificate
- Node B verifies Node A's certificate
- Both must have valid certificates from trusted CA

## Attack Success Conditions

MITM attack succeeds if:
1. ✅ Attacker's certificate is accepted
2. ✅ Can discover nodes
3. ✅ Can publish/subscribe to topics
4. ✅ Can read encrypted messages

## Attack Failure Conditions (Expected)

MITM attack fails if:
1. ❌ Certificate validation rejects attacker's certificate
2. ❌ CA verification detects forgery
3. ❌ Cannot discover nodes
4. ❌ Cannot access topics
5. ❌ Encrypted traffic is unreadable

## Testing MITM Attacks

### Test 1: Certificate Replacement

Use attacker's certificate (different CA) to connect.

**Expected**: Should be rejected by certificate validation.

### Test 2: CA Forgery

Create forged CA and attempt to use it.

**Expected**: Should be rejected by CA verification.

### Test 3: Network Interception

Capture and analyze network traffic.

**Expected**: Traffic should be encrypted and unreadable.

## Real-World Implications

### If MITM Attacks Succeed

This indicates:
- SROS2 is not properly configured
- Certificate validation is not working
- CA private key may be compromised
- Security policies are misconfigured

### If MITM Attacks Fail (Expected)

This confirms:
- SROS2 is working correctly
- Certificate validation is effective
- Encryption is protecting communication
- Security mechanisms are functioning

## Best Practices

1. **Protect CA Private Key**:
   - Store securely
   - Never share
   - Use strong passphrase

2. **Use Enforce Strategy**:
   - `ROS_SECURITY_STRATEGY=Enforce`
   - Not "Permissive"

3. **Regular Certificate Rotation**:
   - Renew certificates periodically
   - Revoke compromised certificates

4. **Monitor Certificate Usage**:
   - Log certificate validation events
   - Alert on validation failures

5. **Network Security**:
   - Use firewalls
   - Network segmentation
   - Monitor for suspicious activity

## Conclusion

Properly configured SROS2 should **defend against MITM attacks** through:
- Certificate validation
- CA verification
- Encryption
- Mutual authentication

If MITM attacks succeed, it indicates a configuration problem, not a flaw in SROS2 itself.

