# MITM Attack Testing - Quick Start

## Overview

Test whether Man-in-the-Middle (MITM) attacks can bypass SROS2 security.

## Prerequisites

1. **SROS2 configured** on target system
2. **Target system running** with SROS2 enabled
3. **Network access** to target system

## Quick Test

### Step 1: Setup MITM Environment

```bash
cd /home/stevenchen/master-test/attack_experiments/mitm_attack/scripts
./setup_mitm_environment.sh
```

This creates:
- Attacker's CA (different from target system)
- Attacker's node certificate
- Test configuration

### Step 2: Test Certificate Replacement

```bash
./test_certificate_replacement.sh
```

**Expected Result**: Should FAIL (certificate rejected)

### Step 3: Test CA Forgery

```bash
./test_ca_forgery.sh
```

**Expected Result**: Should FAIL (forged CA rejected)

### Step 4: Test Network Interception (Optional)

```bash
sudo ./test_network_interception.sh
```

**Expected Result**: Traffic encrypted, unreadable

## Expected Results

If SROS2 is properly configured:
- ✅ All attacks should **FAIL**
- ✅ Certificate validation should **REJECT** unauthorized certificates
- ✅ CA verification should **DETECT** forgeries
- ✅ Encrypted traffic should be **UNREADABLE**

## What This Proves

- **If attacks fail**: SROS2 is working correctly ✅
- **If attacks succeed**: SROS2 configuration may be incomplete ⚠️

## Full Documentation

- `docs/MITM_ATTACK_THEORY.md` - Detailed attack theory
- `docs/SROS2_MITM_DEFENSE.md` - How SROS2 defends against MITM
- `README.md` - Complete overview

