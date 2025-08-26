# Contract Addresses

## Base Sepolia Deployment

This document contains the deployed contract addresses for Davo Systems on Base Sepolia testnet.

### Core Contracts

| Contract | Address | Explorer |
|----------|---------|----------|
| DAVO Token | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| TaskEscrow | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| RobotIdentity | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| CommitRevealBids | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| FeeRouter | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| BuybackBurner | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |
| TelemetryAttestor | `0x...` | [View](https://sepolia.basescan.org/address/0x...) |

### Treasury

| Component | Address | Description |
|-----------|---------|-------------|
| Safe Treasury | `0x...` | Multi-sig treasury for fee collection |

### Deployment Details

- **Network**: Base Sepolia (Chain ID: 84532)
- **Deployer**: `0x...`
- **Deployment Block**: `...`
- **Deployment Transaction**: [View](https://sepolia.basescan.org/tx/0x...)

### Configuration

#### Fee Router Parameters
- **Treasury Split**: 50%
- **Burn Split**: 50%
- **Timelock Duration**: 48 hours

#### Task Escrow Parameters
- **Commit Window**: 300 seconds (5 minutes)
- **Reveal Window**: 300 seconds (5 minutes)
- **Fee Percentage**: 2.5%

#### Robot Identity Parameters
- **EIP-712 Domain**: DavoSystems v1
- **Chain ID**: 84532

### Recent Transactions

| Transaction | Type | Block | Explorer |
|-------------|------|-------|----------|
| `0x...` | Task Posted | `...` | [View](https://sepolia.basescan.org/tx/0x...) |
| `0x...` | Task Settled | `...` | [View](https://sepolia.basescan.org/tx/0x...) |
| `0x...` | Fees Routed | `...` | [View](https://sepolia.basescan.org/tx/0x...) |

### Token Distribution

| Holder | DAVO Balance | Percentage |
|--------|-------------|------------|
| Deployer | `...` | `...`% |
| TaskEscrow | `...` | `...`% |
| Treasury | `...` | `...`% |
| Burned | `...` | `...`% |

### Robot Registrations

| Robot ID | Name | Public Key Hash | Owner |
|----------|------|-----------------|-------|
| `1` | `airport_robot_01` | `0x...` | `0x...` |
| `2` | `airport_robot_02` | `0x...` | `0x...` |

---

*This file is automatically updated by deployment scripts. Last updated: [TIMESTAMP]*
