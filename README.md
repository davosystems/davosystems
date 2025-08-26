# Davo Systems

![ui](https://github.com/user-attachments/assets/81f3b42e-1e6a-4466-97ac-2e4858561060)

On-chain coordination layer for robotics on Base.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI Contracts](https://github.com/davo-systems/davo-systems/actions/workflows/ci-contracts.yml/badge.svg)](https://github.com/davo-systems/davo-systems/actions/workflows/ci-contracts.yml)
[![CI ROS](https://github.com/davo-systems/davo-systems/actions/workflows/ci-ros.yml/badge.svg)](https://github.com/davo-systems/davo-systems/actions/workflows/ci-ros.yml)
[![Deploy Target](https://img.shields.io/badge/Deploy%20Target-Base%20Sepolia-blue.svg)](https://sepolia.basescan.org/)

## Overview

Davo Systems provides a bridge between ROS 2 robotics systems and Ethereum-compatible blockchains, enabling decentralized task coordination and escrow for autonomous robots.

### Key Features

- **Task Escrow**: Secure on-chain task posting and settlement
- **Commit-Reveal Bidding**: Sealed-bid auction mechanism for task assignment
- **Robot Identity**: ERC721-based robot identity binding
- **ROS 2 Bridge**: Real-time integration with Open-RMF systems
- **Fee Distribution**: 50% buyback+burn, 50% treasury allocation

## Quick Start

### Prerequisites

- Node.js 18+ and npm
- Python 3.8+ with pip
- Foundry (forge)
- ROS 2 Humble or newer
- Docker (for demo)

### 10-Minute Demo

1. **Clone and setup**:
   ```bash
   git clone https://github.com/davo-systems/davo-systems.git
   cd davo-systems
   cp .env.example .env
   # Edit .env with your Base Sepolia RPC URL and private key
   ```

2. **Deploy contracts**:
   ```bash
   cd contracts
   forge install
   forge build
   forge script script/DeployBaseSepolia.s.sol --rpc-url $BASE_SEPOLIA_RPC --broadcast --verify
   ```

3. **Run demo**:
   ```bash
   cd ../demos
   chmod +x run_rmf_airport.sh
   ./run_rmf_airport.sh
   ```

4. **View results**:
   - Check `docs/addresses.md` for deployed contract addresses
   - Visit Base Sepolia explorer links in the demo output
   - Review task lifecycle in the sequence diagram below

## Architecture

![Architecture Diagram](docs/architecture.png)

The system consists of:

- **Smart Contracts**: Task escrow, bidding, and fee distribution on Base
- **ROS 2 Bridge**: Python node that subscribes to RMF topics and posts transactions
- **Robot Identity**: ERC721 tokens binding robot public keys to on-chain identities
- **Demo Environment**: Open-RMF airport simulation with bridge integration

## Task Lifecycle

![Task Sequence](docs/sequence-task-lifecycle.png)

1. **Task Posted**: Off-chain orchestrator posts task specification and budget
2. **Bids Committed**: Robots submit sealed bids during commit window
3. **Bids Revealed**: Winning bid is determined and task assigned
4. **Task Execution**: Robot performs task in physical environment
5. **Result Submission**: Robot signs and submits task result
6. **Settlement**: Winner receives payment, fees distributed

## Contract Addresses

Deployed contracts on Base Sepolia:

- **DAVO Token**: [View on Explorer](https://sepolia.basescan.org/address/0x...)
- **TaskEscrow**: [View on Explorer](https://sepolia.basescan.org/address/0x...)
- **RobotIdentity**: [View on Explorer](https://sepolia.basescan.org/address/0x...)
- **FeeRouter**: [View on Explorer](https://sepolia.basescan.org/address/0x...)

See [docs/addresses.md](docs/addresses.md) for complete deployment details.

## Development

### Project Structure

```
davo-systems/
├── contracts/           # Foundry smart contracts
├── ros2-davo-bridge/    # ROS 2 bridge package
├── demos/              # Demo scripts and configurations
├── docs/               # Documentation and diagrams
├── subgraph/           # The Graph indexing (optional)
└── .github/workflows/  # CI/CD pipelines
```

### Testing

```bash
# Smart contracts
cd contracts
forge test

# ROS 2 bridge
cd ros2-davo-bridge
pytest

# Full integration
cd demos
./run_rmf_airport.sh
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests
5. Submit a pull request

## Security

- All contracts are tested with comprehensive test coverage
- Fee router uses timelock for parameter changes
- Reentrancy guards on critical functions
- EIP-712 signatures for task result verification

See [docs/risk-notes.md](docs/risk-notes.md) for detailed security considerations.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Forked repositories maintain their original licenses:
- Open-RMF: Apache 2.0
- Nav2: BSD-3
- micro-ROS Agent: Apache 2.0
- Gazebo: Apache 2.0

## Acknowledgments

- [Open-RMF](https://github.com/open-rmf) for robotics middleware
- [Foundry](https://getfoundry.sh/) for smart contract development
- [Base](https://base.org/) for the L2 infrastructure
- [OpenZeppelin](https://openzeppelin.com/) for secure contract libraries
