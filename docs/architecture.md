# Davo Systems Architecture

## System Overview

Davo Systems provides a decentralized coordination layer for robotics by bridging ROS 2 systems with Ethereum-compatible blockchains. The architecture enables secure task posting, sealed-bid auctions, and automated settlement for autonomous robot operations.

## Core Components

### 1. Smart Contracts (Base Sepolia)

#### DAVO Token (ERC20)
- **Purpose**: Native token for task payments and fee distribution
- **Supply**: Fixed 1,000,000 tokens minted to deployer
- **Features**: Burnable for deflationary mechanics

#### RobotIdentity (ERC721)
- **Purpose**: On-chain identity binding for robots
- **Features**: 
  - Binds robot public keys to EVM addresses
  - EIP-712 domain for task result signatures
  - Owner-controlled key binding

#### TaskEscrow
- **Purpose**: Core task coordination contract
- **Features**:
  - Task posting with budget escrow
  - Commit-reveal bidding mechanism
  - EIP-712 signature verification for results
  - Automated settlement and payment

#### CommitRevealBids
- **Purpose**: Sealed-bid auction system
- **Features**:
  - Two-phase bidding (commit + reveal)
  - Hash-based bid commitment
  - Lowest bid wins with reputation tiebreak

#### FeeRouter
- **Purpose**: Fee distribution and tokenomics
- **Features**:
  - 50% buyback+burn, 50% treasury
  - Timelock for parameter changes
  - Safe treasury integration

#### BuybackBurner
- **Purpose**: Token burn mechanism
- **Features**:
  - Mock swap for testnet (direct burn)
  - Interface for mainnet DEX integration

#### TelemetryAttestor (Optional)
- **Purpose**: Neutral verification for task results
- **Features**:
  - Allowlisted signer attestation
  - Alternative to robot self-signing

### 2. ROS 2 Bridge (ros2-davo-bridge)

#### Bridge Node
- **Language**: Python (rclpy + web3.py)
- **Subscriptions**:
  - `/task_states` - Task lifecycle events
  - `/fleet_states` - Robot status and identity
- **Actions**:
  - Posts tasks to TaskEscrow
  - Submits signed task results
  - Maps RMF task IDs to on-chain IDs

#### Signer Module
- **Purpose**: EIP-712 signature generation
- **Features**:
  - Task result struct hashing
  - Robot key management
  - Domain separation

### 3. Demo Environment

#### Open-RMF Integration
- **Base**: rmf_demos airport world
- **Extensions**: Davo bridge integration
- **Features**:
  - Real-time task simulation
  - Robot fleet coordination
  - End-to-end workflow demonstration

## Data Flow

### Task Lifecycle

1. **Task Creation**
   - Off-chain orchestrator defines task specification
   - Bridge node posts task to TaskEscrow
   - DAVO tokens escrowed for budget

2. **Bidding Phase**
   - Robots commit sealed bids (hash of price + salt)
   - Commit window closes, reveal window opens
   - Robots reveal bids with price and salt
   - Lowest valid bid wins

3. **Task Execution**
   - Winning robot assigned to task
   - Robot performs task in physical environment
   - RMF system tracks task progress

4. **Result Submission**
   - Robot computes result hash from telemetry
   - Signs result with EIP-712 signature
   - Bridge submits result to TaskEscrow

5. **Settlement**
   - TaskEscrow verifies signature
   - Winner receives payment
   - Fees routed to treasury and burn

### Message Flow

```
ROS 2 Topics                    Bridge Node                    Blockchain
     |                              |                              |
     |-- Task Requested ----------->|                              |
     |                              |-- postTask() --------------->|
     |                              |                              |
     |-- Task Assigned ----------->|                              |
     |                              |-- assignWinner() ----------->|
     |                              |                              |
     |-- Task Completed ---------->|                              |
     |                              |-- submitResult() ----------->|
     |                              |                              |
     |                              |<-- TaskSettled event --------|
```

## Security Model

### On-Chain Security
- **Reentrancy Guards**: Protect settlement functions
- **Access Control**: Owner-only operations for critical functions
- **Signature Verification**: EIP-712 for task result authenticity
- **Timelock**: Parameter changes require 48-hour delay

### Off-Chain Security
- **Private Key Management**: Secure storage for robot keys
- **Network Security**: Encrypted RPC communication
- **Input Validation**: Bridge validates all inputs before submission

### Risk Mitigation
- **Oracle Risk**: Optional TelemetryAttestor for neutral verification
- **Key Management**: Robot identity binding prevents key compromise
- **Replay Protection**: Nonce-based transaction ordering

## Integration Points

### Open-RMF Compatibility
- **Topic Mapping**: Standard RMF topic names
- **Message Types**: Compatible with existing RMF messages
- **Fleet Adapters**: No modification required to existing adapters

### Blockchain Integration
- **Base Sepolia**: Primary deployment target
- **EIP-712**: Standard signature format
- **ERC Standards**: Compatible with existing tooling

### External Dependencies
- **Foundry**: Smart contract development and testing
- **OpenZeppelin**: Secure contract libraries
- **Safe**: Treasury management
- **Chainlink**: Oracle integration (future)

## Scalability Considerations

### Performance
- **Gas Optimization**: Efficient contract design for Base L2
- **Batch Operations**: Multiple tasks in single transaction
- **Off-Chain Computation**: Heavy computation off-chain, results on-chain

### Horizontal Scaling
- **Multiple Bridges**: Independent bridge nodes per fleet
- **Sharded Tasks**: Task partitioning across multiple contracts
- **Layer 2**: Base provides high throughput for task coordination

### Vertical Scaling
- **Robot Identity**: Unlimited robot registration
- **Task Complexity**: Flexible task specification format
- **Fee Distribution**: Configurable fee routing

## Future Extensions

### Planned Features
- **Subgraph Indexing**: The Graph integration for querying
- **Multi-Chain**: Support for additional L2s
- **Advanced Bidding**: Multi-round auctions, reputation systems
- **Task Templates**: Reusable task specifications

### Integration Opportunities
- **DeFi Protocols**: Yield farming for treasury funds
- **Cross-Chain**: Interoperability with other robotics networks
- **AI/ML**: Automated task optimization and bidding strategies
