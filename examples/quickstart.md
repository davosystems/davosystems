# Davo Systems Quick Start Guide

Get up and running with Davo Systems in under 15 minutes.

## Prerequisites

Before you begin, ensure you have the following installed:

- **Node.js 18+** and npm
- **Python 3.8+** with pip
- **Foundry** (forge) - [Install Guide](https://getfoundry.sh/)
- **ROS 2 Humble** - [Install Guide](https://docs.ros.org/en/humble/Installation.html)
- **Docker** - [Install Guide](https://docs.docker.com/get-docker/)

## Step 1: Clone and Setup

```bash
# Clone the repository
git clone https://github.com/davo-systems/davo-systems.git
cd davo-systems

# Copy environment file
cp env.example .env

# Edit .env with your configuration
nano .env  # or use your preferred editor
```

Edit the `.env` file with your Base Sepolia RPC URL and private keys:

```bash
BASE_SEPOLIA_RPC=https://sepolia.base.org
PRIVATE_KEY=your_private_key_here
ROBOT_PRIVATE_KEY=your_robot_private_key_here
BASESCAN_API_KEY=your_basescan_api_key_here
```

## Step 2: Deploy Contracts

```bash
# Navigate to contracts directory
cd contracts

# Install dependencies
forge install

# Build contracts
forge build

# Deploy to Base Sepolia
forge script script/DeployBaseSepolia.s.sol \
  --rpc-url $BASE_SEPOLIA_RPC \
  --broadcast \
  --verify \
  --etherscan-api-key $BASESCAN_API_KEY
```

This will deploy all contracts and update `docs/addresses.md` with the deployed addresses.

## Step 3: Build ROS 2 Bridge

```bash
# Navigate to bridge directory
cd ../ros2-davo-bridge

# Install Python dependencies
pip install -r requirements.txt

# Build ROS 2 package
colcon build --packages-select davo_bridge
```

## Step 4: Run the Demo

```bash
# Navigate back to project root
cd ..

# Make demo script executable (Linux/macOS)
chmod +x demos/run_rmf_airport.sh

# Run the demo
./demos/run_rmf_airport.sh
```

The demo will:
1. Start the RMF airport simulation
2. Launch the Davo bridge
3. Generate a test task
4. Execute the complete task lifecycle
5. Display transaction links

## Step 5: Verify Results

After the demo completes, check:

1. **Contract Addresses**: `docs/addresses.md`
2. **Demo Data**: `demos/demo_data.json`
3. **Base Sepolia Explorer**: Links provided in demo output

## Troubleshooting

### Common Issues

**"Foundry not found"**
```bash
curl -L https://foundry.paradigm.xyz | bash
foundryup
```

**"ROS 2 not found"**
```bash
# Ubuntu
sudo apt update && sudo apt install ros-humble-desktop

# macOS
brew install ros2
```

**"Docker not found"**
```bash
# Ubuntu
sudo apt install docker.io
sudo systemctl start docker
sudo usermod -aG docker $USER

# macOS
brew install --cask docker
```

**"Permission denied" on demo script**
```bash
chmod +x demos/run_rmf_airport.sh
```

### Network Issues

If you encounter network connectivity issues:

1. Check your Base Sepolia RPC URL
2. Ensure you have sufficient testnet ETH
3. Verify your private keys are correct

### Contract Deployment Issues

If contract deployment fails:

1. Check your private key has sufficient ETH
2. Verify BaseScan API key is correct
3. Ensure RPC endpoint is accessible

## Next Steps

After completing the quickstart:

1. **Explore the Code**: Review the smart contracts and bridge implementation
2. **Run Tests**: Execute `forge test` and `pytest` to verify functionality
3. **Customize**: Modify task specifications and robot configurations
4. **Deploy**: Consider deploying to Base mainnet for production use

## Support

- **Documentation**: [docs/](docs/)
- **Issues**: [GitHub Issues](https://github.com/davo-systems/davo-systems/issues)
- **Discussions**: [GitHub Discussions](https://github.com/davo-systems/davo-systems/discussions)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
