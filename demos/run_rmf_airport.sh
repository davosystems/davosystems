#!/bin/bash

# Davo Systems RMF Airport Demo
# Runs Open-RMF airport world with Davo bridge integration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$DEMO_DIR")"
CONTRACTS_DIR="$PROJECT_ROOT/contracts"
BRIDGE_DIR="$PROJECT_ROOT/ros2-davo-bridge"

# Check if .env file exists
if [ ! -f "$PROJECT_ROOT/.env" ]; then
    echo -e "${RED}Error: .env file not found${NC}"
    echo "Please create .env file with your configuration:"
    echo "cp .env.example .env"
    echo "Then edit .env with your Base Sepolia RPC URL and private keys"
    exit 1
fi

# Load environment variables
source "$PROJECT_ROOT/.env"

# Check required environment variables
if [ -z "$BASE_SEPOLIA_RPC" ]; then
    echo -e "${RED}Error: BASE_SEPOLIA_RPC not set in .env${NC}"
    exit 1
fi

if [ -z "$PRIVATE_KEY" ]; then
    echo -e "${RED}Error: PRIVATE_KEY not set in .env${NC}"
    exit 1
fi

if [ -z "$ROBOT_PRIVATE_KEY" ]; then
    echo -e "${RED}Error: ROBOT_PRIVATE_KEY not set in .env${NC}"
    exit 1
fi

echo -e "${BLUE}🚀 Starting Davo Systems RMF Airport Demo${NC}"
echo "=================================================="

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo -e "${YELLOW}Checking prerequisites...${NC}"

if ! command_exists docker; then
    echo -e "${RED}Error: Docker not found${NC}"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

if ! command_exists forge; then
    echo -e "${RED}Error: Foundry not found${NC}"
    echo "Please install Foundry: https://getfoundry.sh/"
    exit 1
fi

if ! command_exists colcon; then
    echo -e "${RED}Error: ROS 2 not found${NC}"
    echo "Please install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ Prerequisites check passed${NC}"

# Deploy contracts if not already deployed
echo -e "${YELLOW}Checking contract deployment...${NC}"

if [ ! -f "$PROJECT_ROOT/docs/addresses.md" ] || ! grep -q "0x[0-9a-fA-F]\{40\}" "$PROJECT_ROOT/docs/addresses.md"; then
    echo -e "${YELLOW}Contracts not deployed. Deploying to Base Sepolia...${NC}"
    
    cd "$CONTRACTS_DIR"
    
    # Install dependencies
    forge install
    
    # Build contracts
    forge build
    
    # Deploy contracts
    forge script script/DeployBaseSepolia.s.sol \
        --rpc-url "$BASE_SEPOLIA_RPC" \
        --broadcast \
        --verify \
        --etherscan-api-key "$BASESCAN_API_KEY"
    
    echo -e "${GREEN}✓ Contracts deployed successfully${NC}"
else
    echo -e "${GREEN}✓ Contracts already deployed${NC}"
fi

# Update bridge configuration with contract addresses
echo -e "${YELLOW}Updating bridge configuration...${NC}"

# Extract contract addresses from addresses.md
DAVO_TOKEN=$(grep "DAVO Token" "$PROJECT_ROOT/docs/addresses.md" | grep -o "0x[0-9a-fA-F]\{40\}" | head -1)
TASK_ESCROW=$(grep "TaskEscrow" "$PROJECT_ROOT/docs/addresses.md" | grep -o "0x[0-9a-fA-F]\{40\}" | head -1)
ROBOT_IDENTITY=$(grep "RobotIdentity" "$PROJECT_ROOT/docs/addresses.md" | grep -o "0x[0-9a-fA-F]\{40\}" | head -1)

if [ -z "$DAVO_TOKEN" ] || [ -z "$TASK_ESCROW" ] || [ -z "$ROBOT_IDENTITY" ]; then
    echo -e "${RED}Error: Could not extract contract addresses${NC}"
    exit 1
fi

# Update config.yaml with contract addresses
CONFIG_FILE="$BRIDGE_DIR/config/config.yaml"
sed -i.bak "s/davo_token: \"0x0000000000000000000000000000000000000000\"/davo_token: \"$DAVO_TOKEN\"/" "$CONFIG_FILE"
sed -i.bak "s/task_escrow: \"0x0000000000000000000000000000000000000000\"/task_escrow: \"$TASK_ESCROW\"/" "$CONFIG_FILE"
sed -i.bak "s/robot_identity: \"0x0000000000000000000000000000000000000000\"/robot_identity: \"$ROBOT_IDENTITY\"/" "$CONFIG_FILE"

echo -e "${GREEN}✓ Bridge configuration updated${NC}"

# Build ROS 2 bridge
echo -e "${YELLOW}Building ROS 2 bridge...${NC}"

cd "$BRIDGE_DIR"

# Install Python dependencies
pip install -r requirements.txt 2>/dev/null || pip install web3 eth-account pyyaml

# Build ROS 2 package
colcon build --packages-select davo_bridge

echo -e "${GREEN}✓ Bridge built successfully${NC}"

# Start RMF airport demo
echo -e "${YELLOW}Starting RMF airport demo...${NC}"

# Pull RMF demo Docker image
docker pull openrobotics/rmf_demos:latest

# Start RMF airport world in background
echo -e "${BLUE}Launching RMF airport world...${NC}"
docker run -d \
    --name rmf_airport_demo \
    --rm \
    -p 8080:8080 \
    -p 11311:11311 \
    -e ROS_DOMAIN_ID=0 \
    openrobotics/rmf_demos:latest \
    ros2 launch rmf_demos airport.launch.xml

# Wait for RMF to start
echo -e "${YELLOW}Waiting for RMF to start...${NC}"
sleep 30

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Start Davo bridge
echo -e "${BLUE}Starting Davo bridge...${NC}"
cd "$BRIDGE_DIR"
source install/setup.bash

# Set environment variables for bridge
export RPC_URL="$BASE_SEPOLIA_RPC"
export PRIVATE_KEY="$PRIVATE_KEY"
export ROBOT_PRIVATE_KEY="$ROBOT_PRIVATE_KEY"

# Start bridge node in background
ros2 launch davo_bridge bridge.launch.py &
BRIDGE_PID=$!

echo -e "${GREEN}✓ Bridge started (PID: $BRIDGE_PID)${NC}"

# Wait for bridge to initialize
sleep 10

# Generate demo task
echo -e "${BLUE}Generating demo task...${NC}"
python3 "$DEMO_DIR/generate_demo_data.py" \
    --rpc-url "$BASE_SEPOLIA_RPC" \
    --private-key "$PRIVATE_KEY" \
    --task-escrow "$TASK_ESCROW" \
    --davo-token "$DAVO_TOKEN"

echo -e "${GREEN}✓ Demo task generated${NC}"

# Monitor for task completion
echo -e "${BLUE}Monitoring task execution...${NC}"
echo "Press Ctrl+C to stop the demo"

# Function to cleanup on exit
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    
    # Stop bridge
    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null || true
    fi
    
    # Stop RMF demo
    docker stop rmf_airport_demo 2>/dev/null || true
    
    echo -e "${GREEN}✓ Demo stopped${NC}"
    exit 0
}

# Set trap for cleanup
trap cleanup SIGINT SIGTERM

# Monitor logs
echo -e "${BLUE}Demo logs:${NC}"
echo "=================================================="

# Tail bridge logs
tail -f "$BRIDGE_DIR/davo_bridge.log" &
TAIL_PID=$!

# Wait for user interrupt
wait $TAIL_PID

cleanup
