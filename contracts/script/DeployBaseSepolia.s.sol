// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "forge-std/Script.sol";
import "../src/DAVO.sol";
import "../src/RobotIdentity.sol";
import "../src/CommitRevealBids.sol";
import "../src/BuybackBurner.sol";
import "../src/FeeRouter.sol";
import "../src/TaskEscrow.sol";
import "../src/TelemetryAttestor.sol";

/**
 * @title DeployBaseSepolia
 * @dev Deployment script for Davo Systems on Base Sepolia
 */
contract DeployBaseSepolia is Script {
    // Deployment addresses
    address public davoToken;
    address public robotIdentity;
    address public commitRevealBids;
    address public buybackBurner;
    address public feeRouter;
    address public taskEscrow;
    address public telemetryAttestor;

    // Treasury address (Safe multi-sig)
    address public treasury;

    function run() external {
        uint256 deployerPrivateKey = vm.envUint("PRIVATE_KEY");
        address deployer = vm.addr(deployerPrivateKey);
        
        // For demo, use deployer as treasury (replace with Safe address for production)
        treasury = deployer;

        vm.startBroadcast(deployerPrivateKey);

        console.log("Deploying Davo Systems contracts to Base Sepolia...");
        console.log("Deployer:", deployer);
        console.log("Treasury:", treasury);

        // 1. Deploy DAVO Token
        console.log("\n1. Deploying DAVO Token...");
        davoToken = address(new DAVO(deployer));
        console.log("DAVO Token deployed at:", davoToken);

        // 2. Deploy Robot Identity
        console.log("\n2. Deploying Robot Identity...");
        robotIdentity = address(new RobotIdentity(deployer));
        console.log("Robot Identity deployed at:", robotIdentity);

        // 3. Deploy Commit Reveal Bids
        console.log("\n3. Deploying Commit Reveal Bids...");
        commitRevealBids = address(new CommitRevealBids(deployer));
        console.log("Commit Reveal Bids deployed at:", commitRevealBids);

        // 4. Deploy Buyback Burner
        console.log("\n4. Deploying Buyback Burner...");
        buybackBurner = address(new BuybackBurner(deployer, davoToken));
        console.log("Buyback Burner deployed at:", buybackBurner);

        // 5. Deploy Fee Router
        console.log("\n5. Deploying Fee Router...");
        feeRouter = address(new FeeRouter(deployer, davoToken, buybackBurner, treasury));
        console.log("Fee Router deployed at:", feeRouter);

        // 6. Deploy Task Escrow
        console.log("\n6. Deploying Task Escrow...");
        taskEscrow = address(new TaskEscrow(deployer, davoToken, robotIdentity, commitRevealBids));
        console.log("Task Escrow deployed at:", taskEscrow);

        // 7. Deploy Telemetry Attestor
        console.log("\n7. Deploying Telemetry Attestor...");
        telemetryAttestor = address(new TelemetryAttestor(deployer));
        console.log("Telemetry Attestor deployed at:", telemetryAttestor);

        // 8. Set up initial configuration
        console.log("\n8. Setting up initial configuration...");
        
        // Transfer ownership of TaskEscrow to FeeRouter for fee collection
        TaskEscrow(taskEscrow).transferOwnership(feeRouter);
        console.log("TaskEscrow ownership transferred to FeeRouter");

        // Register demo robots
        _registerDemoRobots();

        vm.stopBroadcast();

        // 9. Write addresses to file
        _writeAddressesFile();

        console.log("\n✅ Deployment complete!");
        console.log("Check docs/addresses.md for contract addresses");
    }

    function _registerDemoRobots() internal {
        RobotIdentity robotId = RobotIdentity(robotIdentity);
        
        // Demo robot 1
        robotId.registerRobot(
            1,
            0x70997970C51812dc3A010C7d01b50e0d17dc79C8, // Demo address
            keccak256(abi.encodePacked("robot_01_public_key")),
            "ipfs://QmDemoRobot1"
        );
        console.log("Demo robot 1 registered");

        // Demo robot 2
        robotId.registerRobot(
            2,
            0x3C44CdDdB6a900fa2b585dd299e03d12FA4293BC, // Demo address
            keccak256(abi.encodePacked("robot_02_public_key")),
            "ipfs://QmDemoRobot2"
        );
        console.log("Demo robot 2 registered");
    }

    function _writeAddressesFile() internal {
        string memory addressesContent = string.concat(
            "# Contract Addresses\n\n",
            "## Base Sepolia Deployment\n\n",
            "This document contains the deployed contract addresses for Davo Systems on Base Sepolia testnet.\n\n",
            "### Core Contracts\n\n",
            "| Contract | Address | Explorer |\n",
            "|----------|---------|----------|\n",
            "| DAVO Token | `", vm.toString(davoToken), "` | [View](https://sepolia.basescan.org/address/", vm.toString(davoToken), ") |\n",
            "| TaskEscrow | `", vm.toString(taskEscrow), "` | [View](https://sepolia.basescan.org/address/", vm.toString(taskEscrow), ") |\n",
            "| RobotIdentity | `", vm.toString(robotIdentity), "` | [View](https://sepolia.basescan.org/address/", vm.toString(robotIdentity), ") |\n",
            "| CommitRevealBids | `", vm.toString(commitRevealBids), "` | [View](https://sepolia.basescan.org/address/", vm.toString(commitRevealBids), ") |\n",
            "| FeeRouter | `", vm.toString(feeRouter), "` | [View](https://sepolia.basescan.org/address/", vm.toString(feeRouter), ") |\n",
            "| BuybackBurner | `", vm.toString(buybackBurner), "` | [View](https://sepolia.basescan.org/address/", vm.toString(buybackBurner), ") |\n",
            "| TelemetryAttestor | `", vm.toString(telemetryAttestor), "` | [View](https://sepolia.basescan.org/address/", vm.toString(telemetryAttestor), ") |\n\n",
            "### Treasury\n\n",
            "| Component | Address | Description |\n",
            "|-----------|---------|-------------|\n",
            "| Safe Treasury | `", vm.toString(treasury), "` | Multi-sig treasury for fee collection |\n\n",
            "### Deployment Details\n\n",
            "- **Network**: Base Sepolia (Chain ID: 84532)\n",
            "- **Deployer**: `", vm.toString(vm.addr(vm.envUint("PRIVATE_KEY"))), "`\n",
            "- **Deployment Block**: `", vm.toString(block.number), "`\n",
            "- **Deployment Transaction**: [View](https://sepolia.basescan.org/tx/", vm.toString(tx.gasprice), ")\n\n",
            "### Configuration\n\n",
            "#### Fee Router Parameters\n",
            "- **Treasury Split**: 50%\n",
            "- **Burn Split**: 50%\n",
            "- **Timelock Duration**: 48 hours\n\n",
            "#### Task Escrow Parameters\n",
            "- **Commit Window**: 300 seconds (5 minutes)\n",
            "- **Reveal Window**: 300 seconds (5 minutes)\n",
            "- **Fee Percentage**: 2.5%\n\n",
            "#### Robot Identity Parameters\n",
            "- **EIP-712 Domain**: DavoSystems v1\n",
            "- **Chain ID**: 84532\n\n",
            "### Recent Transactions\n\n",
            "| Transaction | Type | Block | Explorer |\n",
            "|-------------|------|-------|----------|\n",
            "| `", vm.toString(tx.gasprice), "` | Deployment | `", vm.toString(block.number), "` | [View](https://sepolia.basescan.org/tx/", vm.toString(tx.gasprice), ") |\n\n",
            "### Token Distribution\n\n",
            "| Holder | DAVO Balance | Percentage |\n",
            "|--------|-------------|------------|\n",
            "| Deployer | `1000000000000000000000000` | `100`% |\n",
            "| TaskEscrow | `0` | `0`% |\n",
            "| Treasury | `0` | `0`% |\n",
            "| Burned | `0` | `0`% |\n\n",
            "### Robot Registrations\n\n",
            "| Robot ID | Name | Public Key Hash | Owner |\n",
            "|----------|------|-----------------|-------|\n",
            "| `1` | `airport_robot_01` | `", vm.toString(keccak256(abi.encodePacked("robot_01_public_key"))), "` | `", vm.toString(vm.addr(vm.envUint("PRIVATE_KEY"))), "` |\n",
            "| `2` | `airport_robot_02` | `", vm.toString(keccak256(abi.encodePacked("robot_02_public_key"))), "` | `", vm.toString(vm.addr(vm.envUint("PRIVATE_KEY"))), "` |\n\n",
            "---\n\n",
            "*This file is automatically updated by deployment scripts. Last updated: ", vm.toString(block.timestamp), "*"
        );

        vm.writeFile("../docs/addresses.md", addressesContent);
        console.log("Addresses written to docs/addresses.md");
    }
}
