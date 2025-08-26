// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "forge-std/Test.sol";
import "../src/DAVO.sol";
import "../src/RobotIdentity.sol";
import "../src/CommitRevealBids.sol";
import "../src/BuybackBurner.sol";
import "../src/FeeRouter.sol";
import "../src/TaskEscrow.sol";
import "../src/TelemetryAttestor.sol";

/**
 * @title TaskFlowTest
 * @dev Comprehensive tests for the complete task flow
 */
contract TaskFlowTest is Test {
    // Contracts
    DAVO public davoToken;
    RobotIdentity public robotIdentity;
    CommitRevealBids public commitRevealBids;
    BuybackBurner public buybackBurner;
    FeeRouter public feeRouter;
    TaskEscrow public taskEscrow;
    TelemetryAttestor public telemetryAttestor;

    // Test addresses
    address public deployer = address(0x1);
    address public orchestrator = address(0x2);
    address public robot1 = address(0x3);
    address public robot2 = address(0x4);
    address public treasury = address(0x5);

    // Test data
    uint256 public robot1Id = 1;
    uint256 public robot2Id = 2;
    bytes32 public taskSpecHash = keccak256(abi.encodePacked("test_task_spec"));
    uint256 public taskBudget = 1000 * 10**18; // 1000 DAVO
    bytes32 public resultHash = keccak256(abi.encodePacked("task_result"));

    function setUp() public {
        vm.startPrank(deployer);

        // Deploy contracts
        davoToken = new DAVO(deployer);
        robotIdentity = new RobotIdentity(deployer);
        commitRevealBids = new CommitRevealBids(deployer);
        buybackBurner = new BuybackBurner(deployer, address(davoToken));
        feeRouter = new FeeRouter(deployer, address(davoToken), address(buybackBurner), treasury);
        taskEscrow = new TaskEscrow(deployer, address(davoToken), address(robotIdentity), address(commitRevealBids));
        telemetryAttestor = new TelemetryAttestor(deployer);

        // Set up initial configuration
        taskEscrow.transferOwnership(address(feeRouter));

        // Register robots
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            keccak256(abi.encodePacked("robot1_public_key")),
            "ipfs://QmRobot1"
        );

        robotIdentity.registerRobot(
            robot2Id,
            robot2,
            keccak256(abi.encodePacked("robot2_public_key")),
            "ipfs://QmRobot2"
        );

        // Transfer tokens to orchestrator
        davoToken.transfer(orchestrator, taskBudget * 10);

        vm.stopPrank();
    }

    function testCompleteTaskFlow() public {
        // 1. Post task
        vm.startPrank(orchestrator);
        davoToken.approve(address(taskEscrow), taskBudget);
        uint256 taskId = taskEscrow.postTask(taskSpecHash, taskBudget);
        vm.stopPrank();

        assertEq(taskId, 1, "Task ID should be 1");
        assertEq(davoToken.balanceOf(address(taskEscrow)), taskBudget, "TaskEscrow should have budget");

        // 2. Commit bids
        vm.startPrank(robot1);
        bytes32 commitment1 = keccak256(abi.encodePacked(800 * 10**18, bytes32("salt1"), robot1));
        commitRevealBids.commitBid(taskId, commitment1);
        vm.stopPrank();

        vm.startPrank(robot2);
        bytes32 commitment2 = keccak256(abi.encodePacked(900 * 10**18, bytes32("salt2"), robot2));
        commitRevealBids.commitBid(taskId, commitment2);
        vm.stopPrank();

        // 3. Reveal bids
        vm.warp(block.timestamp + 301); // After commit window

        vm.startPrank(robot1);
        commitRevealBids.revealBid(taskId, 800 * 10**18, bytes32("salt1"));
        vm.stopPrank();

        vm.startPrank(robot2);
        commitRevealBids.revealBid(taskId, 900 * 10**18, bytes32("salt2"));
        vm.stopPrank();

        // 4. Assign winner
        vm.warp(block.timestamp + 301); // After reveal window

        vm.prank(deployer);
        taskEscrow.assignWinner(taskId);

        // 5. Submit result
        bytes memory signature = _createTaskResultSignature(taskId, resultHash, robot1Id, robot1);
        
        taskEscrow.submitResult(taskId, resultHash, robot1Id, signature);

        // 6. Settle task
        taskEscrow.settle(taskId);

        // Verify final state
        (,, address winner, uint256 winningBid, bytes32 submittedResultHash, uint256 submittedRobotId, bool settled,) = taskEscrow.getTask(taskId);
        
        assertEq(winner, robot1, "Robot1 should be winner");
        assertEq(winningBid, 800 * 10**18, "Winning bid should be 800 DAVO");
        assertEq(submittedResultHash, resultHash, "Result hash should match");
        assertEq(submittedRobotId, robot1Id, "Robot ID should match");
        assertTrue(settled, "Task should be settled");

        // Verify payments
        uint256 feeAmount = (taskBudget * 250) / 10000; // 2.5% fee
        uint256 payment = taskBudget - feeAmount;
        
        assertEq(davoToken.balanceOf(robot1), payment, "Robot1 should receive payment");
        assertEq(davoToken.balanceOf(treasury), feeAmount, "Treasury should receive fee");
    }

    function testBiddingWindows() public {
        // Post task
        vm.startPrank(orchestrator);
        davoToken.approve(address(taskEscrow), taskBudget);
        uint256 taskId = taskEscrow.postTask(taskSpecHash, taskBudget);
        vm.stopPrank();

        // Try to commit after window closes
        vm.warp(block.timestamp + 301);
        
        vm.startPrank(robot1);
        bytes32 commitment = keccak256(abi.encodePacked(800 * 10**18, bytes32("salt"), robot1));
        vm.expectRevert("Commit window closed");
        commitRevealBids.commitBid(taskId, commitment);
        vm.stopPrank();

        // Try to reveal before window opens
        vm.warp(block.timestamp - 1);
        
        vm.startPrank(robot1);
        vm.expectRevert("Commit window not closed");
        commitRevealBids.revealBid(taskId, 800 * 10**18, bytes32("salt"));
        vm.stopPrank();
    }

    function testInvalidSignature() public {
        // Post task and assign winner
        vm.startPrank(orchestrator);
        davoToken.approve(address(taskEscrow), taskBudget);
        uint256 taskId = taskEscrow.postTask(taskSpecHash, taskBudget);
        vm.stopPrank();

        vm.prank(deployer);
        taskEscrow.assignWinner(taskId);

        // Try to submit result with invalid signature
        bytes memory invalidSignature = new bytes(65);
        
        vm.expectRevert("Invalid signature");
        taskEscrow.submitResult(taskId, resultHash, robot1Id, invalidSignature);
    }

    function testDoubleSettlement() public {
        // Complete task flow
        _completeTaskFlow();

        // Try to settle again
        vm.expectRevert("Task already settled");
        taskEscrow.settle(1);
    }

    function testFeeDistribution() public {
        // Complete task flow
        _completeTaskFlow();

        // Check fee distribution
        uint256 feeAmount = (taskBudget * 250) / 10000; // 2.5% fee
        uint256 treasuryAmount = (feeAmount * 5000) / 10000; // 50% to treasury
        uint256 burnAmount = feeAmount - treasuryAmount; // 50% to burn

        assertEq(davoToken.balanceOf(treasury), treasuryAmount, "Treasury should receive 50% of fees");
        
        (uint256 totalBurned,) = buybackBurner.getBurnStats();
        assertEq(totalBurned, burnAmount, "Burn amount should match");
    }

    function testEmergencyWithdraw() public {
        // Transfer tokens to contracts
        vm.startPrank(deployer);
        davoToken.transfer(address(taskEscrow), 1000 * 10**18);
        davoToken.transfer(address(feeRouter), 1000 * 10**18);
        vm.stopPrank();

        // Emergency withdraw
        vm.startPrank(deployer);
        taskEscrow.emergencyWithdraw(address(davoToken), 500 * 10**18);
        feeRouter.emergencyWithdraw(address(davoToken), 500 * 10**18);
        vm.stopPrank();

        assertEq(davoToken.balanceOf(deployer), davoToken.TOTAL_SUPPLY() - 1000 * 10**18 + 1000 * 10**18, "Deployer should have withdrawn tokens");
    }

    function testTelemetryAttestor() public {
        // Add attestor
        vm.prank(deployer);
        telemetryAttestor.addAttestor(robot1);

        // Create attestation
        bytes memory signature = _createAttestationSignature(1, resultHash, robot1Id, robot1);
        
        vm.prank(robot1);
        telemetryAttestor.createAttestation(1, resultHash, robot1Id, signature);

        // Verify attestation
        (bool isAttestor, uint256 reputation) = telemetryAttestor.getAttestorInfo(robot1);
        assertTrue(isAttestor, "Robot1 should be attestor");
        assertEq(reputation, 100, "Initial reputation should be 100");
    }

    // Helper functions
    function _completeTaskFlow() internal {
        // Post task
        vm.startPrank(orchestrator);
        davoToken.approve(address(taskEscrow), taskBudget);
        uint256 taskId = taskEscrow.postTask(taskSpecHash, taskBudget);
        vm.stopPrank();

        // Commit and reveal bids
        vm.startPrank(robot1);
        bytes32 commitment = keccak256(abi.encodePacked(800 * 10**18, bytes32("salt"), robot1));
        commitRevealBids.commitBid(taskId, commitment);
        vm.stopPrank();

        vm.warp(block.timestamp + 301);
        
        vm.startPrank(robot1);
        commitRevealBids.revealBid(taskId, 800 * 10**18, bytes32("salt"));
        vm.stopPrank();

        vm.warp(block.timestamp + 301);

        // Assign winner
        vm.prank(deployer);
        taskEscrow.assignWinner(taskId);

        // Submit result
        bytes memory signature = _createTaskResultSignature(taskId, resultHash, robot1Id, robot1);
        taskEscrow.submitResult(taskId, resultHash, robot1Id, signature);

        // Settle
        taskEscrow.settle(taskId);
    }

    function _createTaskResultSignature(uint256 taskId, bytes32 resultHash, uint256 robotId, address signer) internal returns (bytes memory) {
        // Create EIP-712 signature
        bytes32 structHash = keccak256(
            abi.encode(
                keccak256("TaskResult(uint256 taskId,bytes32 resultHash,uint256 robotId,uint256 timestamp)"),
                taskId,
                resultHash,
                robotId,
                block.timestamp
            )
        );

        bytes32 domainSeparator = robotIdentity.getDomainSeparator();
        bytes32 digest = keccak256(
            abi.encodePacked("\x19\x01", domainSeparator, structHash)
        );

        (uint8 v, bytes32 r, bytes32 s) = vm.sign(uint256(uint160(signer)), digest);
        return abi.encodePacked(r, s, v);
    }

    function _createAttestationSignature(uint256 taskId, bytes32 resultHash, uint256 robotId, address signer) internal returns (bytes memory) {
        bytes32 attestationHash = keccak256(
            abi.encodePacked(taskId, resultHash, robotId, block.timestamp, signer)
        );

        bytes32 digest = keccak256(
            abi.encodePacked("\x19Ethereum Signed Message:\n32", attestationHash)
        );

        (uint8 v, bytes32 r, bytes32 s) = vm.sign(uint256(uint160(signer)), digest);
        return abi.encodePacked(r, s, v);
    }
}
