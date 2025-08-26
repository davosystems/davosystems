// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "forge-std/Test.sol";
import "../src/RobotIdentity.sol";

contract RobotIdentityTest is Test {
    RobotIdentity public robotIdentity;
    
    address public deployer = address(0x1);
    address public robot1 = address(0x2);
    address public robot2 = address(0x3);
    
    uint256 public robot1Id = 1;
    uint256 public robot2Id = 2;
    bytes32 public robot1PubkeyHash = keccak256(abi.encodePacked("robot1_public_key"));
    bytes32 public robot2PubkeyHash = keccak256(abi.encodePacked("robot2_public_key"));

    function setUp() public {
        vm.startPrank(deployer);
        robotIdentity = new RobotIdentity(deployer);
        vm.stopPrank();
    }

    function testRegisterRobot() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        // Verify robot registration
        (address evmAddress, bytes32 pubkeyHash, string memory uri) = robotIdentity.getRobotInfo(robot1Id);
        
        assertEq(evmAddress, robot1, "Robot address should match");
        assertEq(pubkeyHash, robot1PubkeyHash, "Public key hash should match");
        assertEq(uri, "ipfs://QmRobot1", "URI should match");
        
        vm.stopPrank();
    }

    function testBindKey() public {
        vm.startPrank(deployer);
        
        // Register robot first
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        // Bind new key
        bytes32 newPubkeyHash = keccak256(abi.encodePacked("new_robot1_key"));
        robotIdentity.bindKey(robot1Id, robot1, newPubkeyHash);
        
        // Verify key binding
        (,, bytes32 boundPubkeyHash) = robotIdentity.getRobotInfo(robot1Id);
        assertEq(boundPubkeyHash, newPubkeyHash, "Public key should be updated");
        
        vm.stopPrank();
    }

    function testGetRobotId() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
        
        uint256 retrievedId = robotIdentity.getRobotId(robot1);
        assertEq(retrievedId, robot1Id, "Robot ID should match");
    }

    function testIsRobotBound() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
        
        bool isBound = robotIdentity.isRobotBound(robot1Id, robot1);
        assertTrue(isBound, "Robot should be bound");
        
        bool isNotBound = robotIdentity.isRobotBound(robot1Id, robot2);
        assertFalse(isNotBound, "Wrong address should not be bound");
    }

    function testGetDomainSeparator() public {
        bytes32 domainSeparator = robotIdentity.getDomainSeparator();
        assertTrue(domainSeparator != bytes32(0), "Domain separator should not be zero");
    }

    function testDuplicateRegistration() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        // Try to register same robot ID again
        vm.expectRevert("Robot ID already exists");
        robotIdentity.registerRobot(
            robot1Id,
            robot2,
            robot2PubkeyHash,
            "ipfs://QmRobot2"
        );
        
        vm.stopPrank();
    }

    function testDuplicateAddress() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        // Try to register same address again
        vm.expectRevert("EVM address already bound");
        robotIdentity.registerRobot(
            robot2Id,
            robot1,
            robot2PubkeyHash,
            "ipfs://QmRobot2"
        );
        
        vm.stopPrank();
    }

    function testInvalidRobotId() public {
        vm.startPrank(deployer);
        
        vm.expectRevert("Robot ID must be positive");
        robotIdentity.registerRobot(
            0,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
    }

    function testInvalidAddress() public {
        vm.startPrank(deployer);
        
        vm.expectRevert("Invalid EVM address");
        robotIdentity.registerRobot(
            robot1Id,
            address(0),
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
    }

    function testBindKeyNonExistentRobot() public {
        vm.startPrank(deployer);
        
        vm.expectRevert("Robot does not exist");
        robotIdentity.bindKey(robot1Id, robot1, robot1PubkeyHash);
        
        vm.stopPrank();
    }

    function testBindKeyAddressMismatch() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.expectRevert("Robot ID and address mismatch");
        robotIdentity.bindKey(robot1Id, robot2, robot2PubkeyHash);
        
        vm.stopPrank();
    }

    function testGetRobotInfoNonExistent() public {
        vm.expectRevert("Robot does not exist");
        robotIdentity.getRobotInfo(robot1Id);
    }

    function testGetRobotIdNonExistent() public {
        vm.expectRevert("Address not bound to robot");
        robotIdentity.getRobotId(robot1);
    }

    function testOnlyOwnerRegistration() public {
        vm.startPrank(robot1);
        
        vm.expectRevert();
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
    }

    function testOnlyOwnerBindKey() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        vm.stopPrank();
        
        vm.startPrank(robot1);
        
        vm.expectRevert();
        robotIdentity.bindKey(robot1Id, robot1, robot2PubkeyHash);
        
        vm.stopPrank();
    }

    function testMultipleRobots() public {
        vm.startPrank(deployer);
        
        robotIdentity.registerRobot(
            robot1Id,
            robot1,
            robot1PubkeyHash,
            "ipfs://QmRobot1"
        );
        
        robotIdentity.registerRobot(
            robot2Id,
            robot2,
            robot2PubkeyHash,
            "ipfs://QmRobot2"
        );
        
        vm.stopPrank();
        
        // Verify both robots
        (address evmAddress1,,) = robotIdentity.getRobotInfo(robot1Id);
        (address evmAddress2,,) = robotIdentity.getRobotInfo(robot2Id);
        
        assertEq(evmAddress1, robot1, "Robot 1 address should match");
        assertEq(evmAddress2, robot2, "Robot 2 address should match");
        
        uint256 id1 = robotIdentity.getRobotId(robot1);
        uint256 id2 = robotIdentity.getRobotId(robot2);
        
        assertEq(id1, robot1Id, "Robot 1 ID should match");
        assertEq(id2, robot2Id, "Robot 2 ID should match");
    }
}
