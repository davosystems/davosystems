// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "forge-std/Test.sol";
import "../src/DAVO.sol";
import "../src/BuybackBurner.sol";
import "../src/FeeRouter.sol";

contract FeeRouterTest is Test {
    DAVO public davoToken;
    BuybackBurner public buybackBurner;
    FeeRouter public feeRouter;
    
    address public deployer = address(0x1);
    address public treasury = address(0x2);
    address public user = address(0x3);
    
    uint256 public constant FEE_AMOUNT = 1000 * 10**18; // 1000 DAVO

    function setUp() public {
        vm.startPrank(deployer);
        
        davoToken = new DAVO(deployer);
        buybackBurner = new BuybackBurner(deployer, address(davoToken));
        feeRouter = new FeeRouter(deployer, address(davoToken), address(buybackBurner), treasury);
        
        // Transfer tokens to user
        davoToken.transfer(user, FEE_AMOUNT * 10);
        
        vm.stopPrank();
    }

    function testRouteFees() public {
        vm.startPrank(user);
        
        // Approve tokens for fee router
        davoToken.approve(address(feeRouter), FEE_AMOUNT);
        
        uint256 initialTreasuryBalance = davoToken.balanceOf(treasury);
        uint256 initialBurnerBalance = davoToken.balanceOf(address(buybackBurner));
        
        // Route fees
        feeRouter.routeFees(FEE_AMOUNT);
        
        // Check treasury received 50%
        assertEq(
            davoToken.balanceOf(treasury) - initialTreasuryBalance,
            FEE_AMOUNT / 2,
            "Treasury should receive 50%"
        );
        
        // Check burner received 50%
        assertEq(
            davoToken.balanceOf(address(buybackBurner)) - initialBurnerBalance,
            FEE_AMOUNT / 2,
            "Burner should receive 50%"
        );
        
        vm.stopPrank();
    }

    function testSetTreasury() public {
        address newTreasury = address(0x4);
        
        vm.startPrank(deployer);
        
        // Set new treasury (first call sets timelock)
        feeRouter.setTreasury(newTreasury);
        
        // Try to execute before timelock expires
        vm.expectRevert("Timelock not expired");
        feeRouter.setTreasury(newTreasury);
        
        // Wait for timelock
        vm.warp(block.timestamp + 48 hours + 1);
        
        // Execute treasury change
        feeRouter.setTreasury(newTreasury);
        
        assertEq(feeRouter.treasury(), newTreasury, "Treasury should be updated");
        
        vm.stopPrank();
    }

    function testSetFeeSplits() public {
        vm.startPrank(deployer);
        
        // Set new fee splits (first call sets timelock)
        feeRouter.setFeeSplits(6000, 4000); // 60% treasury, 40% burn
        
        // Try to execute before timelock expires
        vm.expectRevert("Timelock not expired");
        feeRouter.setFeeSplits(6000, 4000);
        
        // Wait for timelock
        vm.warp(block.timestamp + 48 hours + 1);
        
        // Execute fee split change
        feeRouter.setFeeSplits(6000, 4000);
        
        assertEq(feeRouter.treasurySplit(), 6000, "Treasury split should be updated");
        assertEq(feeRouter.burnSplit(), 4000, "Burn split should be updated");
        
        vm.stopPrank();
    }

    function testInvalidFeeSplits() public {
        vm.startPrank(deployer);
        
        // Try to set splits that don't equal 100%
        vm.expectRevert("Splits must equal 100%");
        feeRouter.setFeeSplits(6000, 3000);
        
        // Try to set zero splits
        vm.expectRevert("Treasury split must be positive");
        feeRouter.setFeeSplits(0, 10000);
        
        vm.expectRevert("Burn split must be positive");
        feeRouter.setFeeSplits(10000, 0);
        
        vm.stopPrank();
    }

    function testCancelChange() public {
        address newTreasury = address(0x4);
        
        vm.startPrank(deployer);
        
        // Set timelock
        feeRouter.setTreasury(newTreasury);
        
        // Cancel change
        bytes32 changeId = keccak256(abi.encodePacked("TREASURY", newTreasury));
        feeRouter.cancelChange(changeId);
        
        // Try to execute cancelled change
        vm.warp(block.timestamp + 48 hours + 1);
        vm.expectRevert("No pending change");
        feeRouter.setTreasury(newTreasury);
        
        vm.stopPrank();
    }

    function testCalculateSplits() public {
        uint256 amount = 1000 * 10**18;
        
        (uint256 treasuryAmount, uint256 burnAmount) = feeRouter.calculateSplits(amount);
        
        assertEq(treasuryAmount, amount / 2, "Treasury amount should be 50%");
        assertEq(burnAmount, amount / 2, "Burn amount should be 50%");
        assertEq(treasuryAmount + burnAmount, amount, "Total should equal input amount");
    }

    function testEmergencyWithdraw() public {
        vm.startPrank(deployer);
        
        // Transfer tokens to fee router
        davoToken.transfer(address(feeRouter), FEE_AMOUNT);
        
        uint256 initialBalance = davoToken.balanceOf(deployer);
        
        // Emergency withdraw
        feeRouter.emergencyWithdraw(address(davoToken), FEE_AMOUNT);
        
        assertEq(
            davoToken.balanceOf(deployer) - initialBalance,
            FEE_AMOUNT,
            "Deployer should receive withdrawn tokens"
        );
        
        vm.stopPrank();
    }

    function testOnlyOwnerFunctions() public {
        vm.startPrank(user);
        
        // Try to call owner-only functions
        vm.expectRevert();
        feeRouter.setTreasury(address(0x4));
        
        vm.expectRevert();
        feeRouter.setFeeSplits(6000, 4000);
        
        vm.expectRevert();
        feeRouter.cancelChange(bytes32(0));
        
        vm.expectRevert();
        feeRouter.emergencyWithdraw(address(davoToken), 1000);
        
        vm.stopPrank();
    }
}
