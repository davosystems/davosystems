// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "./DAVO.sol";
import "./BuybackBurner.sol";

/**
 * @title FeeRouter
 * @dev Fee distribution contract for Davo Systems
 * Routes fees 50% to treasury, 50% to buyback+burn
 */
contract FeeRouter is Ownable, ReentrancyGuard {
    // Contract interfaces
    DAVO public immutable davoToken;
    BuybackBurner public immutable buybackBurner;

    // Fee distribution parameters
    uint256 public treasurySplit = 5000; // 50% (basis points)
    uint256 public burnSplit = 5000; // 50% (basis points)
    uint256 public constant BASIS_POINTS = 10000;

    // Treasury address
    address public treasury;

    // Timelock for parameter changes
    uint256 public constant TIMELOCK_DURATION = 48 hours;
    mapping(bytes32 => uint256) public pendingChanges;
    mapping(bytes32 => bool) public executedChanges;

    // Events
    event FeesRouted(uint256 amountToTreasury, uint256 amountToBurn);
    event TreasuryUpdated(address oldTreasury, address newTreasury);
    event FeeSplitsUpdated(uint256 newTreasurySplit, uint256 newBurnSplit);
    event TimelockSet(bytes32 indexed changeId, uint256 executeAfter);

    /**
     * @dev Constructor
     * @param initialOwner Initial contract owner
     * @param _davoToken DAVO token contract address
     * @param _buybackBurner Buyback burner contract address
     * @param _treasury Treasury address
     */
    constructor(
        address initialOwner,
        address _davoToken,
        address _buybackBurner,
        address _treasury
    ) Ownable(initialOwner) {
        davoToken = DAVO(_davoToken);
        buybackBurner = BuybackBurner(_buybackBurner);
        treasury = _treasury;
    }

    /**
     * @dev Route fees from TaskEscrow
     * @param amount Total fee amount to route
     */
    function routeFees(uint256 amount) external nonReentrant {
        require(amount > 0, "Amount must be positive");
        require(
            davoToken.transferFrom(msg.sender, address(this), amount),
            "Transfer failed"
        );

        // Calculate splits
        uint256 treasuryAmount = (amount * treasurySplit) / BASIS_POINTS;
        uint256 burnAmount = amount - treasuryAmount; // Remainder to avoid dust

        // Transfer to treasury
        if (treasuryAmount > 0) {
            require(
                davoToken.transfer(treasury, treasuryAmount),
                "Treasury transfer failed"
            );
        }

        // Transfer to buyback burner
        if (burnAmount > 0) {
            require(
                davoToken.transfer(address(buybackBurner), burnAmount),
                "Burner transfer failed"
            );
            
            // Trigger buyback and burn
            buybackBurner.burnTokens(burnAmount);
        }

        emit FeesRouted(treasuryAmount, burnAmount);
    }

    /**
     * @dev Set new treasury address (with timelock)
     * @param newTreasury New treasury address
     */
    function setTreasury(address newTreasury) external onlyOwner {
        require(newTreasury != address(0), "Invalid treasury address");
        
        bytes32 changeId = keccak256(abi.encodePacked("TREASURY", newTreasury));
        require(!executedChanges[changeId], "Change already executed");
        
        if (pendingChanges[changeId] == 0) {
            // Set timelock
            pendingChanges[changeId] = block.timestamp + TIMELOCK_DURATION;
            emit TimelockSet(changeId, pendingChanges[changeId]);
        } else {
            // Execute if timelock expired
            require(block.timestamp >= pendingChanges[changeId], "Timelock not expired");
            
            address oldTreasury = treasury;
            treasury = newTreasury;
            executedChanges[changeId] = true;
            
            emit TreasuryUpdated(oldTreasury, newTreasury);
        }
    }

    /**
     * @dev Set new fee splits (with timelock)
     * @param newTreasurySplit New treasury split (basis points)
     * @param newBurnSplit New burn split (basis points)
     */
    function setFeeSplits(uint256 newTreasurySplit, uint256 newBurnSplit) external onlyOwner {
        require(newTreasurySplit + newBurnSplit == BASIS_POINTS, "Splits must equal 100%");
        require(newTreasurySplit > 0, "Treasury split must be positive");
        require(newBurnSplit > 0, "Burn split must be positive");
        
        bytes32 changeId = keccak256(abi.encodePacked("FEE_SPLITS", newTreasurySplit, newBurnSplit));
        require(!executedChanges[changeId], "Change already executed");
        
        if (pendingChanges[changeId] == 0) {
            // Set timelock
            pendingChanges[changeId] = block.timestamp + TIMELOCK_DURATION;
            emit TimelockSet(changeId, pendingChanges[changeId]);
        } else {
            // Execute if timelock expired
            require(block.timestamp >= pendingChanges[changeId], "Timelock not expired");
            
            treasurySplit = newTreasurySplit;
            burnSplit = newBurnSplit;
            executedChanges[changeId] = true;
            
            emit FeeSplitsUpdated(newTreasurySplit, newBurnSplit);
        }
    }

    /**
     * @dev Cancel a pending change
     * @param changeId Change identifier
     */
    function cancelChange(bytes32 changeId) external onlyOwner {
        require(pendingChanges[changeId] > 0, "No pending change");
        require(!executedChanges[changeId], "Change already executed");
        
        delete pendingChanges[changeId];
    }

    /**
     * @dev Get pending change information
     * @param changeId Change identifier
     * @return executeAfter Timestamp when change can be executed
     * @return canExecute Whether change can be executed now
     */
    function getPendingChange(bytes32 changeId) 
        external 
        view 
        returns (uint256 executeAfter, bool canExecute) 
    {
        executeAfter = pendingChanges[changeId];
        canExecute = executeAfter > 0 && block.timestamp >= executeAfter && !executedChanges[changeId];
    }

    /**
     * @dev Emergency withdraw tokens (owner only)
     * @param token Token address to withdraw
     * @param amount Amount to withdraw
     */
    function emergencyWithdraw(address token, uint256 amount) external onlyOwner {
        require(IERC20(token).transfer(owner(), amount), "Withdraw failed");
    }

    /**
     * @dev Get current fee distribution parameters
     * @return treasurySplit Current treasury split
     * @return burnSplit Current burn split
     * @return treasury Current treasury address
     */
    function getFeeParameters() 
        external 
        view 
        returns (uint256, uint256, address) 
    {
        return (treasurySplit, burnSplit, treasury);
    }

    /**
     * @dev Calculate fee splits for a given amount
     * @param amount Total amount to split
     * @return treasuryAmount Amount for treasury
     * @return burnAmount Amount for burn
     */
    function calculateSplits(uint256 amount) 
        external 
        view 
        returns (uint256 treasuryAmount, uint256 burnAmount) 
    {
        treasuryAmount = (amount * treasurySplit) / BASIS_POINTS;
        burnAmount = amount - treasuryAmount;
    }
}
