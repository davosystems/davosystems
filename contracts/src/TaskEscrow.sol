// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/security/ReentrancyGuard.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";
import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "./DAVO.sol";
import "./RobotIdentity.sol";
import "./CommitRevealBids.sol";

/**
 * @title TaskEscrow
 * @dev Core task coordination contract for Davo Systems
 * Handles task posting, bidding, result submission, and settlement
 */
contract TaskEscrow is Ownable, ReentrancyGuard {
    using ECDSA for bytes32;

    // EIP-712 task result struct
    struct TaskResult {
        uint256 taskId;
        bytes32 resultHash;
        uint256 robotId;
        uint256 timestamp;
    }

    // Task state
    struct Task {
        uint256 taskId;
        bytes32 specHash;
        uint256 budget;
        address winner;
        uint256 winningBid;
        bytes32 resultHash;
        uint256 robotId;
        bool settled;
        uint256 feeAmount;
    }

    // Contract interfaces
    DAVO public immutable davoToken;
    RobotIdentity public immutable robotIdentity;
    CommitRevealBids public immutable biddingContract;

    // Task mapping
    mapping(uint256 => Task) public tasks;
    uint256 public nextTaskId = 1;
    uint256 public feePercentage = 250; // 2.5% (basis points)

    // Events
    event TaskPosted(uint256 indexed taskId, bytes32 specHash, uint256 budget);
    event TaskAssigned(uint256 indexed taskId, address indexed winner, uint256 bid);
    event TaskResultSubmitted(uint256 indexed taskId, bytes32 resultHash, uint256 robotId);
    event TaskSettled(uint256 indexed taskId, address indexed winner, uint256 paid);
    event FeePercentageUpdated(uint256 oldFee, uint256 newFee);

    /**
     * @dev Constructor
     * @param initialOwner Initial contract owner
     * @param _davoToken DAVO token contract address
     * @param _robotIdentity Robot identity contract address
     * @param _biddingContract Bidding contract address
     */
    constructor(
        address initialOwner,
        address _davoToken,
        address _robotIdentity,
        address _biddingContract
    ) Ownable(initialOwner) {
        davoToken = DAVO(_davoToken);
        robotIdentity = RobotIdentity(_robotIdentity);
        biddingContract = CommitRevealBids(_biddingContract);
    }

    /**
     * @dev Post a new task with budget escrow
     * @param specHash Hash of task specification
     * @param budget Budget in DAVO tokens
     * @return taskId New task identifier
     */
    function postTask(bytes32 specHash, uint256 budget) 
        external 
        nonReentrant 
        returns (uint256 taskId) 
    {
        require(specHash != bytes32(0), "Invalid spec hash");
        require(budget > 0, "Budget must be positive");

        taskId = nextTaskId++;
        
        // Transfer budget to escrow
        require(
            davoToken.transferFrom(msg.sender, address(this), budget),
            "Transfer failed"
        );

        tasks[taskId] = Task({
            taskId: taskId,
            specHash: specHash,
            budget: budget,
            winner: address(0),
            winningBid: 0,
            resultHash: bytes32(0),
            robotId: 0,
            settled: false,
            feeAmount: 0
        });

        // Start bidding
        biddingContract.startBidding(taskId);

        emit TaskPosted(taskId, specHash, budget);
    }

    /**
     * @dev Assign task winner (called after bidding)
     * @param taskId Task identifier
     */
    function assignWinner(uint256 taskId) external onlyOwner {
        Task storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(task.winner == address(0), "Task already assigned");

        // Get winner from bidding contract
        (,, uint256 lowestBid, address lowestBidder, bool assigned) = 
            biddingContract.getTaskBidding(taskId);
        
        require(assigned, "Bidding not complete");
        require(lowestBidder != address(0), "No valid bids");

        task.winner = lowestBidder;
        task.winningBid = lowestBid;
        task.feeAmount = (task.budget * feePercentage) / 10000;

        emit TaskAssigned(taskId, lowestBidder, lowestBid);
    }

    /**
     * @dev Submit task result with EIP-712 signature
     * @param taskId Task identifier
     * @param resultHash Hash of task result/telemetry
     * @param robotId Robot identifier
     * @param signature EIP-712 signature
     */
    function submitResult(
        uint256 taskId,
        bytes32 resultHash,
        uint256 robotId,
        bytes calldata signature
    ) external {
        Task storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(task.winner != address(0), "Task not assigned");
        require(task.resultHash == bytes32(0), "Result already submitted");
        require(resultHash != bytes32(0), "Invalid result hash");

        // Verify robot identity
        (address robotAddress,,) = robotIdentity.getRobotInfo(robotId);
        require(robotAddress != address(0), "Robot not found");

        // Verify EIP-712 signature
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

        address signer = digest.recover(signature);
        require(signer == robotAddress, "Invalid signature");

        task.resultHash = resultHash;
        task.robotId = robotId;

        emit TaskResultSubmitted(taskId, resultHash, robotId);
    }

    /**
     * @dev Settle task and pay winner
     * @param taskId Task identifier
     */
    function settle(uint256 taskId) external nonReentrant {
        Task storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(task.winner != address(0), "Task not assigned");
        require(task.resultHash != bytes32(0), "Result not submitted");
        require(!task.settled, "Task already settled");

        task.settled = true;

        // Calculate payment (budget - fee)
        uint256 payment = task.budget - task.feeAmount;

        // Transfer payment to winner
        require(
            davoToken.transfer(task.winner, payment),
            "Winner payment failed"
        );

        // Transfer fee to fee router (will be called by external contract)
        if (task.feeAmount > 0) {
            require(
                davoToken.transfer(owner(), task.feeAmount),
                "Fee transfer failed"
            );
        }

        emit TaskSettled(taskId, task.winner, payment);
    }

    /**
     * @dev Get task information
     * @param taskId Task identifier
     * @return specHash Task specification hash
     * @return budget Task budget
     * @return winner Task winner address
     * @return winningBid Winning bid amount
     * @return resultHash Task result hash
     * @return robotId Robot identifier
     * @return settled Whether task is settled
     * @return feeAmount Fee amount
     */
    function getTask(uint256 taskId) 
        external 
        view 
        returns (
            bytes32 specHash,
            uint256 budget,
            address winner,
            uint256 winningBid,
            bytes32 resultHash,
            uint256 robotId,
            bool settled,
            uint256 feeAmount
        ) 
    {
        Task storage task = tasks[taskId];
        return (
            task.specHash,
            task.budget,
            task.winner,
            task.winningBid,
            task.resultHash,
            task.robotId,
            task.settled,
            task.feeAmount
        );
    }

    /**
     * @dev Update fee percentage (owner only)
     * @param newFeePercentage New fee percentage in basis points
     */
    function updateFeePercentage(uint256 newFeePercentage) external onlyOwner {
        require(newFeePercentage <= 1000, "Fee too high"); // Max 10%
        uint256 oldFee = feePercentage;
        feePercentage = newFeePercentage;
        emit FeePercentageUpdated(oldFee, newFeePercentage);
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
     * @dev Check if task is ready for settlement
     * @param taskId Task identifier
     * @return True if ready for settlement
     */
    function isReadyForSettlement(uint256 taskId) external view returns (bool) {
        Task storage task = tasks[taskId];
        return task.taskId != 0 && 
               task.winner != address(0) && 
               task.resultHash != bytes32(0) && 
               !task.settled;
    }

    /**
     * @dev Get total escrowed amount
     * @return Total DAVO tokens in escrow
     */
    function getEscrowedAmount() external view returns (uint256) {
        return davoToken.balanceOf(address(this));
    }
}
