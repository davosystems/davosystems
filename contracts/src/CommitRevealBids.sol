// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";

/**
 * @title CommitRevealBids
 * @dev Sealed-bid auction system for task assignment
 * Two-phase bidding: commit (hash) then reveal (price + salt)
 */
contract CommitRevealBids is Ownable {
    using ECDSA for bytes32;

    // Bidding windows (configurable)
    uint256 public commitWindow = 300; // 5 minutes
    uint256 public revealWindow = 300; // 5 minutes

    // Task bidding state
    struct TaskBidding {
        uint256 taskId;
        uint256 commitDeadline;
        uint256 revealDeadline;
        uint256 lowestBid;
        address lowestBidder;
        bool assigned;
        mapping(address => bytes32) commitments;
        mapping(address => uint256) revealedBids;
        mapping(address => bool) hasRevealed;
    }

    // Task bidding mapping
    mapping(uint256 => TaskBidding) public tasks;

    // Events
    event BidCommitted(uint256 indexed taskId, address indexed bidder);
    event BidRevealed(uint256 indexed taskId, address indexed bidder, uint256 price);
    event TaskAssigned(uint256 indexed taskId, address indexed winner, uint256 price);
    event BiddingWindowsUpdated(uint256 commitWindow, uint256 revealWindow);

    /**
     * @dev Constructor
     * @param initialOwner Initial contract owner
     */
    constructor(address initialOwner) Ownable(initialOwner) {}

    /**
     * @dev Start bidding for a task
     * @param taskId Unique task identifier
     */
    function startBidding(uint256 taskId) external onlyOwner {
        require(tasks[taskId].taskId == 0, "Bidding already started");
        
        TaskBidding storage task = tasks[taskId];
        task.taskId = taskId;
        task.commitDeadline = block.timestamp + commitWindow;
        task.revealDeadline = task.commitDeadline + revealWindow;
        task.lowestBid = type(uint256).max;
    }

    /**
     * @dev Commit a sealed bid
     * @param taskId Task identifier
     * @param commitment Hash of (price, salt, bidder)
     */
    function commitBid(uint256 taskId, bytes32 commitment) external {
        TaskBidding storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(block.timestamp <= task.commitDeadline, "Commit window closed");
        require(task.commitments[msg.sender] == bytes32(0), "Already committed");

        task.commitments[msg.sender] = commitment;
        emit BidCommitted(taskId, msg.sender);
    }

    /**
     * @dev Reveal a committed bid
     * @param taskId Task identifier
     * @param price Bid price in DAVO tokens
     * @param salt Random salt used in commitment
     */
    function revealBid(uint256 taskId, uint256 price, bytes32 salt) external {
        TaskBidding storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(block.timestamp > task.commitDeadline, "Commit window not closed");
        require(block.timestamp <= task.revealDeadline, "Reveal window closed");
        require(task.commitments[msg.sender] != bytes32(0), "No commitment found");
        require(!task.hasRevealed[msg.sender], "Already revealed");

        // Verify commitment
        bytes32 expectedCommitment = keccak256(abi.encodePacked(price, salt, msg.sender));
        require(task.commitments[msg.sender] == expectedCommitment, "Invalid commitment");

        task.revealedBids[msg.sender] = price;
        task.hasRevealed[msg.sender] = true;

        // Update lowest bid if necessary
        if (price < task.lowestBid) {
            task.lowestBid = price;
            task.lowestBidder = msg.sender;
        }

        emit BidRevealed(taskId, msg.sender, price);
    }

    /**
     * @dev Assign task to lowest bidder
     * @param taskId Task identifier
     */
    function assignWinner(uint256 taskId) external onlyOwner {
        TaskBidding storage task = tasks[taskId];
        require(task.taskId != 0, "Task not found");
        require(block.timestamp > task.revealDeadline, "Reveal window not closed");
        require(!task.assigned, "Task already assigned");
        require(task.lowestBidder != address(0), "No valid bids");

        task.assigned = true;
        emit TaskAssigned(taskId, task.lowestBidder, task.lowestBid);
    }

    /**
     * @dev Get task bidding information
     * @param taskId Task identifier
     * @return commitDeadline Commit window deadline
     * @return revealDeadline Reveal window deadline
     * @return lowestBid Lowest revealed bid
     * @return lowestBidder Address of lowest bidder
     * @return assigned Whether task is assigned
     */
    function getTaskBidding(uint256 taskId) 
        external 
        view 
        returns (
            uint256 commitDeadline,
            uint256 revealDeadline,
            uint256 lowestBid,
            address lowestBidder,
            bool assigned
        ) 
    {
        TaskBidding storage task = tasks[taskId];
        return (
            task.commitDeadline,
            task.revealDeadline,
            task.lowestBid,
            task.lowestBidder,
            task.assigned
        );
    }

    /**
     * @dev Check if address has committed to task
     * @param taskId Task identifier
     * @param bidder Bidder address
     * @return True if committed
     */
    function hasCommitted(uint256 taskId, address bidder) external view returns (bool) {
        return tasks[taskId].commitments[bidder] != bytes32(0);
    }

    /**
     * @dev Check if address has revealed bid
     * @param taskId Task identifier
     * @param bidder Bidder address
     * @return True if revealed
     */
    function hasRevealed(uint256 taskId, address bidder) external view returns (bool) {
        return tasks[taskId].hasRevealed[bidder];
    }

    /**
     * @dev Get revealed bid amount
     * @param taskId Task identifier
     * @param bidder Bidder address
     * @return Bid amount (0 if not revealed)
     */
    function getRevealedBid(uint256 taskId, address bidder) external view returns (uint256) {
        return tasks[taskId].revealedBids[bidder];
    }

    /**
     * @dev Update bidding windows (owner only)
     * @param newCommitWindow New commit window duration
     * @param newRevealWindow New reveal window duration
     */
    function updateBiddingWindows(uint256 newCommitWindow, uint256 newRevealWindow) external onlyOwner {
        require(newCommitWindow > 0, "Commit window must be positive");
        require(newRevealWindow > 0, "Reveal window must be positive");
        
        commitWindow = newCommitWindow;
        revealWindow = newRevealWindow;
        
        emit BiddingWindowsUpdated(newCommitWindow, newRevealWindow);
    }

    /**
     * @dev Check if bidding is active for task
     * @param taskId Task identifier
     * @return True if bidding is active
     */
    function isBiddingActive(uint256 taskId) external view returns (bool) {
        TaskBidding storage task = tasks[taskId];
        return task.taskId != 0 && !task.assigned && block.timestamp <= task.revealDeadline;
    }

    /**
     * @dev Check if commit window is open
     * @param taskId Task identifier
     * @return True if commit window is open
     */
    function isCommitWindowOpen(uint256 taskId) external view returns (bool) {
        TaskBidding storage task = tasks[taskId];
        return task.taskId != 0 && block.timestamp <= task.commitDeadline;
    }

    /**
     * @dev Check if reveal window is open
     * @param taskId Task identifier
     * @return True if reveal window is open
     */
    function isRevealWindowOpen(uint256 taskId) external view returns (bool) {
        TaskBidding storage task = tasks[taskId];
        return task.taskId != 0 && 
               block.timestamp > task.commitDeadline && 
               block.timestamp <= task.revealDeadline;
    }
}
