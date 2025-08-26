// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/token/ERC20/IERC20.sol";
import "./DAVO.sol";

/**
 * @title BuybackBurner
 * @dev Token burn mechanism for Davo Systems
 * For testnet: direct burn, for mainnet: DEX swap then burn
 */
contract BuybackBurner is Ownable {
    // Contract interfaces
    DAVO public immutable davoToken;

    // Burn tracking
    uint256 public totalBurned;
    mapping(address => uint256) public burnedByRouter;

    // Events
    event TokensBurned(uint256 amount);
    event BurnerUpdated(address indexed router, uint256 amount);

    /**
     * @dev Constructor
     * @param initialOwner Initial contract owner
     * @param _davoToken DAVO token contract address
     */
    constructor(address initialOwner, address _davoToken) Ownable(initialOwner) {
        davoToken = DAVO(_davoToken);
    }

    /**
     * @dev Burn tokens (called by FeeRouter)
     * @param amount Amount of tokens to burn
     */
    function burnTokens(uint256 amount) external {
        require(amount > 0, "Amount must be positive");
        require(
            davoToken.transferFrom(msg.sender, address(this), amount),
            "Transfer failed"
        );

        // Burn the tokens
        davoToken.burn(amount);
        
        totalBurned += amount;
        burnedByRouter[msg.sender] += amount;

        emit TokensBurned(amount);
        emit BurnerUpdated(msg.sender, amount);
    }

    /**
     * @dev Manual burn function (owner only, for testing)
     * @param amount Amount of tokens to burn
     */
    function manualBurn(uint256 amount) external onlyOwner {
        require(amount > 0, "Amount must be positive");
        require(
            davoToken.balanceOf(address(this)) >= amount,
            "Insufficient balance"
        );

        davoToken.burn(amount);
        totalBurned += amount;

        emit TokensBurned(amount);
    }

    /**
     * @dev Get burn statistics
     * @return totalBurned Total tokens burned
     * @return currentBalance Current token balance
     */
    function getBurnStats() external view returns (uint256, uint256) {
        return (totalBurned, davoToken.balanceOf(address(this)));
    }

    /**
     * @dev Get burn amount by router
     * @param router Router address
     * @return amount Burned amount by this router
     */
    function getBurnedByRouter(address router) external view returns (uint256) {
        return burnedByRouter[router];
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
     * @dev Interface for future DEX integration
     * This function can be overridden by a mainnet implementation
     * @param router DEX router address
     * @param amount Amount to swap and burn
     */
    function swapDavoForDavo(address router, uint256 amount) external virtual {
        // For testnet: direct burn
        // For mainnet: implement DEX swap logic
        require(amount > 0, "Amount must be positive");
        require(
            davoToken.transferFrom(msg.sender, address(this), amount),
            "Transfer failed"
        );

        davoToken.burn(amount);
        totalBurned += amount;
        burnedByRouter[router] += amount;

        emit TokensBurned(amount);
        emit BurnerUpdated(router, amount);
    }
}
