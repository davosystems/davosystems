// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/token/ERC20/ERC20.sol";
import "@openzeppelin/contracts/token/ERC20/extensions/ERC20Burnable.sol";
import "@openzeppelin/contracts/access/Ownable.sol";

/**
 * @title DAVO Token
 * @dev ERC20 token for Davo Systems task coordination
 * Fixed supply of 1,000,000 tokens with burn functionality
 */
contract DAVO is ERC20, ERC20Burnable, Ownable {
    uint256 public constant TOTAL_SUPPLY = 1_000_000 * 10**18; // 1M tokens with 18 decimals

    event TokensBurned(address indexed burner, uint256 amount);

    /**
     * @dev Constructor mints total supply to deployer
     * @param initialOwner Address to receive initial token supply
     */
    constructor(address initialOwner) ERC20("Davo Systems", "DAVO") Ownable(initialOwner) {
        _mint(initialOwner, TOTAL_SUPPLY);
    }

    /**
     * @dev Burns tokens and emits custom event
     * @param amount Amount of tokens to burn
     */
    function burn(uint256 amount) public override {
        super.burn(amount);
        emit TokensBurned(msg.sender, amount);
    }

    /**
     * @dev Burns tokens from a specific account (requires allowance)
     * @param account Account to burn tokens from
     * @param amount Amount of tokens to burn
     */
    function burnFrom(address account, uint256 amount) public override {
        super.burnFrom(account, amount);
        emit TokensBurned(account, amount);
    }

    /**
     * @dev Returns the total supply of tokens
     * @return Total supply including burned tokens
     */
    function totalSupply() public view override returns (uint256) {
        return super.totalSupply();
    }

    /**
     * @dev Returns the circulating supply (total supply minus burned tokens)
     * @return Circulating supply
     */
    function circulatingSupply() public view returns (uint256) {
        return totalSupply();
    }

    /**
     * @dev Returns the total burned tokens
     * @return Total burned amount
     */
    function totalBurned() public view returns (uint256) {
        return TOTAL_SUPPLY - totalSupply();
    }
}
