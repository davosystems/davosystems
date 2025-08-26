// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/token/ERC721/ERC721.sol";
import "@openzeppelin/contracts/token/ERC721/extensions/ERC721URIStorage.sol";
import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";
import "@openzeppelin/contracts/utils/cryptography/EIP712.sol";

/**
 * @title RobotIdentity
 * @dev ERC721 contract for robot identity binding
 * Binds robot public keys to EVM addresses for EIP-712 signatures
 */
contract RobotIdentity is ERC721, ERC721URIStorage, Ownable, EIP712 {
    using ECDSA for bytes32;

    // EIP-712 domain for task result signatures
    string private constant DOMAIN_NAME = "DavoSystems";
    string private constant DOMAIN_VERSION = "1";

    // Robot identity mapping
    mapping(uint256 => address) public robotToAddress;
    mapping(address => uint256) public addressToRobot;
    mapping(uint256 => bytes32) public robotPublicKeyHash;

    // Events
    event RobotRegistered(uint256 indexed robotId, address indexed evmAddress, bytes32 pubkeyHash);
    event KeyBound(uint256 indexed robotId, address indexed evmAddress, bytes32 pubkeyHash);

    /**
     * @dev Constructor sets up EIP-712 domain
     * @param initialOwner Initial contract owner
     */
    constructor(address initialOwner) 
        ERC721("Robot Identity", "ROBOTID") 
        Ownable(initialOwner)
        EIP712(DOMAIN_NAME, DOMAIN_VERSION)
    {}

    /**
     * @dev Register a new robot identity
     * @param robotId Unique robot identifier
     * @param evmAddress EVM address for the robot
     * @param pubkeyHash Hash of robot's public key
     * @param uri Metadata URI for the robot
     */
    function registerRobot(
        uint256 robotId,
        address evmAddress,
        bytes32 pubkeyHash,
        string memory uri
    ) external onlyOwner {
        require(robotId > 0, "Robot ID must be positive");
        require(evmAddress != address(0), "Invalid EVM address");
        require(robotToAddress[robotId] == address(0), "Robot ID already exists");
        require(addressToRobot[evmAddress] == 0, "EVM address already bound");

        _safeMint(msg.sender, robotId);
        _setTokenURI(robotId, uri);
        
        robotToAddress[robotId] = evmAddress;
        addressToRobot[evmAddress] = robotId;
        robotPublicKeyHash[robotId] = pubkeyHash;

        emit RobotRegistered(robotId, evmAddress, pubkeyHash);
    }

    /**
     * @dev Bind a robot's public key to their EVM address
     * @param robotId Robot identifier
     * @param evmAddress EVM address for the robot
     * @param pubkeyHash Hash of robot's public key
     */
    function bindKey(
        uint256 robotId,
        address evmAddress,
        bytes32 pubkeyHash
    ) external onlyOwner {
        require(_exists(robotId), "Robot does not exist");
        require(evmAddress != address(0), "Invalid EVM address");
        require(robotToAddress[robotId] == evmAddress, "Robot ID and address mismatch");

        robotPublicKeyHash[robotId] = pubkeyHash;
        emit KeyBound(robotId, evmAddress, pubkeyHash);
    }

    /**
     * @dev Get robot information
     * @param robotId Robot identifier
     * @return evmAddress EVM address bound to robot
     * @return pubkeyHash Hash of robot's public key
     * @return uri Metadata URI
     */
    function getRobotInfo(uint256 robotId) 
        external 
        view 
        returns (address evmAddress, bytes32 pubkeyHash, string memory uri) 
    {
        require(_exists(robotId), "Robot does not exist");
        evmAddress = robotToAddress[robotId];
        pubkeyHash = robotPublicKeyHash[robotId];
        uri = tokenURI(robotId);
    }

    /**
     * @dev Get robot ID by EVM address
     * @param evmAddress EVM address
     * @return robotId Robot identifier
     */
    function getRobotId(address evmAddress) external view returns (uint256 robotId) {
        robotId = addressToRobot[evmAddress];
        require(robotId > 0, "Address not bound to robot");
    }

    /**
     * @dev Verify that an address is bound to a robot
     * @param robotId Robot identifier
     * @param evmAddress EVM address to verify
     * @return True if bound
     */
    function isRobotBound(uint256 robotId, address evmAddress) external view returns (bool) {
        return robotToAddress[robotId] == evmAddress && _exists(robotId);
    }

    /**
     * @dev Get EIP-712 domain separator
     * @return Domain separator for task result signatures
     */
    function getDomainSeparator() external view returns (bytes32) {
        return _domainSeparatorV4();
    }

    /**
     * @dev Override required by Solidity
     */
    function tokenURI(uint256 tokenId)
        public
        view
        override(ERC721, ERC721URIStorage)
        returns (string memory)
    {
        return super.tokenURI(tokenId);
    }

    /**
     * @dev Override required by Solidity
     */
    function supportsInterface(bytes4 interfaceId)
        public
        view
        override(ERC721, ERC721URIStorage)
        returns (bool)
    {
        return super.supportsInterface(interfaceId);
    }
}
