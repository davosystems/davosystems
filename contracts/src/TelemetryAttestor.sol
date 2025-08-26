// SPDX-License-Identifier: MIT
pragma solidity ^0.8.19;

import "@openzeppelin/contracts/access/Ownable.sol";
import "@openzeppelin/contracts/utils/cryptography/ECDSA.sol";

/**
 * @title TelemetryAttestor
 * @dev Neutral verification for task results
 * Allowlisted signers can attest task results for organizations
 */
contract TelemetryAttestor is Ownable {
    using ECDSA for bytes32;

    // Attestation struct
    struct Attestation {
        uint256 taskId;
        bytes32 resultHash;
        uint256 robotId;
        uint256 timestamp;
        address attester;
        bool valid;
    }

    // Allowlisted attestors
    mapping(address => bool) public isAttestor;
    mapping(address => uint256) public attestorReputation;
    mapping(bytes32 => Attestation) public attestations;

    // Events
    event AttestorAdded(address indexed attestor);
    event AttestorRemoved(address indexed attestor);
    event AttestationCreated(
        uint256 indexed taskId,
        bytes32 resultHash,
        uint256 robotId,
        address indexed attester
    );
    event AttestationRevoked(
        uint256 indexed taskId,
        bytes32 resultHash,
        address indexed attester
    );
    event ReputationUpdated(address indexed attestor, uint256 newReputation);

    /**
     * @dev Constructor
     * @param initialOwner Initial contract owner
     */
    constructor(address initialOwner) Ownable(initialOwner) {}

    /**
     * @dev Add an attestor to the allowlist
     * @param attestor Address to add as attestor
     */
    function addAttestor(address attestor) external onlyOwner {
        require(attestor != address(0), "Invalid attestor address");
        require(!isAttestor[attestor], "Already an attestor");
        
        isAttestor[attestor] = true;
        attestorReputation[attestor] = 100; // Initial reputation
        
        emit AttestorAdded(attestor);
    }

    /**
     * @dev Remove an attestor from the allowlist
     * @param attestor Address to remove as attestor
     */
    function removeAttestor(address attestor) external onlyOwner {
        require(isAttestor[attestor], "Not an attestor");
        
        isAttestor[attestor] = false;
        attestorReputation[attestor] = 0;
        
        emit AttestorRemoved(attestor);
    }

    /**
     * @dev Create an attestation for a task result
     * @param taskId Task identifier
     * @param resultHash Hash of task result
     * @param robotId Robot identifier
     * @param signature EIP-712 signature from attestor
     */
    function createAttestation(
        uint256 taskId,
        bytes32 resultHash,
        uint256 robotId,
        bytes calldata signature
    ) external {
        require(isAttestor[msg.sender], "Not an attestor");
        require(taskId > 0, "Invalid task ID");
        require(resultHash != bytes32(0), "Invalid result hash");
        require(robotId > 0, "Invalid robot ID");

        // Create attestation hash
        bytes32 attestationHash = keccak256(
            abi.encodePacked(
                taskId,
                resultHash,
                robotId,
                block.timestamp,
                msg.sender
            )
        );

        // Verify signature
        bytes32 digest = keccak256(
            abi.encodePacked("\x19Ethereum Signed Message:\n32", attestationHash)
        );
        address signer = digest.recover(signature);
        require(signer == msg.sender, "Invalid signature");

        // Check if attestation already exists
        require(!attestations[attestationHash].valid, "Attestation already exists");

        // Create attestation
        attestations[attestationHash] = Attestation({
            taskId: taskId,
            resultHash: resultHash,
            robotId: robotId,
            timestamp: block.timestamp,
            attester: msg.sender,
            valid: true
        });

        emit AttestationCreated(taskId, resultHash, robotId, msg.sender);
    }

    /**
     * @dev Revoke an attestation
     * @param taskId Task identifier
     * @param resultHash Hash of task result
     * @param robotId Robot identifier
     * @param timestamp Original attestation timestamp
     * @param attester Original attester address
     */
    function revokeAttestation(
        uint256 taskId,
        bytes32 resultHash,
        uint256 robotId,
        uint256 timestamp,
        address attester
    ) external {
        require(msg.sender == attester || msg.sender == owner(), "Not authorized");

        bytes32 attestationHash = keccak256(
            abi.encodePacked(taskId, resultHash, robotId, timestamp, attester)
        );

        Attestation storage attestation = attestations[attestationHash];
        require(attestation.valid, "Attestation not found");

        attestation.valid = false;

        // Reduce attester reputation if revoked by owner
        if (msg.sender == owner() && attester != owner()) {
            attestorReputation[attester] = attestorReputation[attester] > 10 
                ? attestorReputation[attester] - 10 
                : 0;
            emit ReputationUpdated(attester, attestorReputation[attester]);
        }

        emit AttestationRevoked(taskId, resultHash, attester);
    }

    /**
     * @dev Update attestor reputation
     * @param attestor Attestor address
     * @param newReputation New reputation score
     */
    function updateReputation(address attestor, uint256 newReputation) external onlyOwner {
        require(isAttestor[attestor], "Not an attestor");
        require(newReputation <= 1000, "Reputation too high");
        
        attestorReputation[attestor] = newReputation;
        emit ReputationUpdated(attestor, newReputation);
    }

    /**
     * @dev Get attestation information
     * @param taskId Task identifier
     * @param resultHash Hash of task result
     * @param robotId Robot identifier
     * @param timestamp Original attestation timestamp
     * @param attester Original attester address
     * @return attestation Attestation data
     */
    function getAttestation(
        uint256 taskId,
        bytes32 resultHash,
        uint256 robotId,
        uint256 timestamp,
        address attester
    ) external view returns (Attestation memory attestation) {
        bytes32 attestationHash = keccak256(
            abi.encodePacked(taskId, resultHash, robotId, timestamp, attester)
        );
        return attestations[attestationHash];
    }

    /**
     * @dev Check if an attestation is valid
     * @param taskId Task identifier
     * @param resultHash Hash of task result
     * @param robotId Robot identifier
     * @param timestamp Original attestation timestamp
     * @param attester Original attester address
     * @return True if attestation is valid
     */
    function isAttestationValid(
        uint256 taskId,
        bytes32 resultHash,
        uint256 robotId,
        uint256 timestamp,
        address attester
    ) external view returns (bool) {
        bytes32 attestationHash = keccak256(
            abi.encodePacked(taskId, resultHash, robotId, timestamp, attester)
        );
        return attestations[attestationHash].valid;
    }

    /**
     * @dev Get attestor information
     * @param attestor Attestor address
     * @return isAllowed Whether attestor is allowlisted
     * @return reputation Attestor reputation score
     */
    function getAttestorInfo(address attestor) 
        external 
        view 
        returns (bool isAllowed, uint256 reputation) 
    {
        return (isAttestor[attestor], attestorReputation[attestor]);
    }

    /**
     * @dev Get total number of valid attestations
     * @return count Total valid attestations
     */
    function getValidAttestationCount() external view returns (uint256 count) {
        // This would require iterating through all attestations
        // For gas efficiency, this is a placeholder
        return 0;
    }
}
