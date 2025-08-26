#!/usr/bin/env python3

import hashlib
from eth_account import Account
from eth_account.messages import encode_defunct
from web3 import Web3


class DavoSigner:
    """
    EIP-712 signer for Davo Systems task results.
    Creates signatures compatible with the TaskEscrow contract.
    """

    def __init__(self, private_key):
        """
        Initialize signer with robot's private key.
        
        Args:
            private_key (str): Robot's private key (hex string)
        """
        self.account = Account.from_key(private_key)
        self.w3 = Web3()
        
        # EIP-712 domain for Davo Systems
        self.domain = {
            'name': 'DavoSystems',
            'version': '1',
            'chainId': 84532,  # Base Sepolia
            'verifyingContract': '0x0000000000000000000000000000000000000000'  # Will be set dynamically
        }
        
        # TaskResult type definition
        self.task_result_type = {
            'TaskResult': [
                {'name': 'taskId', 'type': 'uint256'},
                {'name': 'resultHash', 'type': 'bytes32'},
                {'name': 'robotId', 'type': 'uint256'},
                {'name': 'timestamp', 'type': 'uint256'}
            ]
        }

    def set_verifying_contract(self, contract_address):
        """
        Set the verifying contract address for the domain.
        
        Args:
            contract_address (str): TaskEscrow contract address
        """
        self.domain['verifyingContract'] = contract_address

    def sign_task_result(self, task_id, result_hash, robot_id, timestamp=None):
        """
        Sign a task result using EIP-712.
        
        Args:
            task_id (int): Task identifier
            result_hash (str): Hash of task result (hex string)
            robot_id (int): Robot identifier
            timestamp (int, optional): Timestamp (uses current time if None)
            
        Returns:
            bytes: EIP-712 signature
        """
        if timestamp is None:
            timestamp = int(self.w3.eth.get_block('latest').timestamp)
        
        # Create the message data
        message = {
            'taskId': task_id,
            'resultHash': result_hash,
            'robotId': robot_id,
            'timestamp': timestamp
        }
        
        # Create EIP-712 signature
        signable_message = encode_defunct(
            primitive=self.w3.eth.account._sign_typed_data(
                self.domain,
                self.task_result_type,
                message
            )
        )
        
        signed_message = self.account.sign_message(signable_message)
        return signed_message.signature

    def verify_signature(self, task_id, result_hash, robot_id, timestamp, signature, expected_signer):
        """
        Verify an EIP-712 signature.
        
        Args:
            task_id (int): Task identifier
            result_hash (str): Hash of task result
            robot_id (int): Robot identifier
            timestamp (int): Timestamp
            signature (bytes): EIP-712 signature
            expected_signer (str): Expected signer address
            
        Returns:
            bool: True if signature is valid
        """
        try:
            # Create the message data
            message = {
                'taskId': task_id,
                'resultHash': result_hash,
                'robotId': robot_id,
                'timestamp': timestamp
            }
            
            # Create signable message
            signable_message = encode_defunct(
                primitive=self.w3.eth.account._sign_typed_data(
                    self.domain,
                    self.task_result_type,
                    message
                )
            )
            
            # Recover signer
            recovered_address = Account.recover_message(signable_message, signature=signature)
            
            return recovered_address.lower() == expected_signer.lower()
            
        except Exception as e:
            print(f"Error verifying signature: {e}")
            return False

    def create_commitment(self, price, salt, bidder):
        """
        Create a bid commitment hash for CommitRevealBids.
        
        Args:
            price (int): Bid price in wei
            salt (bytes32): Random salt
            bidder (str): Bidder address
            
        Returns:
            bytes32: Commitment hash
        """
        commitment_data = self.w3.solidity_keccak(
            ['uint256', 'bytes32', 'address'],
            [price, salt, bidder]
        )
        return commitment_data

    def sign_attestation(self, task_id, result_hash, robot_id, timestamp):
        """
        Sign an attestation for TelemetryAttestor.
        
        Args:
            task_id (int): Task identifier
            result_hash (str): Hash of task result
            robot_id (int): Robot identifier
            timestamp (int): Timestamp
            
        Returns:
            bytes: Signature
        """
        # Create attestation hash
        attestation_data = self.w3.solidity_keccak(
            ['uint256', 'bytes32', 'uint256', 'uint256', 'address'],
            [task_id, result_hash, robot_id, timestamp, self.account.address]
        )
        
        # Sign with Ethereum message format
        signable_message = encode_defunct(attestation_data)
        signed_message = self.account.sign_message(signable_message)
        
        return signed_message.signature

    def get_public_key_hash(self):
        """
        Get the hash of the robot's public key.
        
        Returns:
            bytes32: Hash of public key
        """
        public_key = self.account.key.public_key
        return hashlib.sha256(public_key.to_bytes()).hexdigest()

    def get_address(self):
        """
        Get the robot's EVM address.
        
        Returns:
            str: EVM address
        """
        return self.account.address


def main():
    """CLI interface for signing."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Davo Systems EIP-712 Signer')
    parser.add_argument('--private-key', required=True, help='Robot private key')
    parser.add_argument('--task-id', type=int, required=True, help='Task ID')
    parser.add_argument('--result-hash', required=True, help='Result hash')
    parser.add_argument('--robot-id', type=int, required=True, help='Robot ID')
    parser.add_argument('--contract-address', help='TaskEscrow contract address')
    
    args = parser.parse_args()
    
    # Initialize signer
    signer = DavoSigner(args.private_key)
    
    if args.contract_address:
        signer.set_verifying_contract(args.contract_address)
    
    # Sign task result
    signature = signer.sign_task_result(
        args.task_id,
        args.result_hash,
        args.robot_id
    )
    
    print(f"Task Result Signature: {signature.hex()}")
    print(f"Robot Address: {signer.get_address()}")
    print(f"Public Key Hash: {signer.get_public_key_hash()}")


if __name__ == '__main__':
    main()
