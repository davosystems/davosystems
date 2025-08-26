#!/usr/bin/env python3

import pytest
import json
import hashlib
from unittest.mock import Mock, patch, MagicMock
from web3 import Web3
from eth_account import Account

from davo_bridge.bridge_node import DavoBridgeNode
from davo_bridge.signer import DavoSigner


class TestDavoSigner:
    """Test the DavoSigner class."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.private_key = "0x" + "1" * 64
        self.signer = DavoSigner(self.private_key)
        self.account = Account.from_key(self.private_key)
    
    def test_init(self):
        """Test signer initialization."""
        assert self.signer.account.address == self.account.address
        assert self.signer.domain['name'] == 'DavoSystems'
        assert self.signer.domain['version'] == '1'
        assert self.signer.domain['chainId'] == 84532
    
    def test_set_verifying_contract(self):
        """Test setting verifying contract address."""
        contract_address = "0x1234567890123456789012345678901234567890"
        self.signer.set_verifying_contract(contract_address)
        assert self.signer.domain['verifyingContract'] == contract_address
    
    def test_sign_task_result(self):
        """Test signing task result."""
        task_id = 1
        result_hash = "0x" + "a" * 64
        robot_id = 1
        
        signature = self.signer.sign_task_result(task_id, result_hash, robot_id)
        
        assert isinstance(signature, bytes)
        assert len(signature) == 65  # ECDSA signature length
    
    def test_verify_signature(self):
        """Test signature verification."""
        task_id = 1
        result_hash = "0x" + "a" * 64
        robot_id = 1
        timestamp = 1234567890
        
        signature = self.signer.sign_task_result(task_id, result_hash, robot_id, timestamp)
        
        # Verify with correct signer
        is_valid = self.signer.verify_signature(
            task_id, result_hash, robot_id, timestamp, signature, self.account.address
        )
        assert is_valid is True
        
        # Verify with wrong signer
        wrong_address = "0x" + "b" * 40
        is_valid = self.signer.verify_signature(
            task_id, result_hash, robot_id, timestamp, signature, wrong_address
        )
        assert is_valid is False
    
    def test_create_commitment(self):
        """Test creating bid commitment."""
        price = 1000 * 10**18
        salt = "0x" + "s" * 64
        bidder = "0x" + "b" * 40
        
        commitment = self.signer.create_commitment(price, salt, bidder)
        
        assert isinstance(commitment, bytes)
        assert len(commitment) == 32
    
    def test_sign_attestation(self):
        """Test signing attestation."""
        task_id = 1
        result_hash = "0x" + "a" * 64
        robot_id = 1
        timestamp = 1234567890
        
        signature = self.signer.sign_attestation(task_id, result_hash, robot_id, timestamp)
        
        assert isinstance(signature, bytes)
        assert len(signature) == 65
    
    def test_get_public_key_hash(self):
        """Test getting public key hash."""
        pubkey_hash = self.signer.get_public_key_hash()
        
        assert isinstance(pubkey_hash, str)
        assert len(pubkey_hash) == 64  # 32 bytes as hex
    
    def test_get_address(self):
        """Test getting account address."""
        address = self.signer.get_address()
        
        assert address == self.account.address
        assert address.startswith("0x")


class TestDavoBridgeNode:
    """Test the DavoBridgeNode class."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.config = {
            'rpc_url': 'https://sepolia.base.org',
            'private_key': '0x' + '1' * 64,
            'robot_private_key': '0x' + '2' * 64,
            'contracts': {
                'task_escrow': '0x' + 'a' * 40,
                'robot_identity': '0x' + 'b' * 40,
                'davo_token': '0x' + 'c' * 40
            },
            'topics': {
                'task_states': '/task_states',
                'fleet_states': '/fleet_states'
            }
        }
    
    @patch('davo_bridge.bridge_node.Web3')
    @patch('davo_bridge.bridge_node.Account')
    def test_init(self, mock_account, mock_web3):
        """Test bridge node initialization."""
        mock_w3 = Mock()
        mock_w3.is_connected.return_value = True
        mock_web3.HTTPProvider.return_value = mock_w3
        mock_web3.return_value = mock_w3
        
        mock_account_instance = Mock()
        mock_account_instance.address = '0x' + '1' * 40
        mock_account.from_key.return_value = mock_account_instance
        
        with patch('davo_bridge.bridge_node.DavoBridgeNode._load_config', return_value=self.config):
            with patch('davo_bridge.bridge_node.DavoBridgeNode._init_contracts'):
                with patch('davo_bridge.bridge_node.DavoBridgeNode._init_database'):
                    bridge = DavoBridgeNode()
                    
                    assert bridge.config == self.config
                    assert bridge.w3 == mock_w3
                    assert bridge.account == mock_account_instance
    
    def test_load_config(self):
        """Test configuration loading."""
        with patch('builtins.open', mock_open(read_data='rpc_url: "https://test.com"')) as mock_file:
            with patch('yaml.safe_load', return_value=self.config):
                bridge = DavoBridgeNode()
                config = bridge._load_config()
                
                assert config == self.config
    
    def test_get_contract_abi(self):
        """Test getting contract ABIs."""
        bridge = DavoBridgeNode()
        
        # Test TaskEscrow ABI
        abi = bridge._get_contract_abi('TaskEscrow')
        assert isinstance(abi, list)
        assert len(abi) > 0
        assert any(func['name'] == 'postTask' for func in abi)
        
        # Test RobotIdentity ABI
        abi = bridge._get_contract_abi('RobotIdentity')
        assert isinstance(abi, list)
        assert len(abi) > 0
        assert any(func['name'] == 'getRobotInfo' for func in abi)
        
        # Test DAVO ABI
        abi = bridge._get_contract_abi('DAVO')
        assert isinstance(abi, list)
        assert len(abi) > 0
        assert any(func['name'] == 'transfer' for func in abi)
    
    def test_store_task_mapping(self):
        """Test storing task mapping."""
        bridge = DavoBridgeNode()
        bridge.conn = Mock()
        bridge.cursor = Mock()
        
        rmf_task_id = "rmf_task_123"
        onchain_task_id = 1
        spec_hash = "0x" + "a" * 64
        budget = 1000 * 10**18
        
        bridge._store_task_mapping(rmf_task_id, onchain_task_id, spec_hash, budget)
        
        bridge.cursor.execute.assert_called_once()
        bridge.conn.commit.assert_called_once()
    
    def test_get_onchain_task_id(self):
        """Test getting on-chain task ID."""
        bridge = DavoBridgeNode()
        bridge.cursor = Mock()
        bridge.cursor.fetchone.return_value = (1,)
        
        task_id = bridge._get_onchain_task_id("rmf_task_123")
        
        assert task_id == 1
        bridge.cursor.execute.assert_called_once()
    
    def test_get_robot_id(self):
        """Test getting robot ID."""
        bridge = DavoBridgeNode()
        bridge.cursor = Mock()
        bridge.cursor.fetchone.return_value = (1,)
        
        robot_id = bridge._get_robot_id("robot_01")
        
        assert robot_id == 1
        bridge.cursor.execute.assert_called_once()
    
    @patch('davo_bridge.bridge_node.DavoBridgeNode._post_task')
    def test_handle_task_requested(self, mock_post_task):
        """Test handling task request."""
        bridge = DavoBridgeNode()
        bridge._store_task_mapping = Mock()
        
        # Mock task message
        msg = Mock()
        msg.task_id = "rmf_task_123"
        msg.task_profile = Mock()
        msg.task_profile.description = Mock()
        msg.task_profile.description.task_type = Mock()
        msg.task_profile.description.task_type.type = "delivery"
        msg.task_profile.priority = Mock()
        msg.task_profile.priority.value = "normal"
        msg.task_profile.start_time = Mock()
        msg.task_profile.start_time.sec = 1234567890
        msg.task_profile.end_time = Mock()
        msg.task_profile.end_time.sec = 1234567890 + 3600
        
        mock_post_task.return_value = 1
        
        bridge._handle_task_requested(msg)
        
        mock_post_task.assert_called_once()
        bridge._store_task_mapping.assert_called_once()
    
    def test_handle_task_completed(self):
        """Test handling task completion."""
        bridge = DavoBridgeNode()
        bridge._get_onchain_task_id = Mock(return_value=1)
        bridge._get_robot_id = Mock(return_value=1)
        bridge._submit_result = Mock()
        
        # Mock task message
        msg = Mock()
        msg.task_id = "rmf_task_123"
        msg.assigned_to = "robot_01"
        msg.completion_time = Mock()
        msg.completion_time.sec = 1234567890
        
        bridge._handle_task_completed(msg)
        
        bridge._submit_result.assert_called_once()
    
    def test_map_robot(self):
        """Test mapping robot."""
        bridge = DavoBridgeNode()
        bridge.cursor = Mock()
        bridge.robot_mapping = {}
        
        bridge._map_robot("robot_01")
        
        assert "robot_01" in bridge.robot_mapping
        assert bridge.robot_mapping["robot_01"] == 1
        bridge.cursor.execute.assert_called_once()
        bridge.conn.commit.assert_called_once()


class TestIntegration:
    """Integration tests for the bridge."""
    
    def test_eip712_signature_compatibility(self):
        """Test that EIP-712 signatures are compatible with Solidity."""
        private_key = "0x" + "1" * 64
        signer = DavoSigner(private_key)
        
        # Set verifying contract
        contract_address = "0x1234567890123456789012345678901234567890"
        signer.set_verifying_contract(contract_address)
        
        # Create signature
        task_id = 1
        result_hash = "0x" + "a" * 64
        robot_id = 1
        timestamp = 1234567890
        
        signature = signer.sign_task_result(task_id, result_hash, robot_id, timestamp)
        
        # Verify signature
        is_valid = signer.verify_signature(
            task_id, result_hash, robot_id, timestamp, signature, signer.get_address()
        )
        
        assert is_valid is True
    
    def test_task_specification_hashing(self):
        """Test task specification hashing."""
        task_spec = {
            "task_type": "delivery",
            "priority": "normal",
            "start_location": "gate_A1",
            "end_location": "gate_B3",
            "cargo": "passenger_luggage",
            "deadline": 1234567890,
            "robot_requirements": ["autonomous", "cargo_capacity"],
            "estimated_duration": 1800
        }
        
        # Hash the specification
        spec_hash = hashlib.sha256(
            json.dumps(task_spec, sort_keys=True).encode()
        ).hexdigest()
        
        assert isinstance(spec_hash, str)
        assert len(spec_hash) == 64  # 32 bytes as hex
        
        # Verify deterministic hashing
        spec_hash2 = hashlib.sha256(
            json.dumps(task_spec, sort_keys=True).encode()
        ).hexdigest()
        
        assert spec_hash == spec_hash2
    
    def test_result_data_hashing(self):
        """Test result data hashing."""
        result_data = {
            "task_id": 1,
            "robot_id": 1,
            "completion_time": 1234567890,
            "actual_duration": 1650,
            "status": "completed",
            "metrics": {
                "distance_traveled": 1250,
                "energy_consumed": 85,
                "cargo_delivered": True,
                "safety_incidents": 0
            }
        }
        
        # Hash the result
        result_hash = hashlib.sha256(
            json.dumps(result_data, sort_keys=True).encode()
        ).hexdigest()
        
        assert isinstance(result_hash, str)
        assert len(result_hash) == 64  # 32 bytes as hex


if __name__ == "__main__":
    pytest.main([__file__])
