#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from rmf_task_msgs.msg import TaskSummary, TaskProfile
from rmf_fleet_msgs.msg import FleetState
import json
import sqlite3
import yaml
import os
from web3 import Web3
from eth_account import Account
from eth_account.messages import encode_defunct
import hashlib
from .signer import DavoSigner


class DavoBridgeNode(Node):
    """
    ROS 2 bridge node for Davo Systems blockchain integration.
    Subscribes to RMF topics and posts transactions to Base Sepolia.
    """

    def __init__(self):
        super().__init__('davo_bridge_node')
        
        # Load configuration
        self.config = self._load_config()
        
        # Initialize Web3 connection
        self.w3 = Web3(Web3.HTTPProvider(self.config['rpc_url']))
        if not self.w3.is_connected():
            self.get_logger().error('Failed to connect to RPC endpoint')
            return
        
        # Initialize account
        self.account = Account.from_key(self.config['private_key'])
        self.get_logger().info(f'Bridge account: {self.account.address}')
        
        # Initialize contracts
        self._init_contracts()
        
        # Initialize database
        self._init_database()
        
        # Initialize signer
        self.signer = DavoSigner(self.config['robot_private_key'])
        
        # Set up QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribe to RMF topics
        self.task_subscription = self.create_subscription(
            TaskSummary,
            self.config['topics']['task_states'],
            self.task_callback,
            qos
        )
        
        self.fleet_subscription = self.create_subscription(
            FleetState,
            self.config['topics']['fleet_states'],
            self.fleet_callback,
            qos
        )
        
        self.get_logger().info('Davo Bridge Node initialized')
        
        # Task mapping
        self.task_mapping = {}  # RMF task ID -> on-chain task ID
        self.robot_mapping = {}  # Robot name -> Robot ID
        
    def _load_config(self):
        """Load configuration from YAML file."""
        config_path = os.path.join(
            os.path.dirname(__file__), 
            '..', '..', 'config', 'config.yaml'
        )
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Override with environment variables
        if 'RPC_URL' in os.environ:
            config['rpc_url'] = os.environ['RPC_URL']
        if 'PRIVATE_KEY' in os.environ:
            config['private_key'] = os.environ['PRIVATE_KEY']
        if 'ROBOT_PRIVATE_KEY' in os.environ:
            config['robot_private_key'] = os.environ['ROBOT_PRIVATE_KEY']
            
        return config
    
    def _init_contracts(self):
        """Initialize contract instances."""
        try:
            # Load contract ABIs (simplified for demo)
            self.task_escrow = self.w3.eth.contract(
                address=self.config['contracts']['task_escrow'],
                abi=self._get_contract_abi('TaskEscrow')
            )
            
            self.robot_identity = self.w3.eth.contract(
                address=self.config['contracts']['robot_identity'],
                abi=self._get_contract_abi('RobotIdentity')
            )
            
            self.davo_token = self.w3.eth.contract(
                address=self.config['contracts']['davo_token'],
                abi=self._get_contract_abi('DAVO')
            )
            
            self.get_logger().info('Contracts initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize contracts: {e}')
    
    def _get_contract_abi(self, contract_name):
        """Get contract ABI (simplified for demo)."""
        # In a real implementation, load from JSON files
        if contract_name == 'TaskEscrow':
            return [
                {
                    "inputs": [
                        {"name": "specHash", "type": "bytes32"},
                        {"name": "budget", "type": "uint256"}
                    ],
                    "name": "postTask",
                    "outputs": [{"name": "", "type": "uint256"}],
                    "stateMutability": "nonpayable",
                    "type": "function"
                },
                {
                    "inputs": [
                        {"name": "taskId", "type": "uint256"},
                        {"name": "resultHash", "type": "bytes32"},
                        {"name": "robotId", "type": "uint256"},
                        {"name": "signature", "type": "bytes"}
                    ],
                    "name": "submitResult",
                    "outputs": [],
                    "stateMutability": "nonpayable",
                    "type": "function"
                }
            ]
        elif contract_name == 'RobotIdentity':
            return [
                {
                    "inputs": [{"name": "robotId", "type": "uint256"}],
                    "name": "getRobotInfo",
                    "outputs": [
                        {"name": "evmAddress", "type": "address"},
                        {"name": "pubkeyHash", "type": "bytes32"},
                        {"name": "uri", "type": "string"}
                    ],
                    "stateMutability": "view",
                    "type": "function"
                }
            ]
        elif contract_name == 'DAVO':
            return [
                {
                    "inputs": [
                        {"name": "to", "type": "address"},
                        {"name": "amount", "type": "uint256"}
                    ],
                    "name": "transfer",
                    "outputs": [{"name": "", "type": "bool"}],
                    "stateMutability": "nonpayable",
                    "type": "function"
                }
            ]
        
        return []
    
    def _init_database(self):
        """Initialize SQLite database for task mapping."""
        self.db_path = os.path.join(os.path.dirname(__file__), 'task_mapping.db')
        self.conn = sqlite3.connect(self.db_path)
        self.cursor = self.conn.cursor()
        
        # Create tables
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS task_mapping (
                rmf_task_id TEXT PRIMARY KEY,
                onchain_task_id INTEGER,
                spec_hash TEXT,
                budget INTEGER,
                status TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS robot_mapping (
                robot_name TEXT PRIMARY KEY,
                robot_id INTEGER,
                evm_address TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        self.conn.commit()
    
    def task_callback(self, msg):
        """Handle task state updates."""
        try:
            task_id = msg.task_id
            state = msg.state
            
            self.get_logger().info(f'Task {task_id} state: {state}')
            
            if state == TaskSummary.STATE_REQUESTED:
                self._handle_task_requested(msg)
            elif state == TaskSummary.STATE_ASSIGNED:
                self._handle_task_assigned(msg)
            elif state == TaskSummary.STATE_COMPLETED:
                self._handle_task_completed(msg)
            elif state == TaskSummary.STATE_FAILED:
                self._handle_task_failed(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in task callback: {e}')
    
    def fleet_callback(self, msg):
        """Handle fleet state updates."""
        try:
            fleet_name = msg.name
            for robot in msg.robots:
                robot_name = robot.name
                if robot_name not in self.robot_mapping:
                    self._map_robot(robot_name)
                    
        except Exception as e:
            self.get_logger().error(f'Error in fleet callback: {e}')
    
    def _handle_task_requested(self, msg):
        """Handle new task request."""
        try:
            # Extract task information
            task_id = msg.task_id
            task_profile = msg.task_profile
            
            # Create task specification hash
            spec_data = {
                'task_type': task_profile.description.task_type.type,
                'priority': task_profile.priority.value,
                'start_time': task_profile.start_time.sec,
                'end_time': task_profile.end_time.sec
            }
            spec_hash = hashlib.sha256(
                json.dumps(spec_data, sort_keys=True).encode()
            ).hexdigest()
            
            # Estimate budget (simplified)
            budget = 1000 * 10**18  # 1000 DAVO tokens
            
            # Post task to blockchain
            onchain_task_id = self._post_task(spec_hash, budget)
            
            # Store mapping
            self._store_task_mapping(task_id, onchain_task_id, spec_hash, budget)
            
            self.get_logger().info(f'Posted task {task_id} as on-chain task {onchain_task_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling task request: {e}')
    
    def _handle_task_assigned(self, msg):
        """Handle task assignment."""
        try:
            task_id = msg.task_id
            robot_name = msg.assigned_to
            
            # Get on-chain task ID
            onchain_task_id = self._get_onchain_task_id(task_id)
            if onchain_task_id is None:
                return
            
            # Get robot ID
            robot_id = self._get_robot_id(robot_name)
            if robot_id is None:
                return
            
            self.get_logger().info(f'Task {task_id} assigned to robot {robot_name} (ID: {robot_id})')
            
        except Exception as e:
            self.get_logger().error(f'Error handling task assignment: {e}')
    
    def _handle_task_completed(self, msg):
        """Handle task completion."""
        try:
            task_id = msg.task_id
            robot_name = msg.assigned_to
            
            # Get on-chain task ID
            onchain_task_id = self._get_onchain_task_id(task_id)
            if onchain_task_id is None:
                return
            
            # Get robot ID
            robot_id = self._get_robot_id(robot_name)
            if robot_id is None:
                return
            
            # Create result hash from task completion data
            result_data = {
                'task_id': task_id,
                'robot_name': robot_name,
                'completion_time': msg.completion_time.sec,
                'status': 'completed'
            }
            result_hash = hashlib.sha256(
                json.dumps(result_data, sort_keys=True).encode()
            ).hexdigest()
            
            # Submit result to blockchain
            self._submit_result(onchain_task_id, result_hash, robot_id)
            
            self.get_logger().info(f'Submitted result for task {task_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling task completion: {e}')
    
    def _handle_task_failed(self, msg):
        """Handle task failure."""
        try:
            task_id = msg.task_id
            self.get_logger().warn(f'Task {task_id} failed')
            
        except Exception as e:
            self.get_logger().error(f'Error handling task failure: {e}')
    
    def _post_task(self, spec_hash, budget):
        """Post task to blockchain."""
        try:
            # Build transaction
            tx = self.task_escrow.functions.postTask(
                spec_hash,
                budget
            ).build_transaction({
                'from': self.account.address,
                'gas': 200000,
                'gasPrice': self.w3.eth.gas_price,
                'nonce': self.w3.eth.get_transaction_count(self.account.address)
            })
            
            # Sign and send transaction
            signed_tx = self.w3.eth.account.sign_transaction(tx, self.account.key)
            tx_hash = self.w3.eth.send_raw_transaction(signed_tx.rawTransaction)
            
            # Wait for confirmation
            receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
            
            # Extract task ID from event
            # This is simplified - in reality, parse the event logs
            return 1  # Placeholder
            
        except Exception as e:
            self.get_logger().error(f'Error posting task: {e}')
            return None
    
    def _submit_result(self, task_id, result_hash, robot_id):
        """Submit task result to blockchain."""
        try:
            # Create EIP-712 signature
            signature = self.signer.sign_task_result(task_id, result_hash, robot_id)
            
            # Build transaction
            tx = self.task_escrow.functions.submitResult(
                task_id,
                result_hash,
                robot_id,
                signature
            ).build_transaction({
                'from': self.account.address,
                'gas': 200000,
                'gasPrice': self.w3.eth.gas_price,
                'nonce': self.w3.eth.get_transaction_count(self.account.address)
            })
            
            # Sign and send transaction
            signed_tx = self.w3.eth.account.sign_transaction(tx, self.account.key)
            tx_hash = self.w3.eth.send_raw_transaction(signed_tx.rawTransaction)
            
            # Wait for confirmation
            receipt = self.w3.eth.wait_for_transaction_receipt(tx_hash)
            
            self.get_logger().info(f'Result submitted for task {task_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error submitting result: {e}')
    
    def _map_robot(self, robot_name):
        """Map robot name to on-chain robot ID."""
        try:
            # Query robot identity contract
            # This is simplified - in reality, query the contract
            robot_id = len(self.robot_mapping) + 1
            
            self.robot_mapping[robot_name] = robot_id
            
            # Store in database
            self.cursor.execute(
                'INSERT OR REPLACE INTO robot_mapping (robot_name, robot_id) VALUES (?, ?)',
                (robot_name, robot_id)
            )
            self.conn.commit()
            
            self.get_logger().info(f'Mapped robot {robot_name} to ID {robot_id}')
            
        except Exception as e:
            self.get_logger().error(f'Error mapping robot: {e}')
    
    def _store_task_mapping(self, rmf_task_id, onchain_task_id, spec_hash, budget):
        """Store task mapping in database."""
        try:
            self.cursor.execute(
                '''INSERT OR REPLACE INTO task_mapping 
                   (rmf_task_id, onchain_task_id, spec_hash, budget, status) 
                   VALUES (?, ?, ?, ?, ?)''',
                (rmf_task_id, onchain_task_id, spec_hash, budget, 'posted')
            )
            self.conn.commit()
            
        except Exception as e:
            self.get_logger().error(f'Error storing task mapping: {e}')
    
    def _get_onchain_task_id(self, rmf_task_id):
        """Get on-chain task ID from RMF task ID."""
        try:
            self.cursor.execute(
                'SELECT onchain_task_id FROM task_mapping WHERE rmf_task_id = ?',
                (rmf_task_id,)
            )
            result = self.cursor.fetchone()
            return result[0] if result else None
            
        except Exception as e:
            self.get_logger().error(f'Error getting on-chain task ID: {e}')
            return None
    
    def _get_robot_id(self, robot_name):
        """Get robot ID from robot name."""
        try:
            self.cursor.execute(
                'SELECT robot_id FROM robot_mapping WHERE robot_name = ?',
                (robot_name,)
            )
            result = self.cursor.fetchone()
            return result[0] if result else None
            
        except Exception as e:
            self.get_logger().error(f'Error getting robot ID: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    
    bridge_node = DavoBridgeNode()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
