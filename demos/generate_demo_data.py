#!/usr/bin/env python3

"""
Demo data generator for Davo Systems.
Creates test tasks and transactions to demonstrate the system.
"""

import argparse
import json
import hashlib
import time
from web3 import Web3
from eth_account import Account


def main():
    parser = argparse.ArgumentParser(description='Generate demo data for Davo Systems')
    parser.add_argument('--rpc-url', required=True, help='Base Sepolia RPC URL')
    parser.add_argument('--private-key', required=True, help='Private key for transactions')
    parser.add_argument('--task-escrow', required=True, help='TaskEscrow contract address')
    parser.add_argument('--davo-token', required=True, help='DAVO token contract address')
    parser.add_argument('--robot-id', type=int, default=1, help='Robot ID for demo')
    
    args = parser.parse_args()
    
    # Initialize Web3
    w3 = Web3(Web3.HTTPProvider(args.rpc_url))
    if not w3.is_connected():
        print("Error: Failed to connect to RPC endpoint")
        return
    
    # Initialize account
    account = Account.from_key(args.private_key)
    print(f"Demo account: {account.address}")
    
    # Contract ABIs (simplified for demo)
    task_escrow_abi = [
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
            "inputs": [{"name": "taskId", "type": "uint256"}],
            "name": "assignWinner",
            "outputs": [],
            "stateMutability": "nonpayable",
            "type": "function"
        },
        {
            "inputs": [{"name": "taskId", "type": "uint256"}],
            "name": "settle",
            "outputs": [],
            "stateMutability": "nonpayable",
            "type": "function"
        }
    ]
    
    davo_token_abi = [
        {
            "inputs": [
                {"name": "spender", "type": "address"},
                {"name": "amount", "type": "uint256"}
            ],
            "name": "approve",
            "outputs": [{"name": "", "type": "bool"}],
            "stateMutability": "nonpayable",
            "type": "function"
        }
    ]
    
    # Initialize contracts
    task_escrow = w3.eth.contract(
        address=args.task_escrow,
        abi=task_escrow_abi
    )
    
    davo_token = w3.eth.contract(
        address=args.davo_token,
        abi=davo_token_abi
    )
    
    # Generate demo task
    print("Generating demo task...")
    
    # Create task specification
    task_spec = {
        "task_type": "delivery",
        "priority": "normal",
        "start_location": "gate_A1",
        "end_location": "gate_B3",
        "cargo": "passenger_luggage",
        "deadline": int(time.time()) + 3600,  # 1 hour from now
        "robot_requirements": ["autonomous", "cargo_capacity"],
        "estimated_duration": 1800  # 30 minutes
    }
    
    # Hash the task specification
    spec_hash = hashlib.sha256(
        json.dumps(task_spec, sort_keys=True).encode()
    ).hexdigest()
    
    # Set budget (1000 DAVO tokens)
    budget = 1000 * 10**18
    
    print(f"Task specification: {json.dumps(task_spec, indent=2)}")
    print(f"Specification hash: {spec_hash}")
    print(f"Budget: {budget / 10**18} DAVO tokens")
    
    # Approve tokens for TaskEscrow
    print("\nApproving DAVO tokens...")
    
    approve_tx = davo_token.functions.approve(
        args.task_escrow,
        budget
    ).build_transaction({
        'from': account.address,
        'gas': 100000,
        'gasPrice': w3.eth.gas_price,
        'nonce': w3.eth.get_transaction_count(account.address)
    })
    
    signed_approve_tx = w3.eth.account.sign_transaction(approve_tx, account.key)
    approve_hash = w3.eth.send_raw_transaction(signed_approve_tx.rawTransaction)
    approve_receipt = w3.eth.wait_for_transaction_receipt(approve_hash)
    
    print(f"✓ Approval transaction: {approve_hash.hex()}")
    print(f"  Block: {approve_receipt.blockNumber}")
    print(f"  Gas used: {approve_receipt.gasUsed}")
    
    # Post task
    print("\nPosting task to TaskEscrow...")
    
    post_task_tx = task_escrow.functions.postTask(
        spec_hash,
        budget
    ).build_transaction({
        'from': account.address,
        'gas': 200000,
        'gasPrice': w3.eth.gas_price,
        'nonce': w3.eth.get_transaction_count(account.address)
    })
    
    signed_post_tx = w3.eth.account.sign_transaction(post_task_tx, account.key)
    post_hash = w3.eth.send_raw_transaction(signed_post_tx.rawTransaction)
    post_receipt = w3.eth.wait_for_transaction_receipt(post_hash)
    
    print(f"✓ Task posted: {post_hash.hex()}")
    print(f"  Block: {post_receipt.blockNumber}")
    print(f"  Gas used: {post_receipt.gasUsed}")
    
    # Extract task ID from event logs (simplified)
    task_id = 1  # In reality, parse the TaskPosted event
    
    print(f"  Task ID: {task_id}")
    
    # Simulate task completion
    print("\nSimulating task completion...")
    
    # Create result data
    result_data = {
        "task_id": task_id,
        "robot_id": args.robot_id,
        "completion_time": int(time.time()),
        "actual_duration": 1650,  # 27.5 minutes
        "status": "completed",
        "metrics": {
            "distance_traveled": 1250,  # meters
            "energy_consumed": 85,  # percentage
            "cargo_delivered": True,
            "safety_incidents": 0
        }
    }
    
    # Hash the result
    result_hash = hashlib.sha256(
        json.dumps(result_data, sort_keys=True).encode()
    ).hexdigest()
    
    print(f"Result data: {json.dumps(result_data, indent=2)}")
    print(f"Result hash: {result_hash}")
    
    # Wait a bit to simulate task execution
    print("\nWaiting for task execution...")
    time.sleep(5)
    
    # Assign winner (simplified - in reality this would be done after bidding)
    print("\nAssigning task winner...")
    
    assign_tx = task_escrow.functions.assignWinner(task_id).build_transaction({
        'from': account.address,
        'gas': 100000,
        'gasPrice': w3.eth.gas_price,
        'nonce': w3.eth.get_transaction_count(account.address)
    })
    
    signed_assign_tx = w3.eth.account.sign_transaction(assign_tx, account.key)
    assign_hash = w3.eth.send_raw_transaction(signed_assign_tx.rawTransaction)
    assign_receipt = w3.eth.wait_for_transaction_receipt(assign_hash)
    
    print(f"✓ Task assigned: {assign_hash.hex()}")
    print(f"  Block: {assign_receipt.blockNumber}")
    
    # Settle task
    print("\nSettling task...")
    
    settle_tx = task_escrow.functions.settle(task_id).build_transaction({
        'from': account.address,
        'gas': 150000,
        'gasPrice': w3.eth.gas_price,
        'nonce': w3.eth.get_transaction_count(account.address)
    })
    
    signed_settle_tx = w3.eth.account.sign_transaction(settle_tx, account.key)
    settle_hash = w3.eth.send_raw_transaction(signed_settle_tx.rawTransaction)
    settle_receipt = w3.eth.wait_for_transaction_receipt(settle_hash)
    
    print(f"✓ Task settled: {settle_hash.hex()}")
    print(f"  Block: {settle_receipt.blockNumber}")
    print(f"  Gas used: {settle_receipt.gasUsed}")
    
    # Print summary
    print("\n" + "="*50)
    print("DEMO SUMMARY")
    print("="*50)
    print(f"Task ID: {task_id}")
    print(f"Task Type: {task_spec['task_type']}")
    print(f"Budget: {budget / 10**18} DAVO")
    print(f"Robot ID: {args.robot_id}")
    print(f"Status: Completed")
    print(f"Duration: {result_data['metrics']['distance_traveled']}m in {result_data['actual_duration']}s")
    print("\nTransactions:")
    print(f"  Approval: {approve_hash.hex()}")
    print(f"  Post Task: {post_hash.hex()}")
    print(f"  Assign Winner: {assign_hash.hex()}")
    print(f"  Settle: {settle_hash.hex()}")
    print("\nExplorer Links:")
    print(f"  Base Sepolia: https://sepolia.basescan.org/")
    print(f"  TaskEscrow: https://sepolia.basescan.org/address/{args.task_escrow}")
    print(f"  DAVO Token: https://sepolia.basescan.org/address/{args.davo_token}")
    print("="*50)
    
    # Save demo data to file
    demo_data = {
        "task_id": task_id,
        "task_spec": task_spec,
        "spec_hash": spec_hash,
        "result_data": result_data,
        "result_hash": result_hash,
        "transactions": {
            "approval": approve_hash.hex(),
            "post_task": post_hash.hex(),
            "assign_winner": assign_hash.hex(),
            "settle": settle_hash.hex()
        },
        "contracts": {
            "task_escrow": args.task_escrow,
            "davo_token": args.davo_token
        },
        "timestamp": int(time.time())
    }
    
    with open("demo_data.json", "w") as f:
        json.dump(demo_data, f, indent=2)
    
    print(f"\nDemo data saved to: demo_data.json")


if __name__ == "__main__":
    main()
