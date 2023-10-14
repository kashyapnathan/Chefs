from flask import Flask, jsonify, request
from flask_cors import CORS
import h5py
from pymongo import MongoClient

app = Flask(__name__)
CORS(app)  # enabling CORS for all routes

# MongoDB setup
client = MongoClient('your_mongo_db_connection_string')
db = client['your_database_name']

@app.route('/')
def home():
    return "Welcome to Fraud Detection Backend!"

@app.route('/api/fraud-check', methods=['POST'])
def check_fraud():
    transaction_data = request.json.get('transaction_data')

    # Perform fraud check (using ML model or rule-based system - utilizing H5 file)
    is_fraudulent = fraud_check_logic(transaction_data)

    # Formulate response
    response = {
        "is_fraudulent": is_fraudulent,
        "message": "Fraud Detected" if is_fraudulent else "No Fraud Detected"
    }
    return jsonify(response)


@app.route('/api/get-detection-score/<transaction_id>')
def get_detection_score(transaction_id):
    # Logic to fetch and return detection score from MongoDB or H5 file
    # Replace the following logic as per your actual implementation
    score = db.transactions.find_one({"transaction_id": transaction_id})["score"]
    return jsonify({"score": score})

@app.route('/api/get-fraud-graph-data')
def get_fraud_graph_data():
    # Logic to fetch and return data for graphs (like fraud trends over time)
    # Replace the following logic as per your actual implementation
    graph_data = list(db.graph_data.find({}))
    return jsonify(graph_data)

@app.route('/api/get-recent-frauds')
def get_recent_frauds():
    # Logic to fetch and return recent fraud transactions
    # Replace the following logic as per your actual implementation
    recent_frauds = list(db.frauds.find().limit(5))
    return jsonify(recent_frauds)

@app.route('/api/add-transaction', methods=['POST'])
def add_transaction():
    # Logic to add a transaction data to MongoDB
    # Replace the following logic as per your actual implementation
    transaction_data = request.json.get('transaction_data')
    db.transactions.insert_one(transaction_data)
    return jsonify({"status": "Transaction Added"})

@app.route('/api/get-user-transactions/<user_id>')
def get_user_transactions(user_id):
    # Logic to fetch and return all transactions of a specific user
    # Replace the following logic as per your actual implementation
    user_transactions = list(db.transactions.find({"user_id": user_id}))
    return jsonify(user_transactions)

@app.route('/api/get-transaction/<transaction_id>')
def get_transaction(transaction_id):
    # Logic to fetch and return a specific transaction data
    # Replace the following logic as per your actual implementation
    transaction = db.transactions.find_one({"transaction_id": transaction_id})
    return jsonify(transaction)

if __name__ == "__main__":
    app.run(port=5000, debug=True)
