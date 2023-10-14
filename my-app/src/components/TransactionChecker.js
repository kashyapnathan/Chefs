import React, { useState } from 'react';
import axios from 'axios';

function TransactionChecker() {
  const [transactionData, setTransactionData] = useState({});
  const [fraudCheckResult, setFraudCheckResult] = useState(null);

  const handleInputChange = (e) => {
    setTransactionData({
      ...transactionData,
      [e.target.name]: e.target.value
    });
  };

  const checkFraud = async () => {
    try {
      const response = await axios.post('http://localhost:5000/api/fraud-check', { transactionData });
      setFraudCheckResult(response.data);
    } catch (error) {
      console.error('Error checking fraud:', error);
    }
  };

  return (
    <div>
      <h2>Transaction Checker</h2>
      {/* Input fields to accept transaction data */}
      <input type="text" name="field1" onChange={handleInputChange} placeholder="Field 1" />
      {/* Add other input fields as per your transaction data */}
      <button onClick={checkFraud}>Check for Fraud</button>
      {fraudCheckResult && (
        <p>{fraudCheckResult.is_fraudulent ? "Fraud Detected" : "No Fraud Detected"}</p>
      )}
    </div>
  );
}

export default TransactionChecker;
