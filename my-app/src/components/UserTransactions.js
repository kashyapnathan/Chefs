import React, { useState, useEffect } from 'react';
import axios from 'axios';

function UserTransactions({ userId }) {
  const [userTransactions, setUserTransactions] = useState([]);

  useEffect(() => {
    const fetchUserTransactions = async () => {
      try {
        const response = await axios.get(`http://localhost:5000/api/get-user-transactions/${userId}`);
        setUserTransactions(response.data);
      } catch (error) {
        console.error('Error fetching user transactions:', error);
      }
    };
    fetchUserTransactions();
  }, [userId]);

  return (
    <div>
      <h2>User Transactions</h2>
      <ul>   
        {userTransactions.map((transaction, index) => (
          <li key={index}>{transaction.transaction_id}</li>  // Adjust as per your data structure
        ))}
      </ul>
    </div>
  );
}

export default UserTransactions;