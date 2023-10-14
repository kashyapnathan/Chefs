import React, { useState, useEffect } from 'react';
import axios from 'axios';

function RecentFrauds() {
  const [recentFrauds, setRecentFrauds] = useState([]);

  useEffect(() => {
    const fetchRecentFrauds = async () => {
      try {
        const response = await axios.get('http://localhost:5000/api/get-recent-frauds');
        setRecentFrauds(response.data);
      } catch (error) {
        console.error('Error fetching recent frauds:', error);
      }
    };
    fetchRecentFrauds();
  }, []);

  return (
    <div>
      <h2>Recent Frauds</h2>
      <ul>
        {recentFrauds.map((fraud, index) => (
          <li key={index}>{fraud.transaction_id}</li>  // Adjust as per your data structure
        ))}
      </ul>
    </div>
  );
}

export default RecentFrauds;
