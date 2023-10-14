import React, { useState, useEffect } from 'react';
import axios from 'axios';

function DetectionScoreViewer({ transactionId }) {
  const [score, setScore] = useState(null);

  useEffect(() => {
    const fetchDetectionScore = async () => {
      try {
        const response = await axios.get(`http://localhost:5000/api/get-detection-score/${transactionId}`);
        setScore(response.data.score);
      } catch (error) {
        console.error('Error fetching detection score:', error);
      }
    };
    fetchDetectionScore();
  }, [transactionId]);

  return (
    <div>
      <h2>Detection Score</h2>
      {score !== null ? (
        <p>Detection Score: {score}</p>
      ) : (
        <p>Loading score...</p>
      )}
    </div>
  );
}

export default DetectionScoreViewer;
