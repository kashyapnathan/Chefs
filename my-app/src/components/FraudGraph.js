import React, { useState, useEffect } from 'react';
import axios from 'axios';

function FraudGraph() {
  const [graphData, setGraphData] = useState([]);

  useEffect(() => {
    const fetchGraphData = async () => {
      try {
        const response = await axios.get('http://localhost:5000/api/get-fraud-graph-data');
        setGraphData(response.data);
      } catch (error) {
        console.error('Error fetching graph data:', error);
      }
    };
    fetchGraphData();
  }, []);

  return (
    <div>
      <h2>Fraud Trends Over Time</h2>
      {/* Implement a graph using your preferred library (like Chart.js, D3.js, etc.)
           using the fetched graphData */}
    </div>
  );
}

export default FraudGraph;