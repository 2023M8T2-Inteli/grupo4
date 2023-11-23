// components/MyComponent.js
'use client'

import { useEffect } from 'react';

function MyComponent() {
  useEffect(() => {
    const eventSource = new EventSource('/api/expression');

    eventSource.onmessage = (event) => {
      const data = JSON.parse(event.data);
      console.log('Received message:', data.message);
      // Update your UI or state with the received data
    };

    return () => {
      // Cleanup when the component is unmounted
      eventSource.close();
    };
  }, []);

  return <div>Your component content here</div>;
}

export default MyComponent;
