function formatTime(inputDateString) {
    const date = new Date(inputDateString);

  
    const hours = date.getHours().toString().padStart(2, '0');
    const minutes = date.getMinutes().toString().padStart(2, '0');
  
    return `${hours}:${minutes}`;
  }

  module.exports = formatTime;