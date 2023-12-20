import React from "react";

const DownloadButton = (props) => {
    const downloadCSV = (dataDownload) => {
        const convertToCSV = (dataToCSV) => {
          // Extract headers and rows
          const headers = Object.keys(dataToCSV[0]);
          const rows = dataToCSV.map((row) => headers.map((header) => JSON.stringify(row[header])).join(','));
      
          return ['sep=,', headers.join(','), ...rows].join('\n');
        };
      
        // Convert the data to CSV format
        const csvData = convertToCSV(dataDownload);
      
        // Create a Blob containing the CSV data
        const blob = new Blob([csvData], { type: 'text/csv' });
      
        // Create a download link
        const link = document.createElement('a');
        link.href = URL.createObjectURL(blob);
        link.download = props.filename + ".csv";
      
        // Append the link to the DOM and trigger the download
        document.body.appendChild(link);
        link.click();
      
        // Remove the link from the DOM
        document.body.removeChild(link);
    };

    return (
        <button onClick={() => downloadCSV(props.data, props.filename)} className="border-green-400 border-[1px] text-green-400 rounded-md py-0 px-2">
            BAIXAR
        </button>
      );
    };
    
    export default DownloadButton;
    