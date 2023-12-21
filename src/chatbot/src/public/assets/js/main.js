let interval;
let callFunction = true;

function fetchQRCode() {
  fetch('http://52.5.70.100:3000/qrcode')
    .then((response) => {
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      return response.json(); // Sempre parseia a resposta como JSON
    })
    .then((data) => {
      if (!data.isAuthenticated) {
        data.qr = data.qr.join('');
        // Mostrar o QR Code se estiver presente no JSON.
        document.getElementById('qrcodeElement').src = "data:image/png;base64," + data.qr;
      } else {
        // Parar de chamar a função se o status for 202 e exibir a mensagem.
        clearInterval(interval);
        callFunction = false;
        document.getElementById('qrcode').src =
          '<h1>Chatbot conectado!</h1>';
        return;
      }
    })
    .catch((error) => {
      console.error('There was a problem with the fetch operation:', error);
      // Trate o erro como achar necessário.
    });
}

// Iniciar a função fetchQRCode imediatamente.
fetchQRCode();
if (callFunction) {
  interval = setInterval(fetchQRCode, 20000); // 20000 milissegundos = 20 segundos
}
