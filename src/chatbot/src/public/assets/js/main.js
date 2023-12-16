let interval;
let callFunction = true;

function fetchQRCode() {
  fetch('http://localhost:3000/qrcode')
    .then((response) => {
      if (!response.ok) {
        throw new Error('Network response was not ok');
      }
      return response.json(); // Sempre parseia a resposta como JSON
    })
    .then((data) => {
      console.log('Response data:', data);

      if (data.message !== 'User is authenticated') {
        // Mostrar o QR Code se estiver presente no JSON.
        document.getElementById('qrcode').src = data.qrCodeUrl;
      } else {
        // Parar de chamar a função se o status for 202 e exibir a mensagem.
        clearInterval(interval);
        callFunction = false;
        console.log('Chatbot conectado!');
        document.getElementById('qrcode').innerHTML =
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