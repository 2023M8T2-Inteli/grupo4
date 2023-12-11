import color from 'picocolors';

export const print = (text: string) => {
  console.log(color.green('◇') + '  ' + text);
};

export const printError = (text: string) => {
  console.log(color.red('◇') + '  ' + text);
};

export const printIntro = () => {
  console.log('');
  console.log(color.bgCyan(color.white(' Whatsapp ChatGPT & DALL-E ')));
  
  console.log(" A Whatsapp bot that uses OpenAI's ChatGPT.");
  
  console.log('');
};

export const printQRCode = (qr: string) => {
  console.log(qr);
  console.log('Scan the QR code above to login to Whatsapp Web...');
};

export const printLoading = () => {
  console.log('Loading...');
};

export const printAuthenticated = () => {
  console.log('Authenticated, session started!');
};

export const printAuthenticationFailure = () => {
  console.log('Authentication failed!');
};
