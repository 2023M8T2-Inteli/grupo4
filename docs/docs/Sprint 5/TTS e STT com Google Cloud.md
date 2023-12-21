# TTS com Google Cloud

TTS, ou "Text-to-Speech", refere-se à tecnologia que converte texto escrito em fala. É uma ferramenta importante na acessibilidade digital, permitindo que pessoas com deficiências visuais ou problemas de leitura acessem conteúdo escrito. A tecnologia TTS utiliza vozes sintéticas, que podem variar em naturalidade e expressividade, dependendo do avanço da tecnologia utilizada.

Essencialmente, o TTS envolve duas etapas principais:

1. **Processamento de Texto:** O texto é analisado e convertido em uma forma estruturada que a máquina pode entender. Isso inclui a interpretação de pontuação, números e abreviações, além de determinar a pronúncia correta das palavras.

2. **Síntese de Fala:** Nesta etapa, o texto estruturado é transformado em som. Isso é feito através de algoritmos que geram ondas sonoras que imitam a fala humana. Esses algoritmos podem ser baseados em regras pré-definidas ou em aprendizado de máquina, sendo que os mais avançados utilizam técnicas de inteligência artificial para produzir uma fala mais natural e expressiva.

O TTS é amplamente utilizado em assistentes virtuais, leitores de e-books, sistemas de navegação veicular, e em diversas outras aplicações que buscam tornar a interação homem-máquina mais natural e acessível.

## Google Cloud
Nesse projeto, foi utilizado a API do Google Cloud, que é uma API paga, mas que possui um trial de 300 dólares para novos usuários. Para utilizar a API, é necessário criar um projeto no Google Cloud e ativar a API Text-to-Speech. Após isso, é necessário criar uma conta de serviço e baixar o arquivo JSON com as credenciais. Esse arquivo deve ser colocado na pasta `src/chatbot` com o nome `googlespeech.json` ou as credenciais podem ser colocadas no arquivo `.env` com as seguintes variáveis.

### Exemplo de arquivo `.env`:

Abaixo está um exemplo de arquivo `.env` com as variáveis necessárias para utilizar a API do Google Cloud. As variáveis devem ser preenchidas com os valores do arquivo JSON baixado.

```env
# Chave privada da conta de serviço GCP
PRIVATE_KEY=""

# Tipo de conta de serviço
TYPE="service_account"

# ID do projeto no Google Cloud Platform
PROJECT_ID=""

# ID da chave privada
PRIVATE_KEY_ID=""

# E-mail da conta de serviço no GCP
CLIENT_EMAIL=""

# ID do cliente
CLIENT_ID=""

# URI de autenticação
AUTH_URI="https://accounts.google.com/o/oauth2/auth"

# URI do token
TOKEN_URI="https://oauth2.googleapis.com/token"

# URL do certificado do provedor de autenticação
AUTH_PROVIDER_X509_CERT_URL="https://www.googleapis.com/oauth2/v1/certs"

# URL do certificado X509 do cliente
CLIENT_X509_CERT_URL=""

# Domínio de referência
UNIVERSE_DOMAIN="googleapis.com"

```

## Integração do Chatbot com o TTS

A integração do TTS com o chatbot foi feita através da utilização da lib `gtts` (Google Text-to-Speech). Essa lib é uma interface para a API do Google Cloud, que permite a conversão de texto em fala. A lib `gtts` foi utilizada para implementar a classe `TextToSpeechClient`, que é responsável por realizar a conversão de texto em fala. Essa classe é utilizada pelo chatbot para gerar a resposta vocal do chatbot.


## Justificativa

A implementação do TTS (Text-to-Speech) no chatbot é fundamental para proporcionar uma interação mais fluída e acessível com o usuário. Optamos por substituir o TTS da OpenAI pelo TTS do Google Cloud visando aprimorar a qualidade vocal do chatbot. Observamos que o TTS da OpenAI, apesar de eficiente, não alcança o mesmo nível de naturalidade e expressividade oferecido pelo TTS do Google Cloud. Uma limitação significativa do TTS da OpenAI é a ausência de suporte para o idioma português. Isso resultava em uma pronúncia com sotaque estrangeiro no chatbot, potencialmente prejudicando a compreensão por parte dos usuários de língua portuguesa.

Ao adotar o TTS do Google Cloud, estamos não só elevando a qualidade da expressão vocal do chatbot, mas também incorporando a capacidade de utilizar vozes em português. Essa mudança é estratégica para assegurar que a interação do usuário com o chatbot seja o mais natural e compreensível possível, especialmente para o público que se comunica em português.

## Conclusão

A integração do TTS (Text-to-Speech) do Google Cloud em nosso chatbot representou um avanço significativo. Com essa mudança, observamos uma melhoria notável na qualidade da expressão vocal do chatbot. Além disso, a implementação trouxe a capacidade de utilização de vozes em português, um recurso essencial para a comunicação efetiva com o público de língua portuguesa. A adoção do TTS do Google Cloud é um marco importante no desenvolvimento do chatbot, visando enriquecer a experiência do usuário, especialmente para aqueles que necessitam de uma interação mais acessível e intuitiva.


