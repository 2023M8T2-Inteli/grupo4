![activitie](https://img.shields.io/github/commit-activity/w/2023M8T2-Inteli/grupo4?style=plastic)
![license](https://img.shields.io/github/license/2023M8T2-Inteli/grupo4?style=plastic)
![issues](https://img.shields.io/github/issues/2023M8T2-Inteli/grupo4?style=plastic)
[![Static Badge](https://img.shields.io/badge/Documenta%C3%A7%C3%A3o%3A%20online-white?logo=docusaurus&logoColor=white&color=%23014007?style=plastic)](https://2023m8t2-inteli.github.io/grupo4/)

# Grupo 4

# Inteli - Instituto de Tecnologia e Liderança

<p align="center">
<a href= "https://www.inteli.edu.br/"><img src="docs/static/img/inteli.png" alt="Inteli - Instituto de Tecnologia e Liderança" border="0"></a>
</p>

## The Manobristas

<p align="center">
<img src="docs/static/img/logo.png" alt="Logo grupo" border="0">
</p>

## :student: Integrantes:

- [Elisa Flemer](https://www.linkedin.com/in/elisaflemer)
- [Felipe Campos](http://www.linkedin.com/in/felipe-pereira-campos-250aa2231)
- [Gabriela Barretto](https://www.linkedin.com/in/gabriela-barretto-dados)
- [Gustavo Ferreira de Oliveira](https://www.linkedin.com/in/gustavo-ferreira-oliveira)
- [Henrique Marlon Conceição Santos](https://www.linkedin.com/in/henriquemarlon/)
- [Henrique Matias](https://www.linkedin.com/in/henriquelfmatias/)
- [Paulo Presa Evangelista](https://www.linkedin.com/in/paulo-evangelista/)

## :teacher: Professores:

### Orientador(a)

- [Murilo Zanini de Carvalho](https://www.linkedin.com/in/murilo-zanini-de-carvalho-0980415b/)

### Instrutores

- [Rodrigo Mangoni Nicola](https://www.linkedin.com/in/rodrigo-mangoni-nicola-537027158/)
- [Ricardo José Missori](https://www.linkedin.com/in/ricardo-jos%C3%A9-missori/)
- [Lisane Valdo](https://www.linkedin.com/in/lisane-valdo/)
- [Guilherme Cestari](https://www.linkedin.com/in/gui-cestari/)

## 📝 Descrição

O projeto tem como objetivo a construção de um robô delivery para a Ambev, utilizando o TurtleBot3, a fim de auxiliar os colaboradores da Ambev que trabalham no setor de almoxarifado e os técnicos que precisem de peças do almoxarifado. Visa-se agilizar e facilitar o processo de retirada de ferramentas para ambas partes, por meio de um sistema que registra os pedidos feitos por meio de um chatbot, os retira no almoxarifado e leva até a estação de trabalho do técnico.

## 📝 Documentação do Projeto

A documentação do projeto pode ser consultada na pasta `docs` do repositório.

### Está procurando a documentação?

[![Static Badge](https://img.shields.io/badge/Acesse%20a%20documenta%C3%A7%C3%A3o-green?logo=docusaurus&logoColor=white)](https://2023m8t2-inteli.github.io/grupo4/)

## 📁 Estrutura de pastas

Dentre os arquivos presentes na raiz do projeto, definem-se:

```
.
├── docs
│   ├── docs
│   │   ├── 1. Contexto da Indústria
│   │   ├── 2. Proposta de Solução
│   │   ├── 3. Desenvolvimento da Solução
│   │   ├── 4. Manual de Usuário
│   │   └── Referências
│   ├── src
│   │   ├── components
│   │   ├── css
│   │   └── pages
│   └── static
│   │   ├── files
│   │   ├── img
│   │   └── video
├── infrastructure
├── src
│   ├── admin-interface
│   │   ├── next
│   │   └── app
│   ├── backend
│   ├── bridge
│   │   ├── assets
│   │   ├── src
│   │   │   ├── vallet
│   │   │   ├── vallet_cpp
│   │   │   └── vallet_msgs
│   │   ├── vallet_scripts
│   │   └── Dockerfile
│   ├── chatbot
│   │   ├── node_modules
│   │   ├── src
│   │   │   ├── cli
│   │   │   ├── commands
│   │   │   ├── handlers
│   │   │   ├── messages
│   │   │   ├── models
│   │   │   ├── prisma
│   │   │   ├── providers
│   │   │   ├── types
│   │   │   └── .env
│   │   └── Dockerfile
│   ├── express-back
│   ├── interfaces
│   │   ├── admin
│   │   │   ├── next
│   │   │   ├── app
│   │   │   │   ├── components
│   │   │   │   ├── locations
│   │   │   │   ├── qrcode
│   │   │   │   ├── tools
│   │   │   │   ├── users
│   │   │   │   └── utils
│   │   │   ├── public
│   │   │   ├── .env
│   │   │   └── Dockerfile
│   │   ├── backend
│   │   │   ├── prisma
│   │   │   ├── routes
│   │   │   ├── services
│   │   │   ├── .env
│   │   │   └── Dockerfile
│   │   └── robot
│   │   │   ├── app
│   │   │   ├── public
│   │   │   └── Dockerfile
│   ├── TEST-microservices
│   │   │   ├── chatbot
│   │   │   │   ├── src
│   │   │   │   |   ├── AI
│   │   │   │   |   ├── handler
│   │   │   │   |   ├── prisma
│   │   │   │   |   ├── websockets
│   │   │   │   |   └── whatsapp
│   │   │   │   └── Dockerfile
│   │   │   ├── gateway
│   │   │   │   └── src
│   │   │   └── Dockerfile
└── README.md
```

## 📋 Licença/License

<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1"><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1"><p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/Spidus/Teste_Final_1">MODELO GIT INTELI</a> by <a rel="cc:attributionURL dct:creator" property="cc:attributionName" href="https://www.yggbrasil.com.br/vr">Inteli, Elisa Flemer, Felipe Campos, Gabriela Barretto, Gustavo Ferreira, Gustavo Pereira, Henrique Marlon, Henrique Matias e Paulo Evangelista.</a> is licensed under <a href="http://creativecommons.org/licenses/by/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">Attribution 4.0 International</a>.</p>

## 🎓 Referências

Aqui estão as referências usadas no projeto:

1. [https://creativecommons.org/share-your-work/](https://creativecommons.org/share-your-work/)
2. [https://aws.amazon.com/pt/getting-started/](https://aws.amazon.com/pt/getting-started/)
3. [https://www.prisma.io/docs/](https://www.prisma.io/docs/)
