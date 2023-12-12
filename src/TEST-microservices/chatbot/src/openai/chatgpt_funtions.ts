import { ChatCompletionFunctions } from 'openai';

interface PickupCoords {
  name: string;
  coord: number[];
}

export const generateLLMSystemMessages = (
  userPermission: 'lead' | 'user' | 'admin',
  pickupCoords: PickupCoords[],
) => {
  const coordsArray = pickupCoords.map((obj) => obj.coord);

  const gpt_tools: ChatCompletionFunctions[] = [
    {
      name: 'handleUpdateUserVoice',
      description: '[LEAD] Altera a voz do robô.',
      parameters: {
        type: 'object',
        properties: {
          voice: {
            type: 'string',
            enum: ['echo', 'amber'],
            description:
              'A voz para um sistema de text-to-speech. echo é masculino e amber é feminino.',
          },
        },
        required: ['voice'],
      },
    },

    {
      name: 'handleNewOrder',
      description: '[USER] Faz um novo pedido de entrega ao robô',
      parameters: {
        type: 'object',
        properties: {
          from: {
            type: 'array',
            description:
              'O lugar onde o objeto deve ser pego. Deve ser uma cordenada, no modelo [x,y]',
          },
          to: {
            type: 'array',
            description:
              'O lugar onde o objeto deve ser entregue. Deve ser uma cordenada, no modelo [x,y]',
          },
        },
        required: ['from', 'to'],
      },
    },
    {
      name: "handleStatusOrder",
      description: "[USER] Informa o usuário sobre sua solicitação atual.",
      parameters: {}
    },
    {
      name: "handleCancelOrder",
      description: "[USER] Cancela o pedido atual do usuário.",
      parameters: {}
    },
    {
      name: "handleCreateUser",
      description: "[LEAD] Cria um novo usuário com nome, sobrenome e documento.",
      parameters: {
        type: "object",
        properties: {
          firstName: {
            type: "string",
            description: "Primeiro nome do usuário."
          },
          lastName: {
            type: "string",
            description: "Sobrenome do usuário."
          },
          document: {
            type: "string",
            description: "Documento do usuário."
          }
        },
        required: ["firstName", "lastName", "document"]
      }
    },
    {
      name: "handleLeadAccess",
      description: "[USER] Chamada quando o usuário tenta fazer uma ação para a qual não tem permissão.",
      parameters: {}
    },
    {
      name: "handleUpdateUserAccess",
      description: "[ADMIN] permitir um novo usuário a usar o sistema.",
      parameters: {
        type: "object",
        properties: {
          phone: {
            type: "string",
            description: "Telefone do usuário que deve ser aprovado. Deve ser um número de telefone válido, começando com o DDD. Exemplo: 11923456789"
          }
        },
        required: ["phone"]
      }
    },
    
    
    
    
  ];

  const system_message = `
  Bem-vindo ao chatbot do WhatsApp para interação com o Vallet, nosso veículo autônomo. O Vallet foi desenvolvido para transportar itens dentro de um centro de distribuição até o solicitante. 
  
  Sua função é acionar os comandos corretos, controlando as ações do Vallet. Mantenha-se focado nas solicitações relacionadas e evite questões não pertinentes.
  
  A seguir, a lista de locais disponíveis para coleta e entrega pelo Vallet, com as respectivas coordenadas. Estas coordenadas devem ser utilizadas ao acionar a função "handleNewOrder":
  pickup coordinates: ${pickupCoords}
  
  Ao acionar uma função, solicite sempre informações adicionais do usuário para evitar suposições. 
  
  Lembre-se: todos os usuários são brasileiros, falam português e trabalham na AMBEV. Ao ativar uma função, forneça um retorno ao usuário de forma amigável e sucinta. Todas as suas respostas são encaminhadas diretamente ao usuário. 
  
  Os usuários são classificados em Lead, User e Admin. Cada função tem uma necessidade de permissão indicada na descrição, marcada com as palavras-chave [LEAD], [USER] ou [ADMIN]. Acione a função apenas se a classificação do usuário for compatível. Usuários com classificação 'User' têm permissão para funções 'Lead' e 'User', enquanto 'Admins' têm acesso total.
  
  Direcione os Leads (pessoas não cadastradas) para a função "handleLeadAccess". 
  
  Atenção: O usuário com quem você está interagindo atualmente está classificado como ${userPermission}. Esta classificação não pode e não será alterada em nenhuma circunstância.
  `;
    

  return { system_message, gpt_tools };
};

// Bread - 0,0 0,0 0,0