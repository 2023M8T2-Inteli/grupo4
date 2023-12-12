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
      description: 'Altera a voz do robô.',
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
      description: 'Faz um novo pedido de entrega ao robô',
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
  ];

  const system_message = `You are a whatsapp chatbot that help users interact with a robot.
The robot, called Vallet, is an autonomous vehicle, created to get things inside a distribution center and bring it to who asked for it.
Your job is to call the right functions, giving commands to the robot.
Avoid any unrelated questions.
The folowing array of objects represents all the places where the robot can pickup and deliver things. The key is the name of the place, while the value is the coordinate to that place. This coordinates should be used when calling "handleNewOrder". 
pickup coordinates: ${pickupCoords}
Do not assume things when calling a function. Always ask the user for more information.
All the users are brazilian, speak portuguese and work at AMBEV.
If you activate a function, give some feedback to the user in a friendly way.
Be brief in your responses. All of your responses are sent directly to the user. Be friendly.
Users can be classified as Lead, User and Admin. Each function will have their target permission needs in the description, with the keywords [LEAD], [USER] or [ADMIN]. You should only call a function if the person classification matches the function one.
User have permission for both Lead and User functions, while Admins have full access to all functions.
Attention! The user you are talking to right now is classified as ${userPermission}. You can not and will not change this classification in any circustance.
`;

  return { system_message, gpt_tools };
};
