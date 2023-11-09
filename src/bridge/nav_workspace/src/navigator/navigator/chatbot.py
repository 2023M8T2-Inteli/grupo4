import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Chatbot(Node):
    def __init__(self):
        super().__init__('chatbot')
        self.intencoes = {
            r'\b(olá|oi|hey|e aí|oiê)\b': 'cumprimento',
            r'\b(tchau|até logo|até mais)\b': 'despedida',
            r'\b((?:(?:vai|va|ir)(?: para| pro| pra|)(?: (?:o |na |no |))?(?:ponto |)?([a-zA-Z]+)))\b': 'ir_para',
        }

        self.destinos = {
            'casa': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'A': {'x': 1.0, 'y': 0.0, 'z': 0.0},
            'B': {'x': 0.0, 'y': 1.0, 'z': 0.0},
        }

        self.intencao = None
        self.destino = None
        self.publisher_ = self.create_publisher(Pose, 'chatbot_msgs', 10)

    def ir_para(self):
        if self.destino in self.destinos:
            pose_msg = Pose()
            pose_msg.position.x = self.destinos[self.destino]['x']
            pose_msg.position.y = self.destinos[self.destino]['y']
            pose_msg.position.z = self.destinos[self.destino]['z']
            self.get_logger().info(f'Indo para {self.destino} - Coordenadas (x:{pose_msg.position.x}, y:{pose_msg.position.y}, z:{pose_msg.position.z})')
            self.publisher_.publish(pose_msg)
            return f'Indo para {self.destino} - Coordenadas (x:{pose_msg.position.x}, y:{pose_msg.position.y}, z:{pose_msg.position.z})'
        else:
            return 'Destino não encontrado.'

    def terminar_chat(self):
        return 'Até mais!'

    def identificar_intencao(self, texto):
        for expressao, intencao in self.intencoes.items():
            match = re.search(expressao, texto, re.IGNORECASE)
            if match:
                if intencao == 'ir_para':
                    return intencao, match.group(2).upper() if match.group(2) else None
                else:
                    return intencao, None
        return 'desconhecido', None

    def executar_acao(self, intencao, destino):
        if intencao == 'cumprimento':
            return 'Olá! Eu sou o Vallet da cervejaria do futuro. Para onde devo ir?'

        elif intencao == 'despedida':
            return self.terminar_chat()

        elif intencao == 'ir_para':
            if destino:
                return self.ir_para()
            else:
                return 'Destino não especificado.'

        else:
            return 'Desculpe, não entendi o que você disse.'

    def iniciar_conversa(self):
        while True:
            texto = input("Você: ")
            self.intencao, self.destino = self.identificar_intencao(texto)
            resposta = self.executar_acao(self.intencao, self.destino)
            print(resposta)
            if self.intencao == 'despedida':
                exit()

def main(args=None):
    rclpy.init(args=args)
    chatbot = Chatbot()
    chatbot.iniciar_conversa()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
