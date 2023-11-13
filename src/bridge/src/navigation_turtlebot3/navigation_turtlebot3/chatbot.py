import re
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Chatbot(Node):
    def __init__(self):
        super().__init__('chatbot')

        self.intencoes = {
            r'(olá|oi|hey|e aí|oiê)': 'cumprimento',
            r'(tchau|até logo|até mais)': 'despedida',
            r'\b(s?e?s?k?o?l)\b': 'skol',
            r'br[aa]?mm?a|brahma': 'brahma',
            r'ze|zé|zé delivery|delivery': 'ze delivery',
            r'b[ea]ts?|bats|beets|bites|beates|bates|betes|beats': 'beats',
            r'almo?x(?:erif(?:ado)?|arif(?:e|ado)?|cherif(?:ado|edo)?)': 'almoxarifado'
        }


        self.destinos = {
            'almoxarifado': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'beats': {'x': 1.26, 'y': 0.0, 'z': 0.0},
            'brahma': {'x': 1.22, 'y': -1.55, 'z': 0.0},
            'ze delivery': {'x': 0.62, 'y': -0.81, 'z': 0.0},
            'skol': {'x': 0.0, 'y': -1.55, 'z': 0.0},
        }

        self.intencao = None
        self.destino = None
        self.publisher_ = self.create_publisher(Pose, '/enqueue', 10)

    def ir_para(self):
        pose_msg = Pose()
        pose_msg.position.x = self.destino['x']
        pose_msg.position.y = self.destino['y']
        pose_msg.position.z = self.destino['z']
        self.publisher_.publish(pose_msg)
        print(f'OK! Vou para {self.intencao} - Coordenadas (x:{pose_msg.position.x}, y:{pose_msg.position.y}, z:{pose_msg.position.z})')

    def terminar_chat(self):
        print('Até mais!')
        exit()

    def identificar_intencao(self, texto):
        for expressao, intencao in self.intencoes.items():
            match = re.search(expressao, texto, re.IGNORECASE)
            if match:
                return intencao
        return None
      

    def executar_acao(self, intencao):
        if intencao:
            if intencao == 'cumprimento':
                print('Olá! Eu sou o Vallet da cervejaria do futuro. Para onde devo ir?')

            elif intencao == 'despedida':
                return self.terminar_chat()
            
            else:
                self.destino = self.destinos[self.intencao]
                self.ir_para()

        else:
            print('Desculpe, não entendi o que você disse.')

    def iniciar_conversa(self):
        while True:
            texto = input("Você: ")
            self.intencao = self.identificar_intencao(texto)
            self.executar_acao(self.intencao)
            

def main(args=None):
    rclpy.init(args=args)
    chatbot = Chatbot()
    chatbot.iniciar_conversa()
    rclpy.shutdown()

if __name__ == '__main__':
    main()