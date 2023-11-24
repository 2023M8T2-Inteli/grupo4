from langchain.chat_models import ChatOpenAI
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableLambda, RunnablePassthrough
from langchain.document_loaders import TextLoader
from langchain.embeddings.sentence_transformer import SentenceTransformerEmbeddings
from langchain.text_splitter import CharacterTextSplitter
from langchain.vectorstores import Chroma
import gradio as gr
import re
import dotenv
import socketio

dotenv.load_dotenv()

class ChatBot:
    def __init__(self) -> None:
        self.ui = gr.Blocks()
        self.set_up_page()
        self.sio = socketio.Client()
        self.sio.connect('http://10.128.64.39:3000')
        
        template = """Responda a pergunta abaixo com base no contexto para encontrar as coordenadas do lugar. Fique atento para possíveis variações no nome quando o usuário perguntar. 
        Sempre responda na língua que o usuário se comunicar. Sempre dê as coordenadas no formato ([x], [y], [z]) Contexto:
        {context}

        Pergunta: {question}
        """
        prompt = ChatPromptTemplate.from_template(template)

        model = ChatOpenAI(model="gpt-3.5-turbo")

        retriever = self._vectorize_data()

        self.chain = (
            {"context": retriever, "question": RunnablePassthrough()}
            | prompt
            | model
        )

    def set_up_page(self):
        with self.ui:
            self.chatbot = gr.Chatbot()
            self.msg = gr.Textbox()
            self.clear = gr.ClearButton([self.msg, self.chatbot])
            self.msg.submit(self.respond, [self.msg, self.chatbot], [
                            self.msg, self.chatbot])

    def respond(self, message, chat_history):
        bot_message = ""
        for s in self.chain.stream(message):
            bot_message += str(s.content)
        ws_json = self.extrair_coordenadas(bot_message)
        home = {
            'x': 0.0,
            'y': 0.0
        }
        if ws_json:
            print(ws_json)
            self.sio.emit('send_points', home)
            self.sio.emit('send_points', ws_json)
            self.sio.emit('send_points', home)
        chat_history.append((message, bot_message))
        return "", chat_history

    def launch(self):
        self.ui.launch()
    
    def _vectorize_data(self):
        # load the document and split it into chunks
        loader = TextLoader("./items.txt")
        documents = loader.load()

        # split it into chunks
        text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
        docs = text_splitter.split_documents(documents)

        # create the open-source embedding function
        embedding_function = SentenceTransformerEmbeddings(model_name="all-MiniLM-L6-v2")

        # load it into Chroma
        vectorstore = Chroma.from_documents(docs, embedding_function)

        retriever = vectorstore.as_retriever() 

        return retriever

    def extrair_coordenadas(self, frase):
        # O padrão regex para encontrar coordenadas
        padrao = r'\((-?\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+)\)'

        # Procura por correspondências no texto
        correspondencias = re.search(padrao, frase)

        if correspondencias:
            # Retorna um dicionário de x, y, z
            return {
                'x': float(correspondencias.group(1)),
                'y': float(correspondencias.group(2)),
            }
        else:
            return None

if __name__ == "__main__":
    chat = ChatBot()
    chat.launch()