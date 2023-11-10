import re

def extrair_coordenadas(mensagem):
    # Expressão regular para encontrar coordenadas: "x" seguido opcionalmente por ":" ou espaço e então um número,
    # o mesmo para "y" e "z". Os números podem ser inteiros ou decimais.
    # Exemplo de padrão: "x:123", "x 123", "x123", "y:456", "y 456", "y456", "z:789", "z 789", "z789"
    padrao = r'[xX][: ]*(-?\d+[.,]?\d*)\s*[yY][: ]*(-?\d+[.,]?\d*)\s*[zZ][: ]*(-?\d+[.,]?\d*)'
    # Buscar pelo padrão na mensagem
    resultado = re.search(padrao, mensagem)
    
    # Se o padrão for encontrado, extrair os valores
    if resultado:
        x, y, z = resultado.groups()
        # Convertendo os valores para float
        return float(x), float(y), float(z)
    else:
        # Se não encontrar o padrão, retornar um erro ou um valor padrão
        return None, None, None

# Testando a função
mensagens = [
    "O robô deve ir para x: 100.5 y: 200.3 z: 300.7",
    "Posição inicial x 0 y 0 z 0",
    "Configure as coordenadas para x100 y220 z-35",
    "Mover para x-100.2 y +200 z50",
    "Leve o robô para x400 y300 z300",
    "Alvo em x400y300z300 corretamente"
]

for msg in mensagens:
    coordenadas = extrair_coordenadas(msg)
    if coordenadas != (None, None, None):
        print(f"Coordenadas extraídas: x={coordenadas[0]}, y={coordenadas[1]}, z={coordenadas[2]}")
    else:
        print(f"Não foi possível extrair coordenadas da mensagem: '{msg}'")
