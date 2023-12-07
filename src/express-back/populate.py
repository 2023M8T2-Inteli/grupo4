import requests
from faker import Faker

fake = Faker()
users = []
tool_ids = []
points = []

def add_random_users(n):
    for i in range(n):
        url = "http://localhost:5000/users"
        payload = {
            "name": fake.name(),
            "cellPhone": fake.phone_number(),
        }
        response = requests.request("POST", url, json=payload)
        users.append(response.json()["id"])


def add_random_tools(n):
    mockToolNames = [
        "Chave de Fenda",
        "Martelo",
        "Alicate",
        "Serrote",
        "Chave Inglesa",
        "Trena",
        "Furadeira",
        "Serra Circular",
        "Nível a Laser",
        "Lanterna",
        "Chave de Boca",
        "Talhadeira",
        "Grampeador",
        "Esmerilhadeira",
        "Sugador de Solda",
        "Chave de Grifo",
        "Tesoura de Aviação",
        "Lima",
        "Escova de Aço",
    ]
    url = "http://localhost:5000/tools"

    for i in range(n):
        payload = {
            "name": mockToolNames.choice(),
            "tag": fake.word(),
            "minQuantity": fake.random_int(min=1, max=10),
            "maxQuantity": fake.random_int(min=10, max=100),
            "price": fake.random_int(min=100, max=1000),
        }

        response = requests.request("POST", url, json=payload)
        tool_ids.append(response.json()["id"])



def add_random_points(n):
    for i in range(n):
        url = "http://localhost:5000/points"
        payload = {
            "name": Faker.company(),
            "pointX": Faker.random_float(min=1, max=10),
            "pointY": Faker.random_float(min=1, max=10),
            "pointZ": Faker.random_float(min=1, max=10),
        }
        response = requests.request("POST", url, json=payload)
        points.append(response.json()["id"])

def add_random_orders(n):
    for i in n:
        url = "http://localhost:5000/orders/queue"
        payload = {
            "toolId": tool_ids.choice(),
            "userId": users.choice(),
            "pointId": points.choice(),
            "type": ["In Progress", "Complete"].choice()
        }
        response = requests.request("POST", url, json=payload)
        

def delete_all():
    url = "http://localhost:5000/orders"
    response = requests.request("DELETE", url)
    url = "http://localhost:5000/tools"
    response = requests.request("DELETE", url)
    url = "http://localhost:5000/users"
    response = requests.request("DELETE", url)
    url = "http://localhost:5000/points"
    response = requests.request("DELETE", url)
    


if __name__ == "__main__":
    add_random_users(10)
