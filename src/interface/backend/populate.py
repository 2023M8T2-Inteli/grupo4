import requests
from faker import Faker
import random

fake = Faker()
users = []
tool_ids = []
points = []

def add_random_users(n):
    for i in range(n):
        url = "http://107.22.195.153:3000/users"
        payload = {
            "name": fake.name(),
            "cellPhone": fake.phone_number(),
        }
        response = requests.request("POST", url, json=payload)
        print(response.json())
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
    url = "http://107.22.195.153:3000/tools"

    for i in range(n):
        payload = {
            "name": random.choice(mockToolNames),
            "tag": fake.word(),
            "minQuantity": fake.random_int(min=1, max=10),
            "maxQuantity": fake.random_int(min=10, max=100),
            "price": fake.random_int(min=100, max=1000),
        }

        response = requests.request("POST", url, json=payload)
        print(response.json())
        tool_ids.append(response.json()["id"])



def add_random_points(n):
    for i in range(n):
        url = "http://107.22.195.153:3000/points"
        payload = {
            "name": fake.company(),
            "pointX": random.uniform(0, 10),
            "pointY": random.uniform(0, 10),
            "pointZ": random.uniform(0, 10),
        }
        response = requests.request("POST", url, json=payload)
        points.append(response.json()["id"])

def add_random_orders(n):
    for i in range(n):
        url = "http://107.22.195.153:3000/orders/queue"
        payload = {
            "toolId": random.choice(tool_ids),
            "userId": random.choice(users),
            "pointId":  random.choice(points),
            "type": random.choice(["In Progress", "Completed"])
        }
        response = requests.request("POST", url, json=payload)
        print(response.json())
        

def delete_all():
    url = "http://107.22.195.153:5000/orders"
    response = requests.request("DELETE", url)
    url = "http://107.22.195.153:5000/tools"
    response = requests.request("DELETE", url)
    url = "http://107.22.195.153:5000/users"
    response = requests.request("DELETE", url)
    url = "http://107.22.195.153:5000/points"
    response = requests.request("DELETE", url)
    


if __name__ == "__main__":
    add_random_tools(10)
    add_random_points(10)
    add_random_orders(10)

