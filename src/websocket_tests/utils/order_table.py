from .model import Model
from datetime import datetime

class Order(Model):
   def __init__(self):
      super().__init__("Order", {
         "code": int,
         "type": str,
         "toolid": str, # Test: 0x01a
         "userId": str, # Test: 0x01a
         "pointId": str, # Test: 0x01a
      })