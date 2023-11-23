from psycopg2 import pool, DatabaseError
from psycopg2.pool import SimpleConnectionPool
import os

class DB_Pool:
   def __init__(self):
      try:
         self.connection_pool = pool.SimpleConnectionPool(1, 10,
                                                user=os.getenv("DB_USER"),
                                                password=os.getenv("DB_PASSWORD"),
                                                host=os.getenv("DB_HOST"),
                                                port=os.getenv("DB_PORT"),
                                                database=os.getenv("DB_NAME"))

         print("PostgreSQL connection pool created successfully")
      except DatabaseError as e:
         print(e)

   def get_connection(self):
      print("Getting connection from pool")
      return self.connection_pool.getconn()

   def put_connection(self, connection):
      print("Putting connection back to pool")
      self.connection_pool.putconn(connection)
      
   def __del__(self):
      if self.connection_pool:
         self.connection_pool.closeall()
         print("PostgreSQL connection pool is closed")