import uuid

class Model:
   def __init__(self, table_name: str, columns: dict[str, type]):
      self.table_name = table_name
      self.columns = columns

   def _check_data(self, data: dict) -> bool:
      for column_name, value_type in data.items():
         if not column_name in self.columns:
            return False
         
         if not isinstance(value_type, self.columns[column_name]):
            return False
         
      return True
   
   def create(self, conn, data):
      print("Checking data...")
      if not self._check_data(data):
         raise ValueError("Invalid data")
      
      try:
         print(f"Inserting into table: {self.table_name}")
         cursor = conn.cursor()

         columns = ["id"]
         for keys, _ in self.columns.items():
            columns.append(keys)

         columns = tuple(columns)

         query = f"INSERT INTO {self.table_name} {columns} VALUES (%s, %s, %s)"

         values = [str(uuid.uuid4())]
         for _, values in data.items():
            values.append(values)
         
         values = tuple(values)

         cursor.execute(query, values)

         conn.commit()

      except Exception as e:
         print(e)
         conn.roolback()

      finally:
         print("Closing cursor and connection")
         cursor.close()
         conn.close()