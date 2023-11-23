import pandas as pd
import pymysql
import psycopg2

# Connect to the database
# connection = pymysql.connect(host='@db-chatbot.cvttljzzyn1u.us-east-1.rds.amazonaws.com', user='admin123', password='admin123', db='postgres')
connection = psycopg2.connect("postgresql://admin123:admin123@db-chatbot.cvttljzzyn1u.us-east-1.rds.amazonaws.com:5432/postgres")

cursor = connection.cursor()

# Execute SQL query
query = "SELECT * FROM User"
cursor.execute(query)

# Fetch data
data = cursor.fetchall()

# Transform data using pandas DataFrame
df = pd.DataFrame(data, columns=['Column1'])

# Create Excel file
excel_file_path = '/home/felipe/Documents/GitHub/grupo4/src/excel_export/output_file.xlsx'
df.to_excel(excel_file_path, index=False)

# Close the database connection
cursor.close()
connection.close()
