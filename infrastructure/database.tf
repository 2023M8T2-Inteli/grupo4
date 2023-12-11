resource "aws_db_subnet_group" "db_subnet_group" {
  name       = "db_subnet_group"
  subnet_ids = [aws_subnet.public_subnet_az1.id, aws_subnet.public_subnet_az2.id]

  tags = {
    Name = "DB_Subnet_Group_Prod"
  }
}

resource "aws_db_instance" "main_postgresql_db" {
  identifier             = "db-chatbot"
  engine                 = "postgres"
  engine_version         = "15.3"
  username               = "admin123"
  db_name                = "postgres"
  instance_class         = "db.t3.micro"
  vpc_security_group_ids = [aws_security_group.db_sg.id]
  allocated_storage      = 20
  multi_az               = false
  password               = "admin123"
  publicly_accessible = true
  skip_final_snapshot = true
  db_subnet_group_name      = aws_db_subnet_group.db_subnet_group.name
}