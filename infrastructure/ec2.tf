resource "aws_instance" "chatbot_host" {
  ami           = "ami-0fc5d935ebf8bc3bc"
  instance_type = "t3.medium"
  key_name      = "ec2_key_par"

  subnet_id                   = aws_subnet.public_subnet_az1.id
  associate_public_ip_address = true

  vpc_security_group_ids = [aws_security_group.chatbot_sg.id]

  tags = {
    Name = "Chatbot_Host"
  }
}
