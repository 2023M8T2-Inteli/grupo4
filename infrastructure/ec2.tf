resource "aws_instance" "chatbot_host" {
  ami           = "ami-0fc5d935ebf8bc3bc"
  instance_type = "t3.medium"
  key_name      = "ec2_key_par"

  subnet_id              = aws_subnet.public_subnet_az1.id
  vpc_security_group_ids = [aws_security_group.chatbot_sg.id]

  ebs_block_device {
    device_name           = "/dev/sda1"
    volume_size           = 16
    volume_type           = "gp2"
    delete_on_termination = true
  }

  tags = {
    Name = "Chatbot_Host"
  }
}

resource "aws_instance" "bridge_host" {
  ami           = "ami-0fc5d935ebf8bc3bc"
  instance_type = "t3.medium"
  key_name      = "ec2_key_par"

  subnet_id              = aws_subnet.public_subnet_az1.id
  vpc_security_group_ids = [aws_security_group.bridge_sg.id]

  ebs_block_device {
    device_name           = "/dev/sda1"
    volume_size           = 16
    volume_type           = "gp2"
    delete_on_termination = true
  }

  tags = {
    Name = "Bridge_Host"
  }
}

resource "aws_instance" "interface_host" {
  ami           = "ami-0fc5d935ebf8bc3bc"
  instance_type = "t3.medium"
  key_name      = "ec2_key_par"

  subnet_id              = aws_subnet.public_subnet_az1.id
  vpc_security_group_ids = [aws_security_group.interface_sg.id]

  ebs_block_device {
    device_name           = "/dev/sda1"
    volume_size           = 16
    volume_type           = "gp2"
    delete_on_termination = true
  }

  tags = {
    Name = "Interface_Host"
  }
}