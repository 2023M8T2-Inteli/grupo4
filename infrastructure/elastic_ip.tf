resource "aws_eip" "chatbot_host_eip" {
  instance = aws_instance.chatbot_host.id
  vpc      = true
}

resource "aws_eip" "bridge_host_eip" {
  instance = aws_instance.bridge_host.id
  vpc      = true
}

resource "aws_eip" "interface_host_eip" {
  instance = aws_instance.interface_host.id
  vpc      = true
}