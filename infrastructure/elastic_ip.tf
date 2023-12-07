resource "aws_eip" "chatbot_host_eip" {
  instance = aws_instance.chatbot_host.id
  vpc      = true
}
