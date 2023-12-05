resource "aws_vpc" "prod_vpc" {
  cidr_block = "192.168.0.0/22"
  enable_dns_hostnames = true
  enable_dns_support = true
  tags = {
    Name = "VPC_Prod"
  }
}