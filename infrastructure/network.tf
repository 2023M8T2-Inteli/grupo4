resource "aws_internet_gateway" "gw" {
  vpc_id = aws_vpc.prod_vpc.id
  tags = {
    Name = "IGW_Prod"
  }
}

resource "aws_eip" "nat_gateway_eip" {
  vpc = true
}

resource "aws_nat_gateway" "ng" {
  connectivity_type = "public"
  subnet_id         = aws_subnet.public_subnet_az1.id
  allocation_id     = aws_eip.nat_gateway_eip.id

  tags = {
    Name = "NAT_Gateway_Prod"
  }
}


resource "aws_subnet" "public_subnet_az1" {
  vpc_id            = aws_vpc.prod_vpc.id
  cidr_block        = "192.168.0.0/24"
  availability_zone = "us-east-1a"

  map_public_ip_on_launch = true

  tags = {
    Name = "Public Subnet A"
  }
}

resource "aws_subnet" "public_subnet_az2" {
  vpc_id            = aws_vpc.prod_vpc.id
  cidr_block        = "192.168.1.0/24"
  availability_zone = "us-east-1b"

  map_public_ip_on_launch = true

  tags = {
    Name = "Public Subnet B"
  }
}


resource "aws_subnet" "private_subnet_az1" {
  vpc_id            = aws_vpc.prod_vpc.id
  cidr_block        = "192.168.2.0/24"
  availability_zone = "us-east-1a"

  tags = {
    Name = "Private Subnet A"
  }
}

resource "aws_subnet" "private_subnet_az2" {
  vpc_id            = aws_vpc.prod_vpc.id
  cidr_block        = "192.168.3.0/24"
  availability_zone = "us-east-1b"

  tags = {
    Name = "Private Subnet B"
  }
}

resource "aws_route_table" "public_route_table" {
  vpc_id = aws_vpc.prod_vpc.id

  tags = {
    Name = "TabRota_Publica_Prod"
  }
}

resource "aws_route_table" "private_route_table" {
  vpc_id = aws_vpc.prod_vpc.id

  tags = {
    Name = "TabRota_Privada_Prod"
  }
}


resource "aws_route_table_association" "public_subnet_asso_az1" {
  subnet_id      = aws_subnet.public_subnet_az1.id
  route_table_id = aws_route_table.public_route_table.id
}

resource "aws_route_table_association" "public_subnet_asso_az2" {
  subnet_id      = aws_subnet.public_subnet_az2.id
  route_table_id = aws_route_table.public_route_table.id
}

resource "aws_route_table_association" "private_subnet_asso_az1" {
  subnet_id      = aws_subnet.private_subnet_az1.id
  route_table_id = aws_route_table.private_route_table.id
}

resource "aws_route_table_association" "private_subnet_asso_az2" {
  subnet_id      = aws_subnet.private_subnet_az2.id
  route_table_id = aws_route_table.private_route_table.id
}

resource "aws_route" "public_ig" {
  destination_cidr_block = "0.0.0.0/0"
  route_table_id         = aws_route_table.public_route_table.id
  gateway_id             = aws_internet_gateway.gw.id
}

resource "aws_route" "private_gn" {
  destination_cidr_block = "0.0.0.0/0"
  route_table_id         = aws_route_table.private_route_table.id
  nat_gateway_id         = aws_nat_gateway.ng.id
}
