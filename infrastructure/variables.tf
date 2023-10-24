variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  default = "ASIAWUPUSUIE3H3D3OFU"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  default = "xPYZ8DEVFU+Yv/ql3mF+pg0s5Cjd6XcCkreSXRPl"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  default = "FwoGZXIvYXdzECQaDDl6VhfIfwdRsP/KGyLNAS9uCknPVHGRK51pEZx4EtkhOBAiInx5ni3cRHa6J44DpUB7lz/3e34pOtvbi1b1hM21xZ5U08lirUeSalO8tJxo1RJTNgop87Sme4daMyFXRbzw8xxGaI5SRr20IC5No2S19/JtGuXrATSkQIX3EKP2JcCEyWzc7gdO+/bRAjXm5qTN76ObPvkbBVKcoU2/Nhzo93nFqRhiOlFKI8NepqhV1guHFnz1b4FNktziPHdQM4ky/5Cu1d+XHxE32vphxC+WqLhC45VDUbBSs+0oxN3cqQYyLayN71qJSYc/qFeMVUYf5LBYhn899qg8izMW9GrD4o6XxkR230jgnUWGoPSKKQ=="
}

variable "public_subnet_cidrs" {
  type        = list(string)
  description = "Public Subnet CIDR values"
  default     = ["192.168.0.0/24"]
}

variable "private_subnet_cidrs" {
  type        = list(string)
  description = "Private Subnet CIDR values"
  default     = ["192.168.1.0/24"]
}

variable "default_availability_zone" {
  type        = string
  description = "Default availability zone"
  default     = "us-east-1a"
}
