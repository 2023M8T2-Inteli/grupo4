variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ASIAWUPUSUIEWUALVFM5"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ffDUTg/sZaNxjh9ll9kXaPmI5F4EstWg+JeU0Ilu"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "FwoGZXIvYXdzECMaDJ8I7kI2v+ayEKiHhSLNARVirOTiPGwewgDLJ4w/YNVP8ySRVYbAHb34bczWd3eayl+EQKj9Tlqt+gCQQ+BPQVa/VcAlb9xKS7WAmAp5Oc5M9Z7YgVsB3MY8L4xOfqj2+7lqGGQwGS+Z569wmhbe+Ulzi0pZ5A+9UqWKbbW4TW8FvheKSjdArbzOMiw5oy/wQb/Lmf5WORIIynVHKZzwDr+XbYhz2vj735SZLvmAX0zVexBzO46YT7zwL3vnpGaov9F0+epKQRYjAUT23cGD4/L4H+rQYIz4rsnYIr8ouca9qwYyLSSWx8Ep07z/USPl7THA71ruNkh+fK1Xq9YqdHselTXGXuZtfONTQjOO/6accQ=="
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
