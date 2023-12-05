variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  #default = "ASIAWUPUSUIEYZEZUYCH"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  #default = "0EcGH+9nUAnZSaQFNeBlnU33627MCV9YPZI5TGrw"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  #default = "FwoGZXIvYXdzEAsaDEB7eQNvQsjnv9D+OSLNAblu0fP8PrzYjPponWEiNGPY1QQMatENkX+Iwz5Gcl9eyMZ3b+FJu2WGAqLEJAIa5BuILrHUyEaOiLDUh6p3/IcaM5KcZWgCfQ2xoBIw+/1pP9MQX+rJ5gAinHR6Uyu2OoKJymRAxGj+fy5jzwxauqnexaX04QLwLqxTO5NH5NTmU/alPJquVCVsci+tY9FX1Mkplpr0J4HTKeybbP6z75r7nLhzBYZarpZjE2n1HfVFbpeAa1O+FviMQbiHWi8TJHIr3Iub80pG/02DsR0ovJK4qwYyLXXUsK2WRh0agCDuft2FiosDHB8bxL51HmNvmvXEY1t5M6bBMVq4tMzPIVUQkg=="
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
