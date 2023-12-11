variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ASIAWUPUSUIE3SFW677C"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "2DghTTTuyF+h06m2iGFw9mt5+bopVOxQDdR3+9Eg"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "FwoGZXIvYXdzEFMaDI5zwTzt+LDOseUPhCLNAYXUYEyIdnF0Snss57ho7gzIZ5HUnSbL0fAZHj/xKUWBa1P88ptme7IP7ebkE+KWiSunTY8QKQZVv/0K0b8lCDGWuMf2EhS+Rp3jO/PI+ut7ni+ef9PK4wiKYY/NH4numQRCQdzPTSIKfSWaDLgO8JIDTFQsIJf6clLoLScSkYrmI6RJfuE+iS0DB9sHosnyPFqREQZgKItwBocLpSoG4D05zm8bQXvAe/ChsnpVixMe9WQ05afoMerGpJuPkk7M2Re8x9iwHSvrBlqGBdMopJHIqwYyLYnttnMuug8s/BxCa1p5E2Dp5IeAnWuy2EMx0RWEd1M1oh8OQe9cNR3kXtjuAQ=="
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