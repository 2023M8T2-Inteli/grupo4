variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  //default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  //default = ""
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  //default = ""
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  //default = ""
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