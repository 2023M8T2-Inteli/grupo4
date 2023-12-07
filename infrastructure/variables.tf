variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ASIAWUPUSUIETUQQJAXD"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ZPNYOgq2L49tI3V36fi/0KVo5ElUutkPAF1bLU7p"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "FwoGZXIvYXdzEE8aDMsMdYLjPfrEiaPuzyLNAZzmbBDAzayghPpJXCOpA2jXpOaXOTMaZ5rPvjIhuCGy+BwKgAGVA1L8vvFB0XhulBIXIZZGwZpeyfTGJQveDxvrGeexUKsFgzE40E7J9ryXmbqlFKKzd90YkJOzSelMs2GrqUoWGbtAy0Alccm1dyH6V6LEUfid0QBz5A31InROb16mKgbOEcXvWr0V2w5hhDP8gFw4rc2W0P2RhtlWfVZg5EmT/dL4vOtsKyUs/l1qNZWJbgjB+6gZJvkLvEGtV7jf2lleXlmm5jJ8wCcoxZfHqwYyLQh+RLK280ezhG2psV2t2Dsxh5kZgnob7RVbGwDgGHx/xxmmE9M1ktzxN9E0Ew=="
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