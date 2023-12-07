variable "AWS_REGION" {
  type        = string
  description = "AWS Region"
  default     = "us-east-1"
}
variable "AWS_ACCESS_KEY_ID" {
  type        = string
  description = "AWS Access Key ID"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "ASIAWUPUSUIE52E5U52M"
}
variable "AWS_SECRET_ACCESS_KEY" {
  type        = string
  description = "AWS Secret Access Key"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "mKL7zQQ1JW0x3Vatt3py3VnNznjMnD15o2YDuxIb"
}

variable "AWS_SESSION_TOKEN" {
  type        = string
  description = "AWS Session Token"
  //O Padrão default é para subir a infraestrutura na AWS de forma manual
  default = "FwoGZXIvYXdzEEMaDObah7IDeEpk29S2dyLNASJWMpaZPzjczSVSedtHstnTucWOr2HOGbOgEu8BS19WsRq8gBYBvTFr8jnFAOQ0gDtPLiBwdpZSDDEnX9eqq8AhMklX5bQuWaC683dZb9N+s4wxOI2N+Do0afgL1AJNm0YaBSkP+ZJTahhnBR72WWyMNLOBx3W5XKqhKeJhUMvolN2FeBgisDbgejMwwa8n2Tp78lMqWNRVcuc45niBQFK17MJ3uarBWHrI1Zw80NzJxkd8b8QdEMRQ3iJ45nLsZzLM82HZM7sm/zaOmRIo8bXEqwYyLTsQU6tD/6LUxWbWv9DFDlYRYr3t9Zk0F1r+9fOZf/w/w44EONfczmg6qkB4Gw=="
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