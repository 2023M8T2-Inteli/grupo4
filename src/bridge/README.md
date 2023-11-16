# Atividade 2 - Módulo 8

### Pacotes Mapping:

Os pacotes mapping_<gazebo/turtlebot3> contemplam os scripts necessários para realizar o mapeamento pelo robô em ambiente simulado ou real e se encontram no diretório: `ativ2-m8/src/mapping_<turtlebot3/gazebo>`. 

Para executá-los, juntamente com as suas dependências, é preciso seguir as seguintes instruções:

- Na pasta raíz desse projeto rode o seguinte comando:

```
sudo chmod +x ./mapping_<turtlebot3/gazebo>_zsh.sh
```

> [!IMPORTANT]
> Verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```sudo chmod +x ./mapping_<turtlebot3/gazebo>_bash.sh```. Se atente à escolha do ambiente ao rodar os comandos, por exemplo, caso vc queira rodar no ambiente simulado do gazebo, rode `sudo chmod +x ./mapping_gazebo_bash.sh`.

- Em seguida, no mesmo diretório rode o comando:

```
./mapping_<turtlebot3/gazebo>_zsh.sh
```

O script acima instalará na sua máquina todas as dependências necessárias para executar os nós ROS necessárias para a etapa de mapeamento.

> [!IMPORTANT]
> Mais uma vez verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```./mapping_<turtlebot3/gazebo>_bash.sh```. Se atente à escolha do ambiente ao rodar os comandos, por exemplo, caso vc queira rodar no ambiente simulado do gazebo, rode `./mapping_gazebo_bash.sh`.

- Após isso, em outro terminal, é preciso rodar o comando abaixo para salvar o mapa gerado pela varredura no mapa:
  
```
ros2 run nav2_map_server map_saver_cli -f ./assets/map_<turtlebot3/gazebo>
```

Depois de obter sucesso com os comandos acima, você cumpriu a rotina de mapeamento. Agora basta clicar em CTRL+C para encerrar todos os processos criados pelo script `.sh`

#### Video do sistema de mapeamento que é lançado por um "launch file".
Neste [link]() é possível ver o funcionamento do pacote.

### Pacotes Navigation:

Os pacotes navigation_<turtlebot3/gazebo> se encontram no diretório `ativ2-m8/src/navigation_<turtlebot3/gazebo>`. 

Para executá-los é preciso seguir as seguintes instruções:

- Na pasta raíz desse projeto rode o seguinte comando:

```
sudo chmod +x ./navigation_<turtlebot3/gazebo>_zsh.sh
```

> [!IMPORTANT]
> Verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```sudo chmod +x ./navigation_<turtlebot3/gazebo>_bash.sh```. Se atente à escolha do ambiente ao rodar os comandos, por exemplo, caso vc queira rodar no ambiente simulado do gazebo, rode `sudo chmod +x ./navigation_gazebo_bash.sh`.

- Em seguida, no mesmo diretório, rode o comando:

```
./navigation_<turtlebot3/gazebo>_zsh.sh
```

> [!IMPORTANT]
> Mais uma vez verifique qual a versão do seu shell com o seguinte comando: ```echo $SHELL```, caso ela difira do tipo zsh rode o comando acima com o sufixo bash: ```./navigation_<turtlebot3/gazebo>_bash.sh```. Se atente à escolha do ambiente ao rodar os comandos, por exemplo, caso vc queira rodar no ambiente simulado do gazebo, rode `sudo chmod +x ./navigation_gazebo_bash.sh`.

- Em seguida, na janela do X-Term criada, indique as cordenadas x e y que você deseja que o robô se mova.

Depois de obter sucesso com os comandos acima, você cumpriu a rotina de mapeamento. Agora basta clicar em CTRL+C para encerrar todos os processos criados pelo script `.sh`. 

#### Video da navegação utilizando também um launch file
Neste [link]() é possível ver o funcionamento dos pacotes.
