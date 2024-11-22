Documentação da Biblioteca para MicroPython do VFD CFW500 da WEG
1. Introdução
Esta documentação visa orientar o desenvolvimento e a utilização de uma biblioteca em MicroPython para controlar o Variador de Frequência (VFD) CFW500 da WEG. A biblioteca facilita a comunicação via protocolo Modbus RTU entre o Raspberry Pi Pico e o VFD, permitindo o controle preciso da velocidade do motor em RPM.

2. Requisitos
Antes de iniciar, certifique-se de que os seguintes componentes e softwares estão disponíveis:

Hardware
Raspberry Pi Pico
VFD CFW500 da WEG
Transceptor RS485 (por exemplo, MAX485)
Cabos de conexão (UART1: GPIO4 & GPIO5 para comunicação Modbus RTU)
Fonte de alimentação adequada para o VFD e o Raspberry Pi Pico
Software
MicroPython firmware instalado no Raspberry Pi Pico
Thonny IDE para programação e upload de scripts
Biblioteca umodbus para comunicação Modbus RTU (compatível com MicroPython)
3. Instalação
3.1. Configuração do Raspberry Pi Pico
Instale o MicroPython no Raspberry Pi Pico:

Baixe a imagem mais recente do MicroPython para Raspberry Pi Pico.
Utilize o Thonny IDE para carregar o firmware no Pico.
Configure a Biblioteca umodbus:

Baixe a biblioteca umodbus compatível com MicroPython.
Carregue a biblioteca no Raspberry Pi Pico utilizando o Thonny.
3.2. Conexões de Hardware
Conecte o UART1 do Pico ao VFD CFW500:

TX (GPIO4) → RX do RS485 Transceiver
RX (GPIO5) → TX do RS485 Transceiver
Conecte o Pino DE/RE:

GPIO2 do Pico → Pino DE/RE do Transceiver RS485
Alimentação:

Conecte o Raspberry Pi Pico ao computador via USB para programação e comunicação serial.
Alimente o VFD conforme especificações do fabricante.
4. Configuração
4.1. Parâmetros do Inversor (CFW500)
Ajuste os seguintes parâmetros no menu de configuração do VFD para garantir que a unidade de engenharia seja RPM:

P0208 = 1700 RPM (Escala da Referência)
P0209 = 3 = RPM (Unidade de Engenharia da Referência)
P0210 = 0 (Forma de Indicação da Referência – inteiro sem casa decimal)
Observação: Consulte o manual do CFW500 para detalhes sobre como acessar e modificar esses parâmetros.

4.2. Configuração do Código (main.py)
Certifique-se de que o arquivo main.py esteja corretamente configurado com os parâmetros UART e endereços Modbus apropriados conforme detalhado no código fornecido anteriormente.

5. Uso Básico
Após a configuração, o uso básico envolve iniciar o programa no Raspberry Pi Pico e interagir com a CLI para controlar o VFD.

5.1. Iniciar o Programa
Abra o Thonny IDE.
Conecte o Raspberry Pi Pico via USB.
Carregue o arquivo main.py no Pico.
Clique em Run para iniciar o programa.
O manual de instruções será exibido automaticamente.
5.2. Comandos Disponíveis
start [rpm]

Descrição: Inicia o motor com a velocidade especificada em RPM.
Exemplo: start 1000
stop

Descrição: Para o motor.
Exemplo: stop
reverse [rpm]

Descrição: Inverte a direção do motor com a velocidade especificada em RPM.
Exemplo: reverse 1000
set_speed [rpm]

Descrição: Altera a referência de velocidade do motor para o valor especificado em RPM.
Exemplo: set_speed 1500
read_speed

Descrição: Lê e exibe a velocidade atual do motor em RPM.
Exemplo: read_speed
status

Descrição: Verifica e exibe o status atual do inversor.
Exemplo: status
reset_fault

Descrição: Reseta quaisquer falhas no inversor.
Exemplo: reset_fault
set_verbose [0-4]

Descrição: Define o nível de verbosidade das mensagens de log.
Exemplo: set_verbose 3
read_max_rpm

Descrição: Lê o valor máximo de RPM configurado no inversor.
Exemplo: read_max_rpm
help

Descrição: Exibe o manual de instruções.
Exemplo: help
exit

Descrição: Encerra o programa.
Exemplo: exit
test

Descrição: Executa a sequência de teste padrão (inicia, monitora e para o motor).
Exemplo: test
5.3. Exemplos de Uso
Iniciar o Motor em 1000 RPM:

yaml
Copiar código
Enter command ('help' for instructions): start 1000
[ACTION] Starting the motor forward at 1000.0 RPM...
[ACTION] Setting speed reference to 1000.0 RPM (Modbus value: 4825)
[INFO] Current Speed: 1000.00 RPM
Verificar o Status do Inversor:

bash
Copiar código
Enter command ('help' for instructions): status
[INFO] The inverter is NOT in fault.
Ler a Velocidade Atual do Motor:

bash
Copiar código
Enter command ('help' for instructions): read_speed
[INFO] Current Speed: 1000.00 RPM
6. Tratamento de Erros
Comandos Inválidos:

O sistema notificará sobre comandos não reconhecidos e sugerirá usar o comando help.
Exemplo:
vbnet
Copiar código
[ERROR] Unrecognized command. Type 'help' to see the list of commands.
Falha na Leitura de Parâmetros:

Se a leitura de parâmetros Modbus falhar, será exibida uma mensagem de erro correspondente.
Exemplo:
css
Copiar código
[ERROR] Failed to read current speed.
Falhas do Inversor:

Em caso de falhas detectadas, o sistema alertará o usuário.
Exemplo:
csharp
Copiar código
[ALERT] The inverter is in FAULT.
7. Considerações de Segurança
Configuração Adequada do VFD:

Assegure-se de que todos os parâmetros do VFD estejam corretamente configurados conforme as especificações do fabricante.
Limites de RPM:

Sempre defina os limites de RPM dentro das especificações do motor e do VFD para evitar danos.
Teste em Ambiente Controlado:

Realize testes iniciais em um ambiente seguro para garantir que o sistema responde adequadamente aos comandos.
8. Referências
Manual do VFD CFW500 da WEG
Documentação da Biblioteca umodbus para MicroPython
MicroPython Documentation: https://docs.micropython.org/en/latest/