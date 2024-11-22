import time
from machine import Pin
from umodbus.serial import Serial as ModbusRTUMaster

# Define os parâmetros UART
UART_ID = 0  # ID da UART (0 ou 1)
TX_PIN_NUM = 0  # Número do pino GPIO para TX (por exemplo, GP0)
RX_PIN_NUM = 1  # Número do pino GPIO para RX (por exemplo, GP1)

# Pino de controle DE/RE (por exemplo, GP2)
DE_RE_PIN = Pin(2, Pin.OUT)
DE_RE_PIN.value(0)  # Inicialmente habilita o receptor

# Configuração do Mestre Modbus RTU usando a biblioteca micropython-modbus
modbus_master = ModbusRTUMaster(
    pins=(TX_PIN_NUM, RX_PIN_NUM),  # Números dos pinos UART TX e RX
    baudrate=19200,                 # Taxa de comunicação
    data_bits=8,                    # Número de bits de dados
    stop_bits=1,                    # Número de bits de parada
    parity=None,                    # Bit de paridade (None, 'E' para par, 'O' para ímpar)
    ctrl_pin=DE_RE_PIN,             # Pino de controle DE/RE para o transceiver RS485
    uart_id=UART_ID                 # ID da UART (0 ou 1)
)

SLAVE_ADDRESS = 1  # Endereço Modbus escravo do inversor

# Intervalo do Keep-Alive (em segundos)
KEEP_ALIVE_INTERVAL = 2  # Deve ser menor que o timeout configurado (5 segundos)

# Variável global para frequência máxima
max_frequency = 60.0  # Será atualizado lendo P0403

# Nível de verbosidade (0 = mínimo, 1 = padrão, 2 = máximo)
VERBOSE_LEVEL = 1

# Dicionário de descrições de bits do P0680 (Estado Lógico)
P0680_BITS = {
    0: "STO - Safe Torque Off",
    1: "Comando Gira",
    2: "Reservado",
    3: "Reservado",
    4: "Parada Rápida Ativa",
    5: "Segunda Rampa Ativa",
    6: "Em Modo de Configuração",
    7: "Em Alarme",
    8: "Rampa Habilitada - RUN",
    9: "Habilitado Geral",
    10: "Sentido de Giro",
    11: "JOG Ativo",
    12: "Modo LOC/REM",
    13: "Subtensão",
    14: "Automático - PID",
    15: "Em Falha"
}

def print_verbose(message, level):
    """Imprime mensagens de acordo com o nível de verbosidade."""
    if VERBOSE_LEVEL >= level:
        print(message)

def read_p0316():
    """Lê o Estado da Interface Serial (P0316)."""
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=316,
        register_qty=1
    )
    if response is not None and len(response) == 1:
        state = response[0]
        if state == 0:
            print("[SERIAL] Interface Serial Inativa")
        elif state == 1:
            print("[SERIAL] Interface Serial Ativa")
        elif state == 2:
            print("[SERIAL] Erro de Watchdog na Interface Serial")
        else:
            print(f"[SERIAL] Estado desconhecido ({state})")
        return state
    else:
        print("[SERIAL] Falha ao ler o parâmetro")
        return None

def read_p0680():
    """Lê o Estado Lógico (P0680) e retorna o valor bruto."""
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=680,
        register_qty=1
    )
    if response is not None and len(response) == 1:
        state = response[0]
        print_verbose(f"[STATUS] P0680 = 0x{state:04X}", 2)
        # Analisa os bits
        for bit in range(16):
            if state & (1 << bit):
                description = P0680_BITS.get(bit, f"Bit {bit} desconhecido")
                print_verbose(f"    Bit {bit}: {description} ATIVO", 2)
        return state
    else:
        print("[STATUS] Falha ao ler o parâmetro P0680")
        return None

def check_fault_via_p0680():
    """Verifica se o inversor está em falha através do bit 15 de P0680."""
    state = read_p0680()
    if state is not None:
        if state & (1 << 15):  # Bit 15
            print("[ALERTA] O inversor está em FALHA.")
            return True
        else:
            print_verbose("[INFO] O inversor NÃO está em falha.", 1)
            return False
    else:
        print("[ERRO] Não foi possível determinar o estado de falha.")
        return None

def start_motor(rpm_reference):
    """Inicia o motor com a referência de velocidade especificada em RPM."""
    print("[AÇÃO] Iniciando o motor para frente...")
    # Palavra de Controle (P0682)
    control_word = 0x0007  # Bits 0, 1, 2 setados para iniciar em sentido direto
    # Escreve a Palavra de Controle
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    # Define a Referência de Velocidade (P0683)
    set_speed_reference(rpm_reference)

def stop_motor():
    """Para o motor."""
    print("[AÇÃO] Parando o motor...")
    # Palavra de Controle (P0682)
    control_word = 0x0002  # Bit 1 habilitado, demais bits zerados
    # Escreve a Palavra de Controle
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )

def reverse_motor(rpm_reference):
    """Reverte o sentido do motor com a referência de velocidade especificada em RPM."""
    print("[AÇÃO] Revertendo a direção do motor...")
    # Palavra de Controle (P0682)
    control_word = 0x0003  # Bits 0 e 1 setados para iniciar em sentido reverso
    # Escreve a Palavra de Controle
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    # Define a Referência de Velocidade (P0683)
    set_speed_reference(rpm_reference)

def set_speed_reference(rpm_value):
    """Define a referência de velocidade (P0683) a partir de RPM."""
    global max_frequency
    max_RPM = max_frequency * 60  # Exemplo: 60 Hz * 60 = 3600 RPM
    # Evita divisão por zero caso max_frequency seja zero
    if max_RPM == 0:
        print("[ERRO] Frequência máxima é zero. Não é possível definir a velocidade.")
        return
    speed_reference_value = int((rpm_value / max_RPM) * 16384)
    # Limita o valor de referência dentro dos limites aceitáveis
    if speed_reference_value > 16384:
        speed_reference_value = 16384
    elif speed_reference_value < -16384:
        speed_reference_value = -16384
    print(f"[AÇÃO] Definindo referência de velocidade para {rpm_value} RPM (valor Modbus: {speed_reference_value})")
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=683,
        register_value=speed_reference_value
    )

def read_current_speed():
    """Lê a Velocidade Atual (P0681) e converte para RPM."""
    global max_frequency
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=681,
        register_qty=1
    )
    if response is not None and len(response) == 1:
        speed_value = response[0]
        # Verifica se o valor é negativo (13 bits com sinal)
        if speed_value & 0x1000:  # Bit 12 indica sinal
            speed_value = speed_value - 0x2000  # Ajusta para valor negativo
        # Converte o valor para Hz
        frequency = (speed_value / 16384) * max_frequency
        # Converte Hz para RPM
        rpm = frequency * 60
        print(f"[INFO] Velocidade Atual: {rpm:.2f} RPM")
        return rpm
    else:
        print("[ERRO] Falha ao ler a velocidade atual.")
        return None

def reset_fault():
    """Envia comando para resetar falhas."""
    print("[AÇÃO] Tentando resetar a falha...")
    # Palavra de Controle (P0682) - Reset de Falhas (Bit 7)
    control_word = 0x0080  # Bit 7 setado em 1
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    time.sleep(0.1)
    # Reseta o bit 7
    control_word = 0x0000
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    print("[INFO] Comando de reset de falha enviado.")

def keep_alive():
    """Função keep-alive para evitar timeout do watchdog."""
    # Pode ser uma leitura simples que não interfira nas operações
    state = read_p0680()
    if state is not None:
        print_verbose("[KEEP-ALIVE] Sinal enviado ao inversor.", 2)

def read_max_frequency():
    """Lê a frequência máxima (P0403) do inversor."""
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=403,
        register_qty=1
    )
    if response is not None and len(response) == 1:
        max_freq = response[0] / 1  # P0403 armazenado em 0.1 Hz
        print(f"[INFO] Frequência máxima lida: {max_freq} Hz")
        return max_freq
    else:
        print("[ERRO] Falha ao ler a frequência máxima (P0403). Usando valor padrão de 60 Hz.")
        return 60.0

def show_manual():
    """Exibe o manual de instruções."""
    manual = """
    ========== MANUAL DE INSTRUÇÕES ==========
    Comandos disponíveis:
    - start [rpm]: Inicia o motor com a velocidade especificada em RPM.
    - stop: Para o motor.
    - reverse [rpm]: Inverte o sentido do motor com a velocidade especificada em RPM.
    - set_speed [rpm]: Altera a referência de velocidade do motor para o RPM especificado.
    - read_speed: Lê e exibe a velocidade atual do motor.
    - status: Lê e exibe o estado atual do inversor.
    - reset_fault: Reseta falhas do inversor.
    - set_verbose [0/1/2]: Define o nível de verbosidade (0 = mínimo, 1 = padrão, 2 = máximo).
    - help: Exibe este manual de instruções.
    - exit: Sai do programa.
    ===========================================
    """
    print(manual)

def main():
    global max_frequency, VERBOSE_LEVEL
    try:
        # Exibe o manual de instruções
        show_manual()
        # Lê a frequência máxima do inversor
        max_frequency = read_max_frequency()
        # Lê o estado da interface serial
        read_p0316()
        # Verifica se há falhas
        if check_fault_via_p0680():
            reset_fault()

        last_keep_alive = time.time()

        while True:
            # Verifica o keep-alive
            current_time = time.time()
            if current_time - last_keep_alive >= KEEP_ALIVE_INTERVAL:
                keep_alive()
                last_keep_alive = current_time

            # Lê o comando do usuário
            user_input = input("Digite um comando ('help' para instruções): ")
            if not user_input:
                continue
            command_parts = user_input.strip().split()
            command = command_parts[0].lower()

            if command == "start":
                if len(command_parts) >= 2:
                    rpm = float(command_parts[1])
                else:
                    rpm = 1000  # Valor padrão
                start_motor(rpm)
            elif command == "stop":
                stop_motor()
            elif command == "reverse":
                if len(command_parts) >= 2:
                    rpm = float(command_parts[1])
                else:
                    rpm = 1000  # Valor padrão
                reverse_motor(rpm)
            elif command == "set_speed":
                if len(command_parts) >= 2:
                    rpm = float(command_parts[1])
                    set_speed_reference(rpm)
                else:
                    print("[ERRO] Especifique a velocidade em RPM.")
            elif command == "read_speed":
                read_current_speed()
            elif command == "status":
                check_fault_via_p0680()
            elif command == "reset_fault":
                reset_fault()
            elif command == "set_verbose":
                if len(command_parts) >= 2:
                    level = int(command_parts[1])
                    if level in [0, 1, 2]:
                        VERBOSE_LEVEL = level
                        print(f"[INFO] Nível de verbosidade definido para {VERBOSE_LEVEL}.")
                    else:
                        print("[ERRO] Nível de verbosidade inválido. Use 0, 1 ou 2.")
                else:
                    print("[ERRO] Especifique o nível de verbosidade (0, 1 ou 2).")
            elif command == "help":
                show_manual()
            elif command == "exit":
                print("[INFO] Encerrando o programa.")
                break
            elif command == "teste":
                # Reutiliza o conteúdo atual da função main como o comando "teste"
                print("[INFO] Executando o teste padrão.")
                # Implementação do teste (pode ser personalizada)
                start_motor(1000)
                time.sleep(5)
                read_current_speed()
                stop_motor()
            else:
                print("[ERRO] Comando não reconhecido. Digite 'help' para ver a lista de comandos.")

    except Exception as e:
        print(f"[ERRO] Ocorreu um erro: {e}")

if __name__ == '__main__':
    main()
