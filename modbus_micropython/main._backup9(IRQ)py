import time
import sys
import uasyncio as asyncio
from machine import Pin
from umodbus.serial import Serial as ModbusRTUMaster
from encoder import Encoder
import select

# Configurações do UART e Modbus
UART_ID = 0  # UART0 para Modbus RTU
TX_PIN_NUM = 0  # GPIO0 (TX) para UART0
RX_PIN_NUM = 1  # GPIO1 (RX) para UART0

# Pino de controle DE/RE para transceptor RS485
DE_RE_PIN = Pin(2, Pin.OUT)
DE_RE_PIN.value(0)  # Inicialmente habilita receptor

# Configuração do Mestre Modbus RTU usando a biblioteca umodbus
modbus_master = ModbusRTUMaster(
    pins=(TX_PIN_NUM, RX_PIN_NUM),
    baudrate=19200,
    data_bits=8,
    stop_bits=1,
    parity=None,
    ctrl_pin=DE_RE_PIN,
    uart_id=UART_ID
)

SLAVE_ADDRESS = 1  # Endereço Modbus escravo do inversor

# Configuração do LED
LED_PIN = 25  # Pino do LED integrado no Raspberry Pi Pico
led = Pin(LED_PIN, Pin.OUT)

# Variáveis globais
max_RPM = 1700.0  # Configurado no P0208
VERBOSE_LEVEL = 1
encoder_position = 0  # Posição atual do encoder

# Dicionário de descrições de bits do P0680 (Estado Lógico)
P0680_BITS = {
    0: "STO - Safe Torque Off",
    1: "Command Rotate",
    2: "Reserved",
    3: "Reserved",
    4: "Rapid Stop Active",
    5: "Second Ramp Active",
    6: "Configuration Mode",
    7: "Alarm",
    8: "Ramp Enabled - RUN",
    9: "General Enabled",
    10: "Rotation Direction",
    11: "JOG Active",
    12: "LOC/REM Mode",
    13: "Undervoltage",
    14: "Automatic - PID",
    15: "Fault"
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
    if response and len(response) == 1:
        state = response[0]
        if state == 0:
            print_verbose("[SERIAL] Serial Interface Inactive", 1)
        elif state == 1:
            print_verbose("[SERIAL] Serial Interface Active", 1)
        elif state == 2:
            print_verbose("[SERIAL] Watchdog Error on Serial Interface", 1)
        else:
            print_verbose(f"[SERIAL] Unknown State ({state})", 1)
        return state
    else:
        print_verbose("[SERIAL] Failed to read parameter", 1)
        return None

def read_p0680():
    """Lê o Estado Lógico (P0680) e retorna o valor bruto."""
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=680,
        register_qty=1
    )
    if response and len(response) == 1:
        state = response[0]
        if VERBOSE_LEVEL >= 4:
            print_verbose(f"[STATUS] P0680 = 0x{state:04X}", 4)
            for bit in range(16):
                if state & (1 << bit):
                    description = P0680_BITS.get(bit, f"Bit {bit} Unknown")
                    print_verbose(f"    Bit {bit}: {description} ACTIVE", 4)
        elif VERBOSE_LEVEL >= 3:
            print_verbose(f"[STATUS] P0680 = 0x{state:04X}", 3)
        return state
    else:
        print_verbose("[STATUS] Failed to read parameter P0680", 1)
        return None

def check_fault_via_p0680(ref=0):
    """Verifica se o inversor está em falha através do bit 15 de P0680."""
    state = read_p0680()
    if state is not None:
        if state & (1 << 15):  # Bit 15
            print_verbose("[ALERT] The inverter is in FAULT.", 1)
            return True
        else:
            if ref:
                print_verbose("[INFO] The inverter is NOT in fault.", 1)
            else:
                print_verbose("[INFO] The inverter is NOT in fault.", 2)
            return False
    else:
        print_verbose("[ERROR] Could not determine fault state.", 1)
        return None

def start_motor(rpm_reference):
    """Inicia o motor com a referência de velocidade especificada em RPM."""
    print_verbose(f"[ACTION] Starting the motor forward at {rpm_reference} RPM...", 1)
    control_word = 0x0007  # Bits 0, 1, 2 setados para iniciar em sentido direto
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    set_speed_reference(rpm_reference)
    if VERBOSE_LEVEL >= 3:
        read_p0680()

def stop_motor():
    """Para o motor."""
    print_verbose("[ACTION] Stopping the motor...", 1)
    control_word = 0x0002  # Bit 1 habilitado, demais bits zerados
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    if VERBOSE_LEVEL >= 3:
        read_p0680()

def reverse_motor(rpm_reference):
    """Reverte o sentido do motor com a referência de velocidade especificada em RPM."""
    print_verbose(f"[ACTION] Reversing the motor direction at {rpm_reference} RPM...", 1)
    control_word = 0x0003  # Bits 0 e 1 setados para iniciar em sentido reverso
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=682,
        register_value=control_word
    )
    set_speed_reference(rpm_reference)
    if VERBOSE_LEVEL >= 3:
        read_p0680()

def set_speed_reference(rpm_value):
    """Define a referência de velocidade (P0208) a partir de RPM."""
    global max_RPM
    if max_RPM == 0:
        print_verbose("[ERROR] Maximum RPM is zero. Cannot set speed.", 1)
        return
    # Assegura que o valor de RPM esteja dentro dos limites
    if rpm_value < 0:
        rpm_value = 0
    elif rpm_value > max_RPM:
        rpm_value = max_RPM
    # Mapeia RPM para valor Modbus considerando que 1700 RPM = 8192 (2000h)
    speed_reference_value = int((rpm_value / max_RPM) * 8192)
    # Limita o valor de referência dentro dos limites aceitáveis de um inteiro de 16 bits
    if speed_reference_value > 32767:
        speed_reference_value = 32767
    elif speed_reference_value < -32768:
        speed_reference_value = -32768
    print_verbose(f"[ACTION] Setting speed reference to {rpm_value} RPM (Modbus value: {speed_reference_value})", 2)
    modbus_master.write_single_register(
        slave_addr=SLAVE_ADDRESS,
        register_address=683,
        register_value=speed_reference_value
    )
    if VERBOSE_LEVEL >= 3:
        read_p0680()

def read_current_speed():
    """Lê a Velocidade Atual (P0681) e retorna em RPM."""
    # Defina o número de pares de polos do motor
    num_pares_polos = 2  # Ajuste conforme o motor em uso

    # Ler a frequência nominal do motor (P0403)
    response_nominal_freq = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=403,  # P0403
        register_qty=1
    )
    if not response_nominal_freq or len(response_nominal_freq) != 1:
        print_verbose("[ERROR] Failed to read nominal frequency (P0403).", 1)
        return None
    nominal_freq = response_nominal_freq[0]

    # Ler o valor de P0681
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=681,  # P0681
        register_qty=1
    )
    if response and len(response) == 1:
        p0681_value = response[0]
        # Converter para inteiro com sinal de 16 bits
        if p0681_value >= 0x8000:
            p0681_value -= 0x10000

        # Calcular a velocidade em Hz
        motor_speed_hz = (p0681_value / 8192) * nominal_freq

        # Calcular RPM
        rpm = motor_speed_hz * (60 / num_pares_polos)

        print_verbose(f"[INFO] Current Speed: {rpm:.2f} RPM", 1)
        return rpm
    else:
        print_verbose("[ERROR] Failed to read current speed (P0681).", 1)
        return None

def reset_fault():
    """Envia comando para resetar falhas."""
    print_verbose("[ACTION] Attempting to reset fault...", 1)
    control_word = 0x0080  # Palavra de Controle para Reset de Falha (Bit 7)
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
    print_verbose("[INFO] Fault reset command sent.", 1)
    if VERBOSE_LEVEL >= 3:
        read_p0680()

def read_max_rpm_function():
    """Função para ler o máximo de RPM (P0208) via CLI."""
    global max_RPM
    response = modbus_master.read_holding_registers(
        slave_addr=SLAVE_ADDRESS,
        starting_addr=208,  # P0208
        register_qty=1
    )
    if response and len(response) == 1:
        max_rpm_raw = response[0]
        max_rpm = float(max_rpm_raw)
        print_verbose(f"[INFO] Maximum RPM Read: {max_rpm} RPM", 1)
        max_RPM = max_rpm
    else:
        print_verbose("[ERROR] Failed to read maximum RPM (P0208). Please check the connection or inverter settings.", 1)

def show_manual():
    """Exibe o manual de instruções."""
    manual = """
    ========== INSTRUCTION MANUAL ==========
    Available Commands:
    - start [rpm]: Start the motor with the specified RPM.
    - stop: Stop the motor.
    - reverse [rpm]: Reverse the motor direction with the specified RPM.
    - set_speed [rpm]: Change the motor speed reference to the specified RPM.
    - read_speed: Read and display the current motor speed.
    - status: Read and display the current inverter status.
    - reset_fault: Reset inverter faults.
    - set_verbose [0-4]: Set verbosity level (0 = minimal, 1 = standard, 2 = detailed, 3 = advanced, 4 = maximum).
    - read_max_rpm: Read the maximum RPM from the inverter.
    - help: Display this instruction manual.
    - exit: Exit the program.
    - test: Execute the default test sequence.
    ===========================================
    """
    print_verbose(manual, 1)

def process_command(command):
    """Processa um único comando."""
    global VERBOSE_LEVEL
    parts = command.strip().split()
    if not parts:
        return True  # Continua executando
    cmd = parts[0].lower()

    if cmd == "start":
        if len(parts) >= 2:
            try:
                rpm = float(parts[1])
            except ValueError:
                print_verbose("[ERROR] Invalid RPM value. Use a number.", 1)
                return True
        else:
            rpm = 1000  # Valor padrão
        start_motor(rpm)
    elif cmd == "stop":
        stop_motor()
    elif cmd == "reverse":
        if len(parts) >= 2:
            try:
                rpm = float(parts[1])
            except ValueError:
                print_verbose("[ERROR] Invalid RPM value. Use a number.", 1)
                return True
        else:
            rpm = 1000  # Valor padrão
        reverse_motor(rpm)
    elif cmd == "set_speed":
        if len(parts) >= 2:
            try:
                rpm = float(parts[1])
                set_speed_reference(rpm)
            except ValueError:
                print_verbose("[ERROR] Invalid RPM value. Use a number.", 1)
        else:
            print_verbose("[ERROR] Specify the speed in RPM.", 1)
    elif cmd == "read_speed":
        read_current_speed()
    elif cmd == "status":
        check_fault_via_p0680(1)
    elif cmd == "reset_fault":
        reset_fault()
    elif cmd == "set_verbose":
        if len(parts) >= 2:
            try:
                level = int(parts[1])
                if level in [0, 1, 2, 3, 4]:
                    VERBOSE_LEVEL = level
                    print_verbose(f"[INFO] Verbosity level set to {VERBOSE_LEVEL}.", 1)
                else:
                    print_verbose("[ERROR] Invalid verbosity level. Use a value between 0 and 4.", 1)
            except ValueError:
                print_verbose("[ERROR] Invalid verbosity level. Use a number between 0 and 4.", 1)
        else:
            print_verbose("[ERROR] Specify the verbosity level (0 to 4).", 1)
    elif cmd == "read_max_rpm":
        read_max_rpm_function()
    elif cmd == "help":
        show_manual()
    elif cmd == "exit":
        print_verbose("[INFO] Exiting the program.", 1)
        return False  # Sinaliza para sair
    elif cmd == "test":
        print_verbose("[INFO] Executing the default test sequence.", 1)
        start_motor(1000)
        time.sleep(5)
        read_current_speed()
        stop_motor()
    else:
        print_verbose("[ERROR] Unrecognized command. Type 'help' to see the list of commands.", 1)
    return True  # Continua executando

def led_blink_task():
    """Task para piscar o LED."""
    async def blink():
        while True:
            led.toggle()
            await asyncio.sleep(0.5)
    return blink()

# Callback function to handle encoder changes
def encoder_callback(value, delta):
    global encoder_position
    encoder_position = value
    print_verbose(f"Encoder Position: {encoder_position}, Delta: {delta}", 2)

async def read_user_input():
    loop = asyncio.get_event_loop()
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            user_input = sys.stdin.readline().strip()
            if user_input:
                should_continue = process_command(user_input)
                if not should_continue:
                    loop.stop()
                    break
        await asyncio.sleep(0.1)

async def status_request_task():
    STATUS_REQUEST_INTERVAL = 5  # Intervalo de requisição de status em segundos
    while True:
        check_fault_via_p0680()
        await asyncio.sleep(STATUS_REQUEST_INTERVAL)

async def main():
    global max_RPM, VERBOSE_LEVEL, encoder_position

    # Exibe o manual de instruções
    show_manual()

    # Lê a referência máxima de RPM do inversor
    read_max_rpm_function()

    # Lê o estado da interface serial
    read_p0316()

    # Verifica se há falhas
    check_fault_via_p0680()

    # Configurar os pinos A e B como entradas com pull-up interno
    pin_a = Pin(16, Pin.IN, Pin.PULL_UP)
    pin_b = Pin(17, Pin.IN, Pin.PULL_UP)

    # Initialize the Encoder
    encoder = Encoder(
        pin_x=pin_a,
        pin_y=pin_b,
        v=0,           # Initial value
        div=1,         # Division factor (adjust if needed)
        vmin=None,     # Optional minimum value
        vmax=None,     # Optional maximum value
        mod=None,      # Optional modulus
        callback=encoder_callback,  # Function to call on value change
        args=(),       # Arguments for callback
        delay=100      # Delay in ms between callbacks
    )

    # Start tasks
    asyncio.create_task(led_blink_task())
    asyncio.create_task(read_user_input())
    asyncio.create_task(status_request_task())

    # Keep the main loop running
    while True:
        await asyncio.sleep(1)

try:
    asyncio.run(main())
finally:
    asyncio.new_event_loop()
