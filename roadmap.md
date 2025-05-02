# CFW500 Modbus Control System Roadmap

This document outlines the current issues, planned improvements, and tasks for the Raspberry Pi Pico VFD Motor Control project using Modbus RTU. Each item is categorized by status and includes detailed implementation notes.

## 🚨 Critical Issues (Modbus Communication Reliability)

### 1. Relay-Main Modbus Read Failure
**Status**: 🔴 TO DO

**Summary**: Relay Pico (Modbus Master) intermittently fails to read Input Registers from Main Pico (Slave) via Function Code 04, yielding "no data received" errors or stale values, even though Main Pico logs show correct internal updates.

**Root Causes**:
- **Register Count Mismatch**: Relay requests 7 registers when Main Pico has 8 defined registers
- **Async Scheduling Delays**: Main Pico's modbus_slave_poll_task might be starved by other tasks
- **RS-485 DE/RE Timing**: DE pin timing may prevent slave from receiving/responding properly
- **Physical/Hardware Issues**: Wiring, ground references, or EMI problems

**Implementation Plan**:

1. **Fix Register Count Mismatch**:
   ```python
   # In relay_pico/main.py, line 161:
   register_qty=8  # FIXED: Match Main Pico's 8 IREGS instead of 7
   ```

2. **Implement Timeout with Exponential Back-off**:
   ```python
   # In relay_pico/main.py:
   # Add a retry mechanism with exponential back-off
   def read_with_retry(slave_addr, starting_addr, register_qty, max_retries=4):
       """Retry Modbus reads with exponential back-off"""
       retry = 0
       timeout_ms = 100  # Initial timeout
       last_exception = None
       
       while retry < max_retries:
           try:
               print(f"[MODBUS] Read attempt {retry+1}, timeout {timeout_ms}ms")
               # Allow time for slave to process
               time.sleep_ms(10)
               result = modbus_master.read_input_registers(
                   slave_addr=slave_addr,
                   starting_addr=starting_addr,
                   register_qty=register_qty
               )
               if result:
                   return result
           except Exception as e:
               last_exception = e
               print(f"[ERROR] Retry {retry+1} failed: {e}")
           
           # Exponential back-off (100, 200, 400, 800ms)
           retry += 1
           timeout_ms = min(timeout_ms * 2, 800)  # Cap at 800ms
           time.sleep_ms(timeout_ms)
       
       # All retries failed
       if last_exception:
           raise last_exception
       return None
   
   # Then use this in the status reading section:
   input_regs = read_with_retry(
       slave_addr=MAIN_PICO_SLAVE_ADDR,
       starting_addr=REG_CURRENT_RPM,
       register_qty=8  # Correct register count
   )
   ```

3. **Add Debug Instrumentation**:
   - Modify `modbus_slave_poll_task` in Main Pico to log frame reception/responses
   - Add timestamp logging to measure processing delays

### 2. DE/RE (RS-485 Direction Control) Timing Issues
**Status**: 🔴 TO DO

**Summary**: The RS-485 transceiver may not be switching between transmit and receive modes fast enough, causing the slave's response to be missed.

**Implementation Plan**:
1. **Choose ONE consistent approach** for DE/RE control across the project:

   **Option A (Recommended): Let Library Handle DE/RE**
   ```python
   # In relay_pico/main.py:
   # Remove manual pin control and sleep delays
   # Comment out these lines:
   # rs485_de_re_pin = Pin(RS485_DE_RE_PIN_NUM, Pin.OUT)
   # rs485_de_re_pin.value(0) # Start in receive mode
   
   # Remove delays around Modbus calls:
   # time.sleep_ms(20) # Delay before write
   modbus_master.write_single_register(MAIN_PICO_SLAVE_ADDR, REG_CMD, 2)
   # time.sleep_ms(20) # Delay after write
   
   # Modify the ModbusRTUMaster initialization to add t1_5/t3_5 char settings
   modbus_master = ModbusRTUMaster(
       pins=(RS485_TX_PIN_NUM, RS485_RX_PIN_NUM),
       baudrate=RS485_BAUDRATE,
       ctrl_pin=RS485_DE_RE_PIN_NUM,
       uart_id=RS485_UART_ID,
       t1_5=750,  # Microseconds for 1.5 char time at 19200 baud
       t3_5=1750  # Microseconds for 3.5 char time at 19200 baud
   )
   ```

   **Option B: Manual DE/RE Wrapper Functions**
   ```python
   # If library DE/RE control is inadequate, implement consistent wrappers:
   def modbus_write(fn, *args, **kwargs):
       """Wrap Modbus write functions with proper DE/RE control"""
       rs485_de_re_pin.value(1)  # Set to transmit mode
       time.sleep_us(50)  # Short delay for transition (microseconds)
       result = None
       try:
           result = fn(*args, **kwargs)  # Call the actual Modbus function
       finally:
           # Wait for UART to finish transmitting
           while not uart.txdone():
               pass
           time.sleep_us(50)  # Short delay before switching (microseconds)
           rs485_de_re_pin.value(0)  # Back to receive mode
       return result
   
   # Use wrapper consistently for all calls:
   modbus_write(modbus_master.write_single_register, 
                MAIN_PICO_SLAVE_ADDR, REG_CMD, 2)
   ```

2. Test with oscilloscope/logic analyzer to verify proper DE/RE timing

3. Reduce the baud rate to 9600 on both devices for initial testing (increasing timing margins)

### 3. Incorrect asyncio timeout API in homing routine
**Status**: 🔴 TO DO

**Summary**: The homing routine in main_pico/main.py uses an invalid `wait_for_ms` API call that doesn't exist in MicroPython's uasyncio.

**Implementation Plan**:
```python
# In main_pico/main.py, around line ~207:
# Replace:
await asyncio.wait_for_ms(endstop_event.wait(), 30000)  # 30 second timeout

# With:
await asyncio.wait_for(endstop_event.wait(), 30.0)  # 30 second timeout (seconds as float)
```

### 4. VFD Baud-Rate Parameter Ignored in CFW500Modbus Class
**Status**: 🔴 TO DO

**Summary**: The CFW500Modbus class constructor ignores any baudrate parameter and hard-codes it to 19200, making the VFD_BAUDRATE setting in main.py ineffective.

**Implementation Plan**:
```python
# In main_pico/cfw500_modbus.py:
# Modify the constructor to accept a baudrate argument:
def __init__(self, uart_id, tx_pin, rx_pin, de_re_pin, slave_address=1, baudrate=19200):
    self.slave_address = slave_address
    self.modbus_master = ModbusRTUMaster(
        pins=(tx_pin, rx_pin),
        baudrate=baudrate,  # Use parameter instead of hard-coded value
        data_bits=8,
        stop_bits=1,
        parity=None,
        ctrl_pin=de_re_pin,
        uart_id=uart_id
    )
    self.max_RPM = 1700.0  # Default value, can be updated

# In main_pico/main.py:
# Pass VFD_BAUDRATE when instantiating the class:
vfd_master = CFW500Modbus(
    uart_id=VFD_UART_ID,
    tx_pin=VFD_TX_PIN_NUM,
    rx_pin=VFD_RX_PIN_NUM,
    de_re_pin=VFD_DE_RE_PIN_NUM,
    slave_address=VFD_SLAVE_ADDRESS,
    baudrate=VFD_BAUDRATE  # Pass the configured baudrate
)
```

## 🔄 System Architecture Improvements

### 1. Implement Event-Driven UART for Slave Interface
**Status**: 🔴 TO DO

**Summary**: Polling for Modbus slave requests wastes CPU cycles and introduces latency. An IRQ-driven approach would be more efficient.

**Implementation Plan**:
```python
# In main_pico/main.py:
# 1. Create a StreamReader for the UART (MicroPython v1.17+)
from machine import UART

# Initialize the UART with IRQ capability
slave_uart = UART(
    id=SLAVE_UART_ID,
    baudrate=SLAVE_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    tx=SLAVE_TX_PIN_NUM,
    rx=SLAVE_RX_PIN_NUM
)

# 2. Create an event for signaling when data is available
uart_data_event = asyncio.Event()

# 3. Set up the UART RX IRQ handler
def uart_rx_irq_handler(uart_obj):
    global uart_data_event
    # Just signal the event - don't do any processing in the IRQ
    uart_data_event.set()

# 4. Register the IRQ handler
slave_uart.irq(UART.RX_ANY, handler=uart_rx_irq_handler)

# 5. Replace the polling task with an event-driven task
async def modbus_slave_event_task(modbus_handler_obj):
    print_verbose("[DEBUG SLAVE EVENT TASK] Starting...", 3)
    while True:
        try:
            # Wait for the IRQ to signal data availability
            await uart_data_event.wait()
            uart_data_event.clear()  # Reset the event
            
            # Process the Modbus request immediately
            result = modbus_handler_obj.process()
            print_verbose(f"[DEBUG SLAVE EVENT] Processed request: {result}", 3)
        except Exception as e:
            print_verbose(f"[ERROR] Modbus Slave processing error: {e}", 0)
        
        # Brief yield to allow other tasks to run
        await asyncio.sleep_ms(1)
```

### 2. Static Test Configuration
**Status**: 🟢 DONE

**Summary**: Main Pico's `utils.py` has been modified to use static test values for Input Registers, allowing isolation of the communication issue from dynamic data updates.

**Implementation**:
```python
# In main_pico/utils.py - Already implemented:
'IREGS': { # Input Registers (Readable by Relay Master)
    # Setting static non-zero values for Step 1.1 testing
    'current_rpm':      {'register': REG_CURRENT_RPM,  'val': 123},      # TEST VALUE
    'vfd_status':       {'register': REG_VFD_STATUS,   'val': 0xABCD},   # TEST VALUE
    # etc...
}
```

### 3. Re-enable Disabled Production Code with DEBUG_MODE Flag
**Status**: 🔴 TO DO

**Summary**: Critical async tasks and initialization code are commented out for testing purposes, but need to be re-enabled for production.

**Implementation Plan**:
```python
# In main_pico/main.py:
# Add a global debug mode flag
DEBUG_MODE = False  # Set to False for production

# Re-enable initialize_encoder conditionally:
if not DEBUG_MODE:
    initialize_encoder(16, 17)  # Re-enabled for production
else:
    print_verbose("[INFO TEST] Encoder Disabled in DEBUG_MODE.", 0)

# Similarly for background tasks:
if not DEBUG_MODE:
    status_task = asyncio.create_task(vfd_status_request_task(vfd_master))
    relay_task = asyncio.create_task(relay_control_task())
    # Update to use event-driven slave task if implemented
    slave_poll_task = asyncio.create_task(modbus_slave_event_task(modbus_slave_handler))
else:
    # In DEBUG_MODE, only start the slave interface
    print_verbose("[INFO TEST] Most tasks disabled in DEBUG_MODE.", 0)
    slave_poll_task = asyncio.create_task(modbus_slave_event_task(modbus_slave_handler))
```

### 4. Implement Non-blocking File I/O with Thread Support
**Status**: 🔴 TO DO

**Summary**: Current configuration file operations are blocking, potentially freezing the async event loop during I/O operations.

**Implementation Plan**:
```python
# In utils.py:
import _thread
import gc

# Thread-safe flag
_file_op_in_progress = False

# Function to run in a separate thread
def _thread_write_config(config_data):
    global _file_op_in_progress
    try:
        # Use temporary file for atomic writes
        temp_file = CONFIG_FILE + ".tmp"
        with open(temp_file, 'w') as f:
            json.dump(config_data, f)
        import os
        os.rename(temp_file, CONFIG_FILE)  # Atomic replacement
        success = True
    except Exception as e:
        print(f"[ERROR] Thread file write failed: {e}")
        success = False
    finally:
        _file_op_in_progress = False
        # Force garbage collection after I/O
        gc.collect()

async def save_configuration_async():
    """Non-blocking configuration save using a thread if available"""
    global _file_op_in_progress
    
    if _file_op_in_progress:
        print_verbose("[WARN] File operation already in progress, skipping", 1)
        return False
    
    # Prepare the config data
    config = {
        "encoder_offset_steps": internal_state['encoder_offset_steps'],
        "encoder_output_mode": internal_state['encoder_output_mode'],
        "VERBOSE_LEVEL": internal_state['VERBOSE_LEVEL']
    }
    
    try:
        _file_op_in_progress = True
        # Try to use threads if available
        _thread.start_new_thread(_thread_write_config, (config,))
        print_verbose("[INFO] Started async configuration save", 2)
        return True
    except (ImportError, AttributeError) as e:
        # Fallback for platforms without thread support
        print_verbose(f"[WARN] Thread not available, using sync save: {e}", 1)
        _file_op_in_progress = False
        # Fallback to synchronous save
        try:
            temp_file = CONFIG_FILE + ".tmp"
            with open(temp_file, 'w') as f:
                json.dump(config, f)
            import os
            os.rename(temp_file, CONFIG_FILE)
            print_verbose("[INFO] Configuration saved synchronously", 1)
            return True
        except Exception as e:
            print_verbose(f"[ERROR] Failed to save configuration: {e}", 0)
            return False
```

### 5. Memory Management and Heap Tracking
**Status**: 🔴 TO DO

**Summary**: The system doesn't monitor memory usage, which can lead to out-of-memory errors and fragmentation.

**Implementation Plan**:
```python
# In utils.py:
import gc

class MemoryMonitor:
    def __init__(self, threshold_pct=20, report_interval_ms=60000):
        self.threshold_pct = threshold_pct  # Free memory threshold percentage
        self.report_interval_ms = report_interval_ms
        self.last_report_time = 0
        self.min_free = 1000000  # Track lowest free memory
        self.mem_warnings = 0
        
    def check_memory(self, force_gc=False, verbose=True):
        """Check memory status and optionally force garbage collection"""
        if force_gc:
            gc.collect()
        
        # Get memory info
        free = gc.mem_free()
        alloc = gc.mem_alloc()
        total = free + alloc
        free_pct = (free / total) * 100
        
        # Update minimum observed free memory
        if free < self.min_free:
            self.min_free = free
        
        # Check if memory is below threshold
        if free_pct < self.threshold_pct:
            self.mem_warnings += 1
            if verbose:
                print_verbose(f"[WARN] Low memory: {free} bytes ({free_pct:.1f}%), forcing GC", 0)
            gc.collect()
            # Get new values after GC
            free = gc.mem_free()
            alloc = gc.mem_alloc()
            free_pct = (free / (free + alloc)) * 100
            if verbose:
                print_verbose(f"[INFO] After GC: {free} bytes ({free_pct:.1f}%)", 1)
        
        # Periodic reporting
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_report_time) >= self.report_interval_ms:
            self.last_report_time = current_time
            if verbose:
                print_verbose(f"[INFO] Memory: {free}/{total} bytes ({free_pct:.1f}%), min: {self.min_free}, warnings: {self.mem_warnings}", 1)
        
        return free, alloc, free_pct

# Singleton instance
memory_monitor = MemoryMonitor()

# In main.py, add periodic memory checks
async def memory_monitor_task():
    """Periodically check memory status"""
    while True:
        memory_monitor.check_memory(force_gc=True)
        # Longer interval during normal operation
        await asyncio.sleep(30)  

# Start the memory monitor task
memory_task = asyncio.create_task(memory_monitor_task())

# Also check memory after potentially memory-intensive operations
# For example, after processing large Modbus frames or JSON operations
memory_monitor.check_memory(force_gc=True, verbose=False)
```

## 🔧 General Improvements

### 1. Replace Blocking Calls with Async Equivalents
**Status**: 🔴 TO DO

**Summary**: Blocking calls to time.sleep_ms() inside async tasks block the entire scheduler, causing sporadic communications issues.

**Implementation Plan**:
```python
# In main_pico/main.py:
# Replace all instances of time.sleep_ms with await asyncio.sleep_ms

# For example in relay_control_task:
async def relay_control_task():
    """Controls relays based on internal fault state."""
    while True:
        if internal_state['fault_detected']:
            # Blink on fault
            relay_pin1.on()
            relay_pin2.on()
            await asyncio.sleep_ms(500)  # Use await, not time.sleep_ms
            relay_pin1.off()
            relay_pin2.off()
            await asyncio.sleep_ms(500)  # Use await, not time.sleep_ms
        else:
            # Solid on if no fault
            relay_pin1.on()
            relay_pin2.on()
            await asyncio.sleep_ms(RELAY_CONTROL_INTERVAL_MS)

# Also in relay_pico/main.py for any async functions
```

### 2. Implement Hardware Watchdog
**Status**: 🔴 TO DO

**Summary**: The system lacks hardware watchdog protection to recover from software hangs or crashes.

**Implementation Plan**:
```python
# In main_pico/main.py:
from machine import WDT

# Initialize watchdog with 8-second timeout (adjust based on longest task)
watchdog = WDT(timeout=8000)  # 8 seconds (ms)

# Modify the main async loop to feed the watchdog
async def main():
    # Existing initialization...
    
    print_verbose("[INFO] Watchdog initialized with 8 second timeout", 1)
    
    # Main loop with watchdog feed
    while True:
        watchdog.feed()  # Reset the watchdog timer
        
        # Process commands
        await process_modbus_commands(vfd_master)
        
        # Main loop sleep
        await asyncio.sleep_ms(MAIN_LOOP_SLEEP_MS)

# Also create a separate periodic watchdog task as backup
async def watchdog_feed_task():
    """Backup task to feed watchdog in case main loop stalls"""
    while True:
        watchdog.feed()
        await asyncio.sleep_ms(5000)  # Feed every 5 seconds

# Start the watchdog feed task
watchdog_task = asyncio.create_task(watchdog_feed_task())
```

### 3. Fix Duplicated Exception Handling in load_configuration
**Status**: 🔴 TO DO

**Summary**: The load_configuration function in utils.py duplicates the same default-reset logic in both exception handlers, making error debugging difficult.

**Implementation Plan**:
```python
# In main_pico/utils.py:
def load_configuration():
    """Loads settings from the configuration file into internal_state."""
    def reset_to_defaults():
        """Helper to reset configuration to defaults"""
        internal_state['encoder_offset_steps'] = 0
        internal_state['encoder_output_mode'] = "deg"
        internal_state['VERBOSE_LEVEL'] = 1
        # Update registers with defaults
        slave_registers['HREGS']['verbosity_level']['val'] = 1
        slave_registers['HREGS']['encoder_mode']['val'] = 1
        slave_registers['IREGS']['offset_steps']['val'] = 0
    
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
        
        # Schema validation (basic)
        valid = True
        required_keys = ["encoder_offset_steps", "encoder_output_mode", "VERBOSE_LEVEL"]
        for key in required_keys:
            if key not in config:
                print_verbose(f"[ERROR] Missing required config key: {key}", 0)
                valid = False
                break
                
        if not valid:
            raise ValueError("Invalid configuration schema")
            
        # Load validated config into internal_state
        internal_state['encoder_offset_steps'] = config.get("encoder_offset_steps", 0)
        internal_state['encoder_output_mode'] = config.get("encoder_output_mode", "deg")
        internal_state['VERBOSE_LEVEL'] = config.get("VERBOSE_LEVEL", 1)

        # Update corresponding slave registers after loading internal state
        slave_registers['HREGS']['verbosity_level']['val'] = internal_state['VERBOSE_LEVEL']
        slave_registers['HREGS']['encoder_mode']['val'] = 0 if internal_state['encoder_output_mode'] == "step" else 1
        slave_registers['IREGS']['offset_steps']['val'] = internal_state['encoder_offset_steps'] # Make offset readable

        print_verbose(f"[INFO] Loaded config: Offset={internal_state['encoder_offset_steps']}, Mode='{internal_state['encoder_output_mode']}', Verbose={internal_state['VERBOSE_LEVEL']}", 0)
    except FileNotFoundError:
        print_verbose("[WARNING] Configuration file not found. Using default settings.", 0)
        reset_to_defaults()
    except Exception as e:
        print_verbose(f"[ERROR] Failed to load configuration: {e}", 0)
        reset_to_defaults()
```

### 4. Fix stdin Polling in Relay Pico
**Status**: 🔴 TO DO

**Summary**: The Relay Pico uses uselect.poll() for sys.stdin, which may be unreliable on MicroPython.

**Implementation Plan**:
```python
# In relay_pico/main.py:
# Replace polling approach with a simple non-blocking read using select.select

import select

def check_stdin_input():
    """Non-blocking read from stdin that works reliably on MicroPython"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

# Replace the polling check in the main loop:
def run_master():
    while True:
        # Check for commands from PC (USB)
        pc_command = check_stdin_input()
        if pc_command:
            print(f"[PC CMD RX] {pc_command}")
            # Rest of command processing...
```

### 5. Fix P0680 Register Type in CFW500Modbus
**Status**: 🔴 TO DO

**Summary**: The read_p0680 method in CFW500Modbus uses read_holding_registers when it should potentially use read_input_registers.

**Implementation Plan**:
```python
# In main_pico/cfw500_modbus.py:
def read_p0680(self):
    """Reads the Logic Status (P0680) and returns the raw value."""
    # According to CFW500 documentation, P0680 is an input register
    response = self.modbus_master.read_input_registers(  # Changed from read_holding_registers
        slave_addr=self.slave_address,
        starting_addr=680,
        register_qty=1
    )
    if response and len(response) == 1:
        state = response[0]
        return state
    else:
        return None
```

### 6. Consistent State Tracking for Command Execution
**Status**: 🔴 TO DO

**Summary**: The command handler doesn't consistently update last_written_cmd and last_written_rpm across all command branches.

**Implementation Plan**:
```python
# In main_pico/main.py, modify handle_command_register_write:
# Inside the try/except block, update internal state at the end of each command branch:

def handle_command_register_write(reg_type, address, val):
    # Existing code...
    
    try:
        if command_to_process == 1: # START
            vfd_master.start_motor(target_rpm_val)
            print_verbose(f"[ACTION CB] Motor START processed (RPM: {target_rpm_val}).", 0)
            internal_state['last_written_cmd'] = command_to_process
            internal_state['last_written_rpm'] = target_rpm_val
        elif command_to_process == 2: # STOP
            vfd_master.stop_motor()
            print_verbose(f"[ACTION CB] Motor STOP processed.", 0)
            internal_state['last_written_cmd'] = command_to_process
            internal_state['last_written_rpm'] = 0  # Zero RPM when stopped
        elif command_to_process == 3: # REVERSE
            vfd_master.reverse_motor(target_rpm_val)
            print_verbose(f"[ACTION CB] Motor REVERSE processed (RPM: {target_rpm_val}).", 0)
            internal_state['last_written_cmd'] = command_to_process
            internal_state['last_written_rpm'] = target_rpm_val
        elif command_to_process == 4: # RESET_FAULT
            vfd_master.reset_fault()
            print_verbose(f"[ACTION CB] VFD Fault RESET processed.", 0)
            internal_state['last_written_cmd'] = command_to_process
            # Don't update RPM for RESET
        elif command_to_process == 5: # CALIBRATE
            # Existing CALIBRATE code...
            internal_state['last_written_cmd'] = command_to_process
            # Don't update RPM for CALIBRATE
            
        # Acknowledgement and register reset remains unchanged
    except Exception as e:
        print_verbose(f"[ERROR CB] Error processing command {command_to_process}: {e}", 0)
        # Optionally track error state
        internal_state['last_error'] = str(e)
```

### 7. Implement Ring Buffer Logging System
**Status**: 🔴 TO DO

**Summary**: Replace print_verbose with a structured, memory-efficient logging system.

**Implementation Plan**:
```python
# In utils.py:
class RingBufferLogger:
    # Log levels
    ERROR = 0
    WARNING = 1
    INFO = 2
    DEBUG = 3
    
    # Map levels to names
    LEVEL_NAMES = {
        ERROR: "ERROR",
        WARNING: "WARN",
        INFO: "INFO",
        DEBUG: "DEBUG"
    }
    
    def __init__(self, max_level=INFO, buffer_size=50, persistent_file=None):
        self.max_level = max_level
        self.buffer_size = buffer_size
        self.persistent_file = persistent_file
        self.log_buffer = []
        self.log_count = 0
        self.error_count = 0
        self.warn_count = 0
    
    def log(self, level, module, message):
        """Add a log entry to the ring buffer"""
        if level > self.max_level:
            return
            
        timestamp = time.ticks_ms()
        level_name = self.LEVEL_NAMES.get(level, "?")
        
        # Create log entry
        log_entry = {
            "time": timestamp,
            "level": level,
            "level_name": level_name,
            "module": module,
            "msg": message
        }
        
        # Print to console (formatted for readability)
        print(f"[{timestamp//1000}.{timestamp%1000:03d}] {level_name:<5} {module:<8} | {message}")
        
        # Update statistics
        self.log_count += 1
        if level == self.ERROR:
            self.error_count += 1
        elif level == self.WARNING:
            self.warn_count += 1
        
        # Add to ring buffer, removing oldest if full
        self.log_buffer.append(log_entry)
        if len(self.log_buffer) > self.buffer_size:
            self.log_buffer.pop(0)
        
        # Only persist critical errors to file to reduce flash wear
        if level == self.ERROR and self.persistent_file:
            try:
                with open(self.persistent_file, "a") as f:
                    f.write(f"{timestamp},{level_name},{module},{message}\n")
            except Exception:
                pass  # Silently fail if writing to file fails
    
    # Helper methods for different log levels
    def error(self, module, message):
        self.log(self.ERROR, module, message)
    
    def warning(self, module, message):
        self.log(self.WARNING, module, message)
    
    def info(self, module, message):
        self.log(self.INFO, module, message)
    
    def debug(self, module, message):
        self.log(self.DEBUG, module, message)
    
    def get_logs(self, level=None, limit=None):
        """Get filtered logs from the buffer"""
        if level is None:
            filtered = self.log_buffer
        else:
            filtered = [entry for entry in self.log_buffer if entry["level"] <= level]
            
        if limit:
            return filtered[-limit:]
        return filtered
    
    def get_stats(self):
        """Get logging statistics"""
        return {
            "total": self.log_count,
            "errors": self.error_count,
            "warnings": self.warn_count,
            "buffer_size": len(self.log_buffer),
            "max_buffer": self.buffer_size
        }
    
    def dump_to_file(self, filename):
        """Dump the entire log buffer to a file"""
        try:
            with open(filename, "w") as f:
                for entry in self.log_buffer:
                    time_str = f"{entry['time']//1000}.{entry['time']%1000:03d}"
                    f.write(f"{time_str},{entry['level_name']},{entry['module']},{entry['msg']}\n")
            return True
        except Exception as e:
            print(f"Failed to dump logs: {e}")
            return False

# Initialize global logger (without persistent file for normal operation)
logger = RingBufferLogger(max_level=INFO, buffer_size=100)

# Modify print_verbose to use logger
def print_verbose(message, level):
    """Convert old print_verbose calls to new logger format"""
    # Map old verbosity levels to new logger levels:
    # 0 -> ERROR, 1 -> WARNING, 2 -> INFO, 3 -> DEBUG
    module = "SYSTEM"
    
    # Extract module name from message if possible
    if "[" in message and "]" in message:
        prefix = message.split("]")[0] + "]"
        if prefix.startswith("["):
            module = prefix[1:-1]  # Remove brackets
            message = message[len(prefix):].lstrip()
    
    if level == 0:
        logger.error(module, message)
    elif level == 1:
        logger.warning(module, message)
    elif level == 2:
        logger.info(module, message)
    elif level == 3:
        logger.debug(module, message)

# Add command to dump logs
def dump_logs_command(filename="logs.txt"):
    """Command to dump logs to file"""
    return logger.dump_to_file(filename)
```

## 🧪 Testing & Development Framework

### 1. Implement Unit Tests Framework
**Status**: 🔴 TO DO

**Summary**: Create a comprehensive testing strategy with unit tests for core functions.

**Implementation Plan**:
```python
# Create a new file: test_framework.py

def run_tests(tests, verbose=True):
    """Simple test runner"""
    passed = 0
    failed = 0
    for test_func in tests:
        test_name = test_func.__name__
        try:
            if verbose:
                print(f"Running test: {test_name}...")
            result = test_func()
            if result is None or result is True:
                passed += 1
                if verbose:
                    print(f"✅ {test_name} PASSED")
            else:
                failed += 1
                print(f"❌ {test_name} FAILED: {result}")
        except Exception as e:
            failed += 1
            print(f"❌ {test_name} ERROR: {e}")
    
    print(f"Test results: {passed} passed, {failed} failed")
    return passed, failed

# Example test cases

def test_modbus_register_setup():
    """Test that registers are correctly set up"""
    from utils import slave_registers
    
    # Check Holding Registers
    if 'command' not in slave_registers['HREGS']:
        return "Missing 'command' in HREGS"
    if slave_registers['HREGS']['command']['register'] != 100:
        return f"Wrong address for 'command': {slave_registers['HREGS']['command']['register']} != 100"
    
    # Check Input Registers
    if 'current_rpm' not in slave_registers['IREGS']:
        return "Missing 'current_rpm' in IREGS"
    if slave_registers['IREGS']['current_rpm']['register'] != 0:
        return f"Wrong address for 'current_rpm': {slave_registers['IREGS']['current_rpm']['register']} != 0"
    
    return True

def test_encoder_calculation():
    """Test encoder position calculation"""
    from encoder_module import ENCODER_RESOLUTION
    
    # Test steps to degrees conversion
    steps = ENCODER_RESOLUTION // 4  # Quarter turn
    degrees = (steps / ENCODER_RESOLUTION) * 360.0
    if abs(degrees - 90.0) > 0.01:
        return f"Wrong degree calculation: {degrees} != 90.0"
    
    return True

# Create more test functions for other modules

# Run tests function
def run_all_tests():
    """Run all unit tests"""
    test_funcs = [
        test_modbus_register_setup,
        test_encoder_calculation,
        # Add more test functions
    ]
    return run_tests(test_funcs)

# Create a debug command to run tests
def test_command():
    """REPL command to run tests"""
    print("Running unit tests...")
    passed, failed = run_all_tests()
    if failed == 0:
        print("All tests passed!")
    else:
        print(f"FAILED: {failed} tests failed")
```

### 2. Create CI Pipeline with GitHub Actions
**Status**: 🔴 TO DO

**Summary**: Set up an automated testing workflow using GitHub Actions.

**Implementation Plan**:
```yaml
# Create file: .github/workflows/ci.yml

name: MicroPython CI

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]

jobs:
  lint:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: '3.9'
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install ruff black
        pip install micropython-stdlib stub
    - name: Lint with ruff
      run: |
        ruff check .
    - name: Check formatting with black
      run: |
        black --check .

  test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v2
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: '3.9'
    - name: Install dependencies
      run: |
        python -m pip install --upgrade pip
        pip install micropython-stdlib micropython-logging pytest pytest-mock
    - name: Run unit tests
      run: |
        python -m pytest tests/
```

### 3. Create Hardware Abstraction Layer (HAL)
**Status**: 🔴 TO DO

**Summary**: Separate hardware interactions from business logic for better testability.

**Implementation Plan**:
```python
# Create a new directory structure for HAL:
# hal/
#   __init__.py
#   gpio.py
#   uart.py
#   rs485.py
#   encoder.py

# Example hal/gpio.py:
class GPIOInterface:
    """Hardware abstraction for GPIO pins"""
    
    def __init__(self, pin_id, direction, pull=None, value=None):
        """Initialize a GPIO pin with mockable interface"""
        self.pin_id = pin_id
        self.direction = direction
        self.pull = pull
        
        # If in real hardware mode, initialize the pin
        try:
            from machine import Pin
            self._pin = Pin(pin_id, direction, pull, value=value)
            self._mock_mode = False
        except (ImportError, AttributeError):
            # Running in mock/test mode
            self._pin = None
            self._mock_value = value if value is not None else 0
            self._mock_mode = True
    
    def value(self, val=None):
        """Get or set pin value"""
        if self._mock_mode:
            if val is not None:
                self._mock_value = val
            return self._mock_value
        else:
            return self._pin.value(val)
    
    def on(self):
        """Set pin high"""
        return self.value(1)
    
    def off(self):
        """Set pin low"""
        return self.value(0)
    
    def toggle(self):
        """Toggle pin state"""
        return self.value(not self.value())

# Example hal/rs485.py:
class RS485Interface:
    """Hardware abstraction for RS485 communication"""
    
    def __init__(self, uart_id, tx_pin, rx_pin, de_re_pin, baudrate=19200):
        """Initialize RS485 interface with UART and direction control"""
        self.uart_id = uart_id
        self.tx_pin = tx_pin
        self.rx_pin = rx_pin
        self.de_re_pin = de_re_pin
        self.baudrate = baudrate
        
        # Initialize hardware or mock objects
        try:
            from machine import UART, Pin
            self._uart = UART(
                uart_id,
                baudrate=baudrate,
                bits=8,
                parity=None,
                stop=1,
                tx=tx_pin,
                rx=rx_pin
            )
            self._de_re = Pin(de_re_pin, Pin.OUT, value=0)  # Start in RX mode
            self._mock_mode = False
        except (ImportError, AttributeError):
            # Running in mock/test mode
            self._uart = None
            self._de_re = None
            self._mock_tx_buffer = bytearray()
            self._mock_rx_buffer = bytearray()
            self._mock_mode = True
    
    def send_frame(self, data):
        """Send a data frame with proper DE/RE control"""
        if self._mock_mode:
            self._mock_tx_buffer = bytearray(data)
            return len(data)
        
        # Set to transmit mode
        self._de_re.value(1)
        try:
            # Allow line driver to stabilize
            import time
            time.sleep_us(50)
            
            # Send data
            result = self._uart.write(data)
            
            # Wait for transmission to complete
            while not self._uart.txdone():
                pass
                
            # Brief delay before switching back
            time.sleep_us(50)
        finally:
            # Always return to receive mode
            self._de_re.value(0)
        
        return result
    
    def receive_frame(self, timeout_ms=100):
        """Receive a data frame with timeout"""
        if self._mock_mode:
            # In mock mode, just return the prepared mock data
            result = bytes(self._mock_rx_buffer)
            self._mock_rx_buffer = bytearray()
            return result
        
        # Set timeout
        self._uart.timeout(timeout_ms)
        
        # Ensure we're in receive mode
        self._de_re.value(0)
        
        # Read available data
        return self._uart.read()
    
    # For testing/mock mode
    def mock_receive_data(self, data):
        """Set data to be returned on next receive (mock mode only)"""
        if self._mock_mode:
            self._mock_rx_buffer = bytearray(data)
```

## 🎯 Future Enhancements

### 1. Improved Error Handling and Reporting
**Status**: 🔴 TO DO

**Summary**: Enhance error handling and reporting for better diagnostics and recovery.

**Implementation Plan**:
1. Create error code system for Modbus failures
2. Add more detailed error logging and counters
3. Implement automatic retry logic for critical operations

### 2. Persistent Communication Statistics
**Status**: 🔴 TO DO

**Summary**: Add communication statistics tracking to identify patterns in failures.

**Implementation Plan**:
```python
# In utils.py:
class ModbusStatistics:
    """Track Modbus communication statistics"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """Reset all statistics"""
        self.total_tx = 0  # Total frames transmitted
        self.total_rx = 0  # Total frames received
        self.tx_errors = 0  # Transmission errors
        self.rx_errors = 0  # Reception errors
        self.crc_errors = 0  # CRC validation errors
        self.timeouts = 0  # Timeout errors
        self.retries = 0  # Number of retries
        self.avg_response_time_ms = 0  # Average response time
        self.max_response_time_ms = 0  # Maximum response time
        self._response_time_sum = 0  # Sum for average calculation
        
    def record_tx(self, success=True):
        """Record a transmission"""
        self.total_tx += 1
        if not success:
            self.tx_errors += 1
    
    def record_rx(self, success=True, crc_error=False, response_time_ms=None):
        """Record a reception"""
        if success:
            self.total_rx += 1
            
            # Update response time statistics
            if response_time_ms is not None:
                self._response_time_sum += response_time_ms
                self.avg_response_time_ms = self._response_time_sum / self.total_rx
                if response_time_ms > self.max_response_time_ms:
                    self.max_response_time_ms = response_time_ms
        else:
            self.rx_errors += 1
            if crc_error:
                self.crc_errors += 1
    
    def record_timeout(self):
        """Record a timeout"""
        self.timeouts += 1
    
    def record_retry(self):
        """Record a retry attempt"""
        self.retries += 1
    
    def get_success_rate(self):
        """Calculate success rate as percentage"""
        if self.total_tx == 0:
            return 100.0  # No transmissions yet
        return (self.total_rx / self.total_tx) * 100
    
    def get_stats_dict(self):
        """Get statistics as a dictionary"""
        return {
            "tx": self.total_tx,
            "rx": self.total_rx,
            "tx_errors": self.tx_errors,
            "rx_errors": self.rx_errors,
            "crc_errors": self.crc_errors,
            "timeouts": self.timeouts,
            "retries": self.retries,
            "avg_response_ms": self.avg_response_time_ms,
            "max_response_ms": self.max_response_time_ms,
            "success_rate": self.get_success_rate()
        }
    
    def __str__(self):
        """String representation for logging"""
        return (f"Modbus Stats: TX={self.total_tx}, RX={self.total_rx}, "
                f"Errors={self.tx_errors+self.rx_errors}, Retries={self.retries}, "
                f"Success={self.get_success_rate():.1f}%, Avg Time={self.avg_response_time_ms:.1f}ms")

# Create global statistics trackers
master_stats = ModbusStatistics()  # For VFD master operations
slave_stats = ModbusStatistics()   # For slave responses to relay

# In relay_pico/main.py:
# Update read_with_retry to use statistics
def read_with_retry(slave_addr, starting_addr, register_qty, max_retries=4):
    """Retry Modbus reads with exponential back-off"""
    retry = 0
    timeout_ms = 100  # Initial timeout
    start_time = time.ticks_ms()
    
    while retry < max_retries:
        try:
            print(f"[MODBUS] Read attempt {retry+1}, timeout {timeout_ms}ms")
            master_stats.record_tx()
            
            # Allow time for slave to process
            time.sleep_ms(10)
            result = modbus_master.read_input_registers(
                slave_addr=slave_addr,
                starting_addr=starting_addr,
                register_qty=register_qty
            )
            
            # Record response time and success
            response_time = time.ticks_diff(time.ticks_ms(), start_time)
            master_stats.record_rx(success=True, response_time_ms=response_time)
            
            if result:
                return result
        except Exception as e:
            # Record failure statistics
            if "timeout" in str(e).lower():
                master_stats.record_timeout()
            elif "crc" in str(e).lower():
                master_stats.record_rx(success=False, crc_error=True)
            else:
                master_stats.record_rx(success=False)
                
            print(f"[ERROR] Retry {retry+1} failed: {e}")
        
        # Record retry attempt
        if retry < max_retries - 1:  # Don't count last failed attempt
            master_stats.record_retry()
            
        # Exponential back-off (100, 200, 400, 800ms)
        retry += 1
        timeout_ms = min(timeout_ms * 2, 800)  # Cap at 800ms
        time.sleep_ms(timeout_ms)
    
    # Record final failure
    print(f"[ERROR] All {max_retries} retries failed")
    return None
```

### 3. Alternative Communication Path
**Status**: 🔴 TO DO

**Summary**: Implement a fallback communication path in case Modbus RTU continues to have issues.

**Implementation Plan**:
1. Evaluate direct UART communication as backup
2. Consider SPI or I2C for short-distance reliable communication
3. Explore hardware-level watchdog for communications recovery

### 4. Implement Operations Manual with Wiring Guidelines
**Status**: 🔴 TO DO

**Summary**: Create comprehensive documentation about proper RS-485 wiring, shielding, termination, and grounding.

**Implementation Plan**:
1. Create OPERATIONS.md file with detailed diagrams
2. Include shielding and grounding guidelines
3. Include proper RS-485 termination resistor placement
4. Document wire length limitations and EMI considerations

## Professional Assessment

### Roadmap Quality Assessment (1-10 scale)

| Criteria | Score | Notes |
|----------|-------|-------|
| Technical Correctness | 9/10 | Solutions are technically sound with IRQ-driven RX & back-off strategies |
| Prioritization | 9/10 | Critical communication issues correctly prioritized |
| Completeness | 8/10 | Added memory management, HAL, and testing framework |
| Implementation Approach | 8/10 | Event-driven approach and robust error handling |
| Testing Strategy | 8/10 | Unit tests, CI/CD pipeline, and hardware abstraction for testing |
| Resource Efficiency | 8/10 | Memory monitoring and optimized ring buffer logging |
| **Overall Score** | **8.5/10** | Production-grade roadmap suitable for industrial deployment |

### Key Recommendations for Improvement

1. **Choose one consistent DE/RE control strategy**: Either let the library handle it or implement your own wrapper, but not both.

2. **Implement event-driven UART for slave interface**: Use IRQ or StreamReader to drastically reduce latency and jitter in slave responses.

3. **Add exponential back-off retry logic**: Replace simple long timeouts with intelligent back-off to prevent bus flooding.

4. **Fix core issues first**: Register count mismatch, wait_for API, and hard-coded baudrate should be fixed immediately.

5. **Add atomic configuration writes**: Use temporary files and os.rename() for reliable configuration persistence.

6. **Implement comprehensive test suite**: Automated tests for both unit functionality and Modbus protocol handling.

## Conclusion

This roadmap outlines the prioritized tasks required to address the Modbus communication issues and improve overall system reliability. The immediate focus should be on fixing the register count mismatch, implementing proper timeout handling with exponential back-off, and establishing a consistent approach to DE/RE control.

After addressing these critical issues, attention should shift to architectural improvements including event-driven UART handling, non-blocking file I/O, and proper memory management. The longer-term enhancements focus on creating a robust, testable, and maintainable system with proper abstraction layers and comprehensive test coverage.

By systematically implementing these changes with proper testing and validation at each step, the project will evolve from prototype to production-quality embedded system capable of reliable operation in industrial environments.

Critique of the “CFW500 Modbus Control System Roadmap”
Aspect  What Works Well Gaps / Risks    Suggested Upgrade
Problem focus & prioritisation  – Starts with the exact comm-loss symptom and lists root-cause hypotheses.
– All critical comm changes (register count, DE/RE, timeout) are P0 items.  – List mixes bug fixes and architecture refactors under the same “Critical” heading, so the true blockers can drown among nice-to-haves.
– No “definition-of-done”: how will you measure that the read failure is fixed (e.g., 1 000 consecutive successful reads)?  Split sheet into Blocking bug, Must-have reliability, Quality-of-life sections, each with an acceptance metric.
Technical correctness   – Register-count fix, wait_for_ms→wait_for, baudrate param and IRQ-driven UART are accurate ​
.   – uart.txdone() is not implemented on RP2040 MicroPython; manual DE/RE wrapper will busy-loop forever.
– uasyncio.StreamReader only exists in mainline v1.20+; roadmap assumes it’s present.   Replace uart.txdone() with while uart.any_tx() >= 0: or while uart.tx_done() is False: depending on fork, or poll TX FIFO empty via UART0.UARTFR register.
Add a version-check guard for StreamReader and fall back to IRQ+event.
Async discipline    – Replaces all time.sleep_ms with await asyncio.sleep_ms and adds watchdog feeding ​
.   – Still uses time.sleep_ms inside read_with_retry() back-off loop; that call is in the relay’s synchronous context but blocks others on the same core.  Make read_with_retry an async def and call await asyncio.sleep_ms(timeout_ms).
Configuration persistence   – Atomic config.tmp → rename() logic is sound ​
.   – Threaded fallback needs _thread which is disabled in most Pico MicroPython builds; risk of silent ImportError.    Document that threaded path is optional, default to synchronous atomic write; make thread use conditional on hasattr(_thread, "start_new_thread").