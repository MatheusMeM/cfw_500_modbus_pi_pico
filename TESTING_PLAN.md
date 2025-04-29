# Modbus Communication Debugging Test Plan (2025-04-29)

**Current Problem:** Relay Pico (Modbus Master) fails to reliably read Input Registers from Main Pico (Modbus Slave) over RS485 Network 1, despite writes working.

**Goal:** Systematically diagnose the root cause of the Modbus read failures using the First Principles Debugging Roadmap.

**Phase 1: Isolate Main Pico Slave Response**

*   **Step 1.1: Drastically Simplify Main Pico Load & Verify Slave Response**
    *   **Objective:** Determine if Main Pico can respond when minimally loaded.
    *   **Code Changes:**
        *   `main_pico/utils.py`: Set static, non-zero initial values for all Input Registers in `slave_registers['IREGS']`.
        *   `main_pico/main.py`:
            *   Comment out `asyncio.create_task` for `vfd_status_request_task`, `relay_control_task`.
            *   Comment out `initialize_encoder(...)`.
            *   Comment out `await homing(...)`.
            *   Add `print_verbose("[DEBUG SLAVE POLL TASK] Starting...", 3)` at the start of `modbus_slave_poll_task`.
            *   Add `print_verbose("[DEBUG SLAVE POLL TASK] Polling...", 3)` inside the `try` block of `modbus_slave_poll_task` loop.
    *   **Deployment:** Deploy modified `main.py` and `utils.py` to Main Pico. Relay Pico code remains as is (commit `9e849ec` or later).
    *   **Test Procedure:**
        1.  Power on both Picos.
        2.  Connect serial terminals to both Picos.
        3.  Observe logs for ~30 seconds. Do NOT send any commands from Relay Pico initially.
    *   **Expected Outcome / Verification:**
        *   *Main Pico Log:* Shows "[DEBUG SLAVE POLL TASK] Starting..." once. Shows "[DEBUG SLAVE POLL TASK] Polling..." repeatedly (every 200ms). Shows initialization messages indicating tasks/homing disabled.
        *   *Relay Pico Log:* Shows periodic "[MODBUS RX] Reading status registers..." messages (every 5 seconds). **Crucially:** Shows "[STATUS RX]" messages reliably containing the static test values (e.g., `RPM: 12.3`, `VFD: 0xABCD`, `EncSteps: 456`, etc.). The "[ERROR] Modbus RX Error: no data received from slave" messages should be absent.
    *   **Analysis:**
        *   *If Expected Outcome Met:* Problem is likely Main Pico resource contention under full load (Hypothesis 1A). Proceed to re-enable tasks one by one or investigate `uasyncio` scheduling.
        *   *If Expected Outcome NOT Met (Reads Still Fail):* Problem is likely deeper (Modbus library, timing, hardware). Proceed to Step 1.2 or Phase 2.

*   **Step 1.2: Explicitly Check Slave Response Construction (Conditional)**
    *   **Objective:** Verify `umodbus` slave library attempts to build/send FC04 response.
    *   **Action:** Modify `umodbus/modbus.py` on Main Pico to print debug message before sending FC04 response.
    *   **Test:** Deploy modified library. Run Step 1.1 test again.
    *   **Expected Outcome:** See "[UMODBUS SLAVE]... Attempting to send response..." on Main Pico log when Relay attempts read.

**Phase 2: Investigate RS485 Link & Timing (Conditional - If Phase 1 Fails)**

*   **Step 2.1: Reduce Baud Rate**
    *   **Objective:** Eliminate high baud rate as a cause.
    *   **Action:** Change `SLAVE_BAUDRATE` / `RS485_BAUDRATE` to 19200 on **both** Picos. Restore Main Pico tasks.
    *   **Test:** Deploy both. Run `start`/`stop`. Observe if Relay reads status reliably.
*   **Step 2.2: Increase Modbus Timeouts (If Possible)**
    *   **Objective:** Check if default timeouts are too short.
    *   **Action:** Investigate `umodbus` or `machine.UART` for timeout settings. Increase significantly if found. Increase Main Pico poll interval further.
    *   **Test:** Deploy. Run `start`/`stop`. Observe if reads improve.
*   **Step 2.3: Verify DE/RE Timing**
    *   **Objective:** Check RS485 transmit enable timing.
    *   **Action:** Add detailed logging around UART writes/reads and DE/RE pin control (potentially modify library or use spare GPIO + oscilloscope).
    *   **Test:** Deploy. Run `start`/`stop`. Analyze interleaved logs / scope traces.

**Phase 3: Check Wiring & Relay Reception (Conditional - If Phase 2 Fails)**

*   **Step 3.1: Physical Check**
    *   **Objective:** Rule out simple hardware issues.
    *   **Action:** Double-check A/B lines, TX/RX/DE/RE pins, and **common ground**.
*   **Step 3.2: Explicitly Check Relay UART Reception**
    *   **Objective:** Verify if Relay UART receives *any* bytes during expected response time.
    *   **Action:** Modify Relay's Modbus master logic or `uart.read()` to log raw received bytes.
    *   **Test:** Deploy. Run status read. Observe Relay logs for raw bytes.

---
*(Previous test phases related to specific command execution logic are now considered resolved and removed from this active plan).*