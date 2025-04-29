# Project Roadmap: CFW500 Pico Control

This document outlines planned improvements and fixes for the Raspberry Pi Pico VFD motor control project, based on recent code reviews. Items are ranked by severity.

## High Severity

1.  **Implement Data Return for Status Commands:**
    *   **Issue:** Commands like `read_speed`, `status`, `read_max_rpm`, `read_offset` are processed by the Main Pico (Slave), but the results are only logged locally. The Relay Pico (Master) only receives an ACK, and the PC user never sees the requested data.
    *   **Solution:**
        *   Define new command/response codes in `protocol.py` (e.g., `CMD_GET_SPEED`, `RSP_SPEED_DATA`, `CMD_GET_STATUS`, `RSP_STATUS_DATA`).
        *   Modify `main_pico/commands.py` for relevant commands to retrieve the data and return it (or signal failure).
        *   Modify `main_pico/main.py` (`slave_task`) to handle these new commands, package the returned data into the payload of a new response frame (e.g., `RSP_SPEED_DATA`), and send it using `utils.send_response`.
        *   Modify `relay_pico/main.py` (`receive_task`) to handle these new response codes, extract the payload, decode it, and print it to the PC console.
    *   **Severity:** High (Core functionality broken for status commands).

## Medium Severity

1.  **Implement Status Polling Task:**
    *   **Issue:** The `status_poll_task` in `relay_pico/main.py` is a placeholder and commented out. There's no automatic way to get periodic updates from the Main Pico.
    *   **Solution:**
        *   Implement the logic within `status_poll_task` on the Relay Pico.
        *   Use a timer (`asyncio.sleep`) for the polling interval.
        *   Define and use appropriate protocol commands (e.g., `CMD_GET_STATUS`) and responses (e.g., `RSP_STATUS_DATA` containing speed, VFD status bits, fault status, etc.).
        *   Use the existing send/receive/retry logic within the task to communicate with the Main Pico.
        *   Parse the response payload and print formatted status to the PC console.
    *   **Severity:** Medium (Important missing feature for usability).

2.  **Resolve `RSP_RAW_RESPONSE` Inconsistency:**
    *   **Issue:** The Relay Pico (Master) handles `RSP_RAW_RESPONSE`, but the Main Pico (Slave) currently never sends it.
    *   **Solution:** Decide on the purpose of `RSP_RAW_RESPONSE`.
        *   *Option A (Remove):* If not needed (e.g., specific data responses like `RSP_SPEED_DATA` are preferred), remove the handling code from `relay_pico/main.py` (`receive_task`).
        *   *Option B (Implement):* Define a use case (e.g., sending unsolicited error messages or verbose logs from Slave) and implement the sending logic in `main_pico/main.py` or `utils.py`.
        *   Update the README to accurately reflect the chosen approach.
    *   **Severity:** Medium (Code inconsistency, potential confusion).

## Low Severity

1.  **Refine NACK with Error Codes:**
    *   **Issue:** The Slave sends a generic `RSP_NACK_RAW` for various failures. The Master/PC doesn't know the specific reason.
    *   **Solution:**
        *   Define error code constants in `protocol.py` (e.g., `ERR_INVALID_CMD`, `ERR_VFD_COMM`, `ERR_PROCESSING`).
        *   Modify `main_pico/commands.py` to return specific error codes on failure.
        *   Modify `main_pico/main.py` (`slave_task`) to include the error code as a 1-byte payload in the `RSP_NACK_RAW` frame sent via `utils.send_response`.
        *   Modify `relay_pico/main.py` (`receive_task`) to extract the error code from NACK payloads and print a more informative error message to the PC.
    *   **Severity:** Low (Improves debugging and user feedback).

2.  **Improve Error Handling in Background Tasks:**
    *   **Issue:** Unhandled exceptions in `status_request_task` or `relay_control_task` on the Main Pico could cause silent task failure.
    *   **Solution:** Wrap the core logic inside the `while True:` loop of these tasks in `main_pico/main.py` with a `try...except Exception as e:` block. Log any caught exceptions using `print_verbose`.
    *   **Severity:** Low (Improves long-term stability).

3.  **Use `asyncio.sleep` in Slave Response Sending:**
    *   **Issue:** `main_pico/utils.py:send_response` uses blocking `time.sleep_ms` for transmission delay.
    *   **Solution:** Replace `time.sleep_ms(delay_ms)` with `await asyncio.sleep_ms(delay_ms)` (or `asyncio.sleep_us`) to prevent blocking other asyncio tasks during the short transmission window. Note: `send_response` itself would need to become an `async def`.
    *   **Severity:** Low (Minor performance/responsiveness improvement).

## Optional Improvements (Lowest Severity)

1.  **Add Modbus Write Confirmation:**
    *   **Issue:** Critical VFD writes (`start`, `stop`, `set_speed`) assume success if no Modbus exception occurs. The VFD might ignore the command silently.
    *   **Solution:** After sending a write command in `cfw500_modbus.py`, add a short delay (`asyncio.sleep_ms`) and then read a relevant status register (e.g., P0680) to confirm the VFD entered the expected state. Modify `commands.py` to handle potential confirmation failures.
    *   **Severity:** Optional (Increases robustness at the cost of performance/complexity).

2.  **Make Motor Parameters Configurable:**
    *   **Issue:** `num_pole_pairs` is hardcoded in `cfw500_modbus.py`.
    *   **Solution:** Read this value from the VFD if possible, or add it as a parameter to `main_pico/config.json` and load it in `cfw500_modbus.py`.
    *   **Severity:** Optional (Improves flexibility).

3.  **Add Initial RS485 Delay:**
    *   **Issue:** Small theoretical chance of bus contention during Pico initialization.
    *   **Solution:** Add a short `await asyncio.sleep_ms(100)` in the `main()` function of both Picos after UART initialization but before starting communication tasks.
    *   **Severity:** Optional (Very low risk mitigation).