# Assignments

## Class Act1
### Exercise Objective
The goal of this exercise is to train you to identify logical FreeRTOS tasks from system behavior, even when no RTOS code is shown.

You should focus on:

- Timing requirements
- Blocking behavior
- Safety and criticality
- Independent execution flows

Think in terms of "what must happen independently", not functions or lines of code.

### System Description
You are given the following description of an embedded system:

#### The system:

- Reads a temperature sensor every 50 ms
- Sends sensor data via Wi-Fi every 2 seconds
- Monitors an emergency button continuously
- Blinks a status LED at 1 Hz
- Stores error messages when failures occur

| Task name | Trigger | Periodic or Event-based| Is time critical? | Can it Block safely? | What if delayed | Priority |
|-----------:|:-----:|:-----:|:-----:|:-----:|:-----:|-------------|
| Temperature sensor | 50ms Timer     | Periodic         |YES|NO|Reading error|Medium|
| WiFi     | 2s timer     | Periodic    |NO|NO|Missing data|Medium|
| Emergency Button | Press button  | Event   |YES|NO|An accident| High|
| LED Status     | 2s timer  | Periodic    |NO|YES|Just a delay of the status|Low|
| Error Message | Error    | Event    |NO|YES|We store it later|Low|


