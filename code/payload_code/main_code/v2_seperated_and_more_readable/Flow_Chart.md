flowchart TD
    A[MCU loop calls app.tick] --> B[processSerialCommands]
    B --> C{Boot echo printed?}
    C -- No --> D[Print one-time boot echo]
    C -- Yes --> E[Continue]
    D --> E

    E --> F[now = millis]
    F --> G{SD enabled and not mounted?}
    G -- Yes --> H[maybeRemount and refresh sdOk]
    G -- No --> I[skip]
    H --> J
    I --> J

    J --> K{now - lastSensorSample >= SENSOR_SAMPLE_INTERVAL_MS?}
    K -- Yes --> L[Read sensors once into lastSens]
    K -- No --> M[Keep previous lastSens]
    L --> N
    M --> N

    N --> O{now - lastGeigerTick < GEIGER_INTERVAL_MS?}
    O -- Yes --> P[Return from tick]
    O -- No --> Q[lastGeigerTick = now]

    Q --> R[Read geiger count and reset counter]
    R --> S[Push sample {t_ms, geigerCount} into ring buffer]

    S --> T{LoRa enabled and loraOk?}
    T -- No --> U[Set LoRa status code]
    T -- Yes --> V{ring buffer has >= 1 sample?}
    V -- No --> W[Set status = waiting]
    V -- Yes --> X[Build LoRa packet ID|time|value with size limit]

    X --> Y{Packet has records?}
    Y -- No --> Z[Set packet-empty status]
    Y -- Yes --> AA[Transmit with retry]
    AA --> AB{TX success?}
    AB -- Yes --> AC[Pop usedSamples from buffer]
    AB -- No --> AD[Keep samples for retry]

    U --> AE
    W --> AE
    Z --> AE
    AC --> AE
    AD --> AE

    AE[Compose CSV row from runId, time, geiger, latest sensors, LoRa status] --> AF{SD enabled and mounted?}
    AF -- Yes --> AG[Append to SD, handle fail counters]
    AF -- No --> AH[Mark storage not ok]

    AG --> AI
    AH --> AI

    AI --> AJ{Flash enabled and initialized?}
    AJ -- Yes --> AK[Append to LittleFS, handle fail counters]
    AJ -- No --> AL[skip]

    AK --> AM
    AL --> AM

    AM --> AN{Serial logging enabled?}
    AN -- Yes --> AO[Print TX summary]
    AN -- No --> AP[skip]

    AO --> AQ[End tick]
    AP --> AQ

