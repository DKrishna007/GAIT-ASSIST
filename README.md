# GAIT-ASSIST - Haptic Feedback Robotic Exo-Suit

A wearable robotic exo-suit with integrated haptic feedback system and smart chest belt for gait assistance and rehabilitation.

## Project Overview

GAIT-ASSIST is a rehabilitation engineering project that combines:
- A robotic lower-limb exo-suit for gait assistance
- - Haptic feedback actuators providing sensory cues to the user
  - - A smart chest belt with IMU sensors for posture/balance monitoring
    - - Real-time embedded control system
     
      - ## Repository Structure
     
      - ```
        GAIT-ASSIST/
        ├── cad/            # 3D CAD designs of exo-suit components
        ├── circuit/        # Electronic circuit diagrams and PCB designs
        ├── documents/      # Project documentation and reports
        ├── matlab/         # MATLAB simulations and analysis scripts
        ├── inputs.txt      # System input parameters and configuration
        ├── report.docx     # Full project report
        └── README.md
        ```

        ## System Architecture

        ```
        [Chest Belt IMU] ──► [Arduino Mega] ──► [Haptic Actuators]
                                  │
        [Knee/Hip Sensors] ──────┤
                                  │
                             [Motor Drivers] ──► [Exo-Suit Motors]
        ```

        ## Hardware Components

        | Component | Specification | Purpose |
        |-----------|--------------|---------|
        | Microcontroller | Arduino Mega 2560 | Main control unit |
        | IMU Sensor | MPU-6050 | Chest belt orientation |
        | Haptic Actuators | ERM vibration motors | Tactile feedback |
        | Joint Sensors | Flex sensors + encoders | Gait phase detection |
        | Motor Drivers | L298N H-bridge | Actuating exo-suit |
        | Battery | 7.4V 3000mAh LiPo | Power supply |

        ## Haptic Feedback System

        The haptic feedback provides:
        - **Vibration patterns** for step timing cues
        - - **Intensity variation** based on gait phase (heel strike, toe-off, swing)
          - - **Directional cues** from chest belt for balance correction
           
            - ### Feedback Patterns
           
            - ```
              Heel Strike:  ■■■□□□□□ (3 pulses, 100ms)
              Toe Off:      ■□■□□□□□ (2 pulses, 50ms)
              Balance Alert: ■■■■■■■■ (continuous, ~200ms)
              ```

              ## Gait Phases Detected

              1. **Initial Contact** (Heel Strike): 0-2% gait cycle
              2. 2. **Loading Response**: 0-10% gait cycle
                 3. 3. **Mid Stance**: 10-30% gait cycle
                    4. 4. **Terminal Stance**: 30-50% gait cycle
                       5. 5. **Pre-swing** (Toe Off): 50-60% gait cycle
                          6. 6. **Swing Phase**: 60-100% gait cycle
                            
                             7. ## Running the System
                            
                             8. ### Arduino Setup
                            
                             9. ```bash
                                # Install Arduino IDE, then open circuit/main.ino
                                # Required libraries:
                                # - Wire.h (I2C)
                                # - MPU6050.h
                                # - Servo.h
                                ```

                                ### MATLAB Analysis

                                ```matlab
                                % Run gait analysis
                                cd matlab/
                                run('gait_analysis.m')
                                run('haptic_timing.m')
                                ```

                                ## CAD Design

                                The exo-suit CAD includes:
                                - Lower limb frame (aluminum extrusion)
                                - - Knee joint actuator housing
                                  - - Chest belt mounting bracket
                                    - - Haptic actuator placement fixtures
                                     
                                      - ## Clinical Application
                                     
                                      - This device targets:
                                      - - Stroke rehabilitation patients
                                        - - Neurological gait disorders (Parkinson's, MS)
                                          - - Post-operative knee/hip recovery
                                            - - Athletic performance feedback training
                                             
                                              - ## Results
                                             
                                              - - Gait phase detection accuracy: ~92%
                                                - - Haptic feedback response time: <20ms
                                                  - - Battery life: ~4 hours continuous use
                                                    - - Weight: ~2.1 kg (full system)
                                                     
                                                      - ## Author
                                                     
                                                      - Krishna (DKrishna007) - Biomedical Engineering / Robotics Project
                                                      - 
