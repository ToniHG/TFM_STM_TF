# TFM CAN Fault-Tolerant System (STM32F429 Master + STM32F407 Node) — Ada (ADL)

Sistema embebido distribuido compuesto por dos placas STM32 conectadas por bus CAN:
- **Master**: STM32F429I-DISC1 (monitorización + UI en LCD + control)
- **Node**:   STM32F407G-DISC1 (nodo remoto: ejecución de tarea + safety)

Implementa:
- Protocolo CAN versionado (mensajes + CRC)
- Servicios distribuidos (heartbeat, diagnostics, sync opcional)
- Tolerancia a fallos (FDIR: Fault Detection, Isolation & Recovery)
- Interfaz de usuario en LCD (master) para visualizar estado y eventos

> Este repositorio es un *scaffold* profesional pensado para TFM. Completa los bodies `.adb`
> según tu versión de Ada Drivers Library y tu configuración concreta de CAN/LCD.

## Requisitos
- GNAT (o toolchain Ada equivalente)
- Ada Drivers Library (ADL)
- ST-LINK / OpenOCD para flasheo
- (Opcional) Python 3 para herramientas host

## Proyectos (GPR)
- `tfm_stm_tf.gpr`: workspace (referencia common + master + node)
- `common/gpr/common.gpr`: librería compartida
- `boards/master_f429_disc1/gpr/master.gpr`: ejecutable master
- `boards/node_f407_disc/gpr/node.gpr`: ejecutable node

## Build (ejemplo)
Los scripts en `tools/scripts/` son plantillas. Ajusta rutas y toolchain.

- Master: `tools/scripts/build_master.sh`
- Node:   `tools/scripts/build_node.sh`

## Documentación
- Arquitectura: `docs/01_architecture.md`
- Protocolo CAN: `docs/03_can_protocol.md`
- Fault Model + FDIR: `docs/04_fault_model_and_fdir.md`
- Guion demo: `docs/05_demo_scenarios.md`
