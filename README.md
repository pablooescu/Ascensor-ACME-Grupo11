# Ascensor ACME — Control PID con Arduino - Grupo 11

Máster en Ingeniería de Telecomunicaciones | UNIR | Curso 2025-2026

Asignatura: Equipos e Instrumentación Electrónica


---
## Descripción del proyecto

Sistema de ascensor industrial inteligente de 5 plantas simulado en Wokwi.
Mejora respecto a la Actividad 2: sustitución del control de histéresis por un
**control PID con PWM por software** para la regulación térmica de la batería,
junto con control remoto por IR, doble vigilancia DHT22 y automatización lumínica.

🔗 [Simulación en Wokwi](https://wokwi.com/projects/465212058774448129)

## Esquemático del circuito

## Bill of Materials (BOM)

| Designator | Componente | Part Number | Proveedor | Coste unit. |
|---|---|---|---|---|
| U1 | Arduino UNO R3 | ARD-0155 | BricoGeek | 23,95 € |
| M1 | Motor paso a paso NEMA17 | MOT-0001 | BricoGeek | 19,90 € |
| M2 | Driver TB6600 | MOT-0022 | BricoGeek | 19,95 € |
| SV1 | Servomotor FEETECH 25kg | SRV-0025 | BricoGeek | 14,85 € |
| U2, U3 | Sensor DHT22 (x2) | SEN-0181 | BricoGeek | 3,95 € c/u |
| U4 | Receptor IR HX1838 | HX1838 | ElectroYa | 1,30 € |
| U5 | Sensor PIR | SEN-0007 | BricoGeek | 9,80 € |
| U6 | LCD 2004 I2C | ELE-RB-442 | ElectroYa | 6,50 € |
| U7 | RTC DS3231 | ELE-RB-EYB213 | ElectroYa | 2,85 € |
| U8 | LDR ALS-PT19 | SEN-0167 | BricoGeek | 2,95 € |
| D1-D2 | LEDs rojo/blanco | LED-0052 | ElectroYa | 0,11 € c/u |
| R1-R4 | Resistencias varias | — | ElectroYa | < 0,05 € |

**Coste total de componentes: 128,36 €** (sin IVA ni envío)

## Lógica del firmware

## Resultados de prueba

## Autores

Grupo 11 — Candela Delgado Araña, Lucía Diez Platero, Jacobo Díez Venegas,
Pablo Escudero Rodríguez-Bailón, Jorge Luis Cobián Martínez
