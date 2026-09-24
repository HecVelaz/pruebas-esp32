# 01-imu-mpu9250 — IMU MPU-9250 / MPU-6500 por I2C

Detecta el módulo en el bus I2C y lee `WHO_AM_I` para saber si es un **MPU-9250** (`0x71`) o un **MPU-6500** (`0x70`). Después imprime el acelerómetro, el giroscopio y la temperatura. El magnetómetro (AK8963) solo existe en el MPU-9250.

Muchos módulos vendidos como "MPU-9250" traen en realidad un MPU-6500, sin magnetómetro. Este programa sirve para comprobarlo.

## Conexiones

**Placa:** Freenove ESP32-S3-WROOM (N16R8) sobre el breakout Freenove v1.2. Los pines del ESP32-S3 se toman de los rótulos del breakout.

| Módulo MPU | Breakout Freenove v1.2 | Nota |
|------------|------------------------|------|
| VCC        | **3V3**                | |
| GND        | **GND**                | |
| SDA        | **GPIO8**              | `PIN_I2C_SDA` en `include/pins.h` |
| SCL        | **GPIO9**              | `PIN_I2C_SCL` en `include/pins.h` |
| AD0        | **GND**                | Dirección I2C `0x68` (`IMU_I2C_ADDR` en `include/pins.h`) |

Pines que el fabricante pide fijar aunque no se usen (hoy **no están conectados**):

| Módulo MPU | Recomendado | Por qué |
|------------|-------------|---------|
| NCS        | 3V3         | En alto fija el modo I2C. En bajo o flotante, el chip puede pasar a SPI. |
| FSYNC      | GND         | El datasheet indica conectarlo a GND si no se usa. |
| INT, EDA, ECL | sin conectar | Son salidas o el bus auxiliar, que no se usa (el AK8963 se lee en bypass). |

Se pueden dejar NCS y FSYNC sin conectar solo si el esquema del módulo confirma que ya tienen resistencias pull-up (NCS) y pull-down (FSYNC). En los GY-9250/6500 suele ser así, pero hay que verificarlo en el esquema o con un multímetro: NCS ≈ 3,3 V y FSYNC ≈ 0 V con el módulo alimentado. Lo mismo vale para AD0, que en esta instalación sí está conectado a GND.

- Si el MPU aparece en `0x69` en lugar de `0x68`, el programa lo usa igual, pero muestra un aviso: AD0 no está a GND.
- Los módulos suelen traer resistencias pull-up en SDA/SCL. Si el escáner no encuentra nada, revisar que existan (4,7 kΩ a 3V3).
- Pines prohibidos en el N16R8: GPIO19/20 (USB nativo) y **GPIO26–37** (flash y PSRAM octal, incluidos 35, 36 y 37).
- En la placa Freenove, GPIO8 y GPIO9 también son líneas de datos del conector de la cámara (pinout ESP32-S3-EYE). No usar la cámara al mismo tiempo que el IMU en estos pines.

## Cómo probar

```bash
cd ~/pruebas-esp32/01-imu-mpu9250
~/.platformio/penv/bin/pio run -t upload -t monitor
```

Cerrar cualquier otro monitor serial antes de flashear.

Salida esperada con un MPU-9250:

```
== Prueba IMU MPU-9250 / MPU-6500 ==
I2C: SDA=GPIO8, SCL=GPIO9
Escaneando bus I2C...
  dispositivo en 0x68
1 dispositivo(s) encontrado(s)
WHO_AM_I en 0x68 = 0x71
Modelo detectado: MPU-9250
Magnetómetro AK8963: OK

acc[g]   0.01  -0.02   1.00 | gyr[dps]     0.3    -1.2     0.4 | T  27.5 C | mag[uT]   21.3  -5.6  -40.2
```

Con un MPU-6500, la columna del magnetómetro no aparece. Si en una línea el magnetómetro no tiene un dato válido, se muestra el motivo en lugar de repetir el último valor: `mag: sin dato nuevo`, `mag: overflow magnético` o `mag: error I2C`.

Si el MPU no inicializa, el mensaje indica la causa: `no se pudo leer WHO_AM_I` (el bus no responde), `WHO_AM_I no es MPU-9250 (0x71) ni MPU-6500 (0x70)` (otro chip) o `falló una escritura I2C durante la configuración`.

Después de activar el modo bypass, el escáner no vuelve a correr, así que el `0x0C` del AK8963 no aparece en la lista.

### Verificaciones

- **Acelerómetro:** en reposo y plano, `z ≈ +1.00 g`. Al girar la placa, el 1 g pasa al eje que apunta hacia arriba.
- **Giroscopio:** en reposo, valores cercanos a 0. Un offset de unos pocos °/s es normal, porque este programa no calibra el bias (`02-imu-ros2` sí lo hace). Al rotar, aparece el eje correspondiente.
- **Magnetómetro:** al rotar horizontalmente, X e Y cambian de signo. El módulo total del campo es del orden de 25–65 µT, sin calibrar hard/soft iron.

## Configuración del driver (`lib/MPU9250`)

| Parámetro | Valor |
|-----------|-------|
| Acelerómetro | ±2 g, 16384 LSB/g |
| Giroscopio | ±250 °/s, 131 LSB/(°/s) |
| DLPF | configurable con `begin(Dlpf)` / `setDlpf()`: 184, 92, 41, 20, 10 o 5 Hz. Este programa usa 41 Hz. |
| Magnetómetro | AK8963 vía bypass I2C, 16 bits (0,15 µT/LSB) con ajuste ASA de fábrica |
| Bus I2C | 400 kHz |

Frecuencias: cada componente trabaja a su propio ritmo.

| Qué | Frecuencia |
|-----|------------|
| Muestreo interno del MPU (registros de acel./gyro) | 200 Hz (`SMPLRT_DIV = 4`) |
| Conversión del AK8963 | 100 Hz (modo continuo 2) |
| Lectura e impresión por serial en este programa | ~5 Hz (`delay(200)` en `loop()`) |

El programa lee el valor más reciente de los registros cada 200 ms. No usa la FIFO, así que las muestras intermedias se descartan. Para una prueba visual alcanza. Para adquirir datos, ver `02-imu-ros2`.

## Resultados

| Fecha | Módulo | WHO_AM_I | Magnetómetro | Observaciones |
|-------|--------|----------|--------------|---------------|
|       |        |          |              |               |
