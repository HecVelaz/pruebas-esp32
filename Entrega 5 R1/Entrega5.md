**UNIVERSIDAD NACIONAL DE ASUNCIÓN**

**FACULTAD DE INGENIERÍA**

# **ROBOTICA I**

TRABAJO PRÁCTICO FINAL 2025S2

## **"Línea de control de calidad y empaquetado de medicamentos en tabletas"**

---
### **ESTACIÓN [ 2 ]:** [ Q control: Detección y Colocación ]

**Grupo [E2]:**

*   Héctor Dejesús Velázquez Ojeda
*   Mathias Ramón Aguilar Delvalle
*   Victoria Carolina Paredes Frutos
*   Francisco Ramón González Galeano

**ENTREGA [5] :** [Demo: Control en lazo abierto]

**Fecha:** [09/11/2025]

**Versión:** [v2.0]

---

### **1. Control de motores de lazo abierto.**

- Ventajas: simplicidad, baja latencia, menos componentes, costo
  reducido. Aplicable cuando la precisión no es crítica o cuando la dinámica
  es bien conocida y las perturbaciones son pequeñas.
- Desventajas: deriva acumulada, sensibilidad a variaciones de carga/voltaje,
  falta de compensación de fricción o cambios en franja de rendimiento.

Caracteristicas Demo 1:
- Se aplica una señal de entrada PWM a los motores JGB37-520 sin medir
  la salida (posición, velocidad o fuerza) durante la ejecución.
- No hay retroalimentación correccional; el sistema no compensa perturbaciones
  ni errores de modelo.
- Se realiza el cambio de sentido de giro.

Caracteristicas Demo 2:
- Se aplica una señal de entrada PWM al motor DC 5840-31zy montado en la base del robot SCARA de la estación, midiendo la salida (posición) durante la ejecución. En este caso se mide con un encoder incremental de 1000 PPR la posición del eje para poder visualizarla en matlab mediante comunicación serial entre esp32 y la pc.
- No hay retroalimentación correccional; el sistema no compensa perturbaciones
  ni errores de modelo. Solo visualización.
- Se realizan pruebas de escalones de entre 50 y 255 PWM en ambos sentidos de giro. Inicia con 50 PWM luego de 5 segundos aumenta a 100, luego a 150, 200 y 255 del valor PWM. Este valor es el escalado para ajustar el voltaje de entrada de 12V a los motores.

---
### **2. Lista de materiales**

Software

* IDE Arduino
* MATLAB

Hardware
* 1x Arduino Uno
* 1x esp32 Wroom
* 1x Módulo L298N
* 2x Motor DC JGB37-520 12V 200rpm
* 2x Motor 5840-31zy DC 12V
* 1x Encoder rotatorio incremental 38S6G5-B-G24N- 1000 PPR AB
* 1x Fuente DC regulable
* Cables jumper

---

### **3. Código de control de motores.**

#### Demo 1

Programa para controlar dos motores DC JGB37-520 con un módulo L298N, permite controlar de forma independiente la velocidad,
  y giros de cada motor.

```cpp
//definición de pines
const int PinENA = 8;
const int PinIN1 = 7;
const int PinIN2 = 6;
const int PinENB = 5;
const int PinIN3 = 4;
const int PinIN4 = 3;

void setup() {
  // inicializar la comunicación serial a 9600 bits por segundo:
  Serial.begin(9600);
  // configuramos los pines como salida
  pinMode(PinENA, OUTPUT);
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinENB, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);
  //Inicializamos los pines
  analogWrite(PinENA,0);
  digitalWrite (PinIN1, LOW);
  digitalWrite (PinIN2, LOW);
  analogWrite(PinENB,0);
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, LOW);

}

void loop() {

  MotorA_Horario(250); //Motor A horario con velocidad de 250(PWM 0-250)
  MotorB_Horario(250); //Motor B horario con velocidad de 250(PWM 0-250)
  Serial.println("Giro del Motor A y B en sentido horario. PWM:250");
  delay(5000);

  MotorA_Stop(); //Motor Apagado
  MotorB_Stop(); //Motor Apagado
  Serial.println("Motor A y B detenidos");
  delay(3000);

  MotorA_Antihorario(150);  //Motor A horario con velocidad de 150(PWM 0-250)
  MotorB_Antihorario(150);  //Motor B horario con velocidad de 150(PWM 0-250)
  Serial.println("Giro del Motor A y B en sentido antihorario. PWM:150");
  delay(5000);

  MotorA_Stop(); //Motor Apagado
  MotorB_Stop(); //Motor Apagado
  Serial.println("Motor A y B detenidos");
  delay(3000);

}

//función para girar el motor A en sentido horario
void MotorA_Horario(int velocidad) //velocidad 0-250
{
  digitalWrite (PinIN1, HIGH);
  digitalWrite (PinIN2, LOW);
  analogWrite(PinENA,velocidad);
}
//función para girar el motor A en sentido anthorario
void MotorA_Antihorario(int velocidad) //velocidad 0-250
{
  digitalWrite (PinIN1, LOW);
  digitalWrite (PinIN2, HIGH);
  analogWrite(PinENA,velocidad);
}

//función para apagar el motor A
void MotorA_Stop()
{
  digitalWrite (PinIN1, LOW);
  digitalWrite (PinIN2, LOW);
  analogWrite(PinENA,0);
}

//función para girar el motor B en sentido horario
void MotorB_Horario(int velocidad) //velocidad 0-250
{
  digitalWrite (PinIN3, HIGH);
  digitalWrite (PinIN4, LOW);
  analogWrite(PinENB,velocidad);
}
//función para girar el motor B en sentido anthorario
void MotorB_Antihorario(int velocidad) //velocidad 0-250
{
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, HIGH);
  analogWrite(PinENB,velocidad);
}

//función para apagar el motor B
void MotorB_Stop()
{
  digitalWrite (PinIN3, LOW);
  digitalWrite (PinIN4, LOW);
  analogWrite(PinENB,0);
}

```

#### Demo 2

Programa para controlar un motor 5840-31zy con un módulo L298N, permite controlar la velocidad y giro del motor. Se imprime por serial los datos para visualización en matlab.

```cpp
#include "driver/ledc.h"

// -------------------- Pines --------------------
const byte encA = 32;
const byte encB = 33;
const byte PWMA = 26;
const byte PWMB = 25;
const byte ledok = 2;

// -------------------- PWM --------------------
const int frecPWM = 20000;     // 20 kHz
const int resolucionPWM = 8;   // 8 bits (0-255)
const int potencia[5] = {50, 100, 150, 200, 255};  // niveles PWM de prueba

// -------------------- Variables --------------------
volatile long contador = 0;
bool sentido = true;
bool start = false; // estado inicial
bool pruebasCompletadas = false;

// -------------------- Tiempos --------------------
unsigned long lastTime = 0;
unsigned long SampleTime = 50;   // 50 ms (20 Hz)
unsigned long tInicio = 0;
unsigned long tiempoNivel = 0;
const unsigned long DURACION_NIVEL = 5000; // 5 segundos por nivel

// -------------------- Estado de pruebas --------------------
int nivelActual = 0;
bool fasePositiva = true;

// -------------------- Interrupción del encoder --------------------
void IRAM_ATTR leerEncoder() {
  if (digitalRead(encB) == HIGH) {
    contador++;
    sentido = true;
  } else {
    contador--;
    sentido = false;
  }
}

// -------------------- Funciones motor --------------------
void moverMotor(double salida) {
  int pwm = constrain(abs(salida), 0, 255);
  if (salida > 0) {
    ledcWrite(PWMA, pwm);
    ledcWrite(PWMB, 0);
  } else {
    ledcWrite(PWMB, pwm);
    ledcWrite(PWMA, 0);
  }
}

void detenerMotor() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

// -------------------- Iniciar prueba --------------------
void iniciarPrueba() {
  start = true;
  digitalWrite(ledok, HIGH);
  tiempoNivel = millis();
  nivelActual = 0;
  fasePositiva = true;
  pruebasCompletadas = false;

  Serial.println("INICIO_PRUEBAS");
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Configurar pines
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(ledok, OUTPUT);
  digitalWrite(ledok, LOW);

  // Configurar PWM con nueva API LEDC (Core 3.x.x)
  ledcAttach(PWMA, frecPWM, resolucionPWM);
  ledcAttach(PWMB, frecPWM, resolucionPWM);

  // Detener motor
  detenerMotor();

  // Configurar interrupción del encoder
  attachInterrupt(digitalPinToInterrupt(encA), leerEncoder, RISING);

  delay(1000);
  Serial.println("SISTEMA_LISTO");
}

// -------------------- LOOP --------------------
void loop() {
  static unsigned long startTime = millis();
  unsigned long now = millis();

  // Iniciar pruebas si no se han iniciado
  if (!start && !pruebasCompletadas) {
    iniciarPrueba();
  }

  // Lógica de pruebas automáticas
  if (start && !pruebasCompletadas) {
    // Verificar si es tiempo de cambiar de nivel
    if (now - tiempoNivel >= DURACION_NIVEL) {
      nivelActual++;
      tiempoNivel = now;

      // Verificar si hemos completado todos los niveles en la fase actual
      if (nivelActual >= 5) {
        if (fasePositiva) {
          // Completada fase positiva, iniciar fase negativa
          fasePositiva = false;
          nivelActual = 0;
          Serial.println("INICIANDO_FASE_NEGATIVA");
        } else {
          // Completadas ambas fases
          pruebasCompletadas = true;
          detenerMotor();
          digitalWrite(ledok, LOW);
          Serial.println("FIN_PRUEBAS");
          return;
        }
      }

      // Informar cambio de nivel
      Serial.print("CAMBIO_NIVEL:");
      Serial.print(nivelActual);
      Serial.print(",PWM:");
      Serial.print(potencia[nivelActual]);
      Serial.print(",SENTIDO:");
      Serial.println(fasePositiva ? "POSITIVO" : "NEGATIVO");
    }

    // Aplicar PWM según fase actual
    int pwmAplicar = potencia[nivelActual];
    if (!fasePositiva) {
      pwmAplicar = -pwmAplicar; // Sentido inverso
    }
    moverMotor(pwmAplicar);
  }

  // Mostrar datos cada 50 ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 50) {
    lastPrint = now;

    // Solo imprimir si las pruebas están en curso
    if (start && !pruebasCompletadas) {
      float tiempoSegundos = (now - startTime) / 1000.0;

      // Enviar datos en formato CSV para fácil procesamiento
      Serial.print(tiempoSegundos, 3);
      Serial.print(",");
      Serial.print(contador);
      Serial.print(",");
      Serial.print(potencia[nivelActual] * (fasePositiva ? 1 : -1));
      Serial.print(",");
      Serial.println(fasePositiva ? 1 : -1);
    }
  }

  // Si las pruebas están completadas, parpadear LED
  if (pruebasCompletadas) {
    static unsigned long lastBlink = 0;
    if (now - lastBlink >= 500) {
      lastBlink = now;
      digitalWrite(ledok, !digitalRead(ledok));
    }
  }
}

```

---

### **4. Resultados**

Videos de la demostración del control en lazo abierto:

##### Demo 1

[![Demo motores1](https://img.youtube.com/vi/869SbaP6YNM/0.jpg)](https://youtu.be/869SbaP6YNM)

##### Demo 2

[![Demo motores2](https://img.youtube.com/vi/rI0R6aCKIVA/0.jpg)](https://youtu.be/rI0R6aCKIVA)

<div align="center">
  <img src="https://drive.google.com/uc?id=1jzzm06LHwOt-3UvJzoX9IIDhJjKwEYdj" width="800" />
  <p><b>Fig. 1.</b> Resultados Demo 2.</p>
</div>

---

### **5. Referencias**

[1] Naylamp Mechatronics, “Ejemplos_L298N”, GitHub, Disponible en: https://github.com/naylampmechatronics/Ejemplos_L298N.

[2] **Proyectos Robóticos**, *Control PID para Arduino*. Disponible en:
<https://sites.google.com/site/proyectosroboticos/control-de-motores/control-pid-mejorado?authuser=0>.

---

### **6. Carpeta del proyecto**
Carpeta del grupo E2:
[TPF - Estación 2](https://drive.google.com/drive/folders/1JggH_t1V1V1kcT6KU79V80s9d-Px1JNs?usp=sharing)
