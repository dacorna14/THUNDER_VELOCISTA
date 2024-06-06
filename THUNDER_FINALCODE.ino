#include <QTRSensors.h>
#include <IRremote.hpp>

// Definición de pines de salida
#define led1      13
#define led_on    9   
#define mi1       3
#define mi2       4
#define pwmi      6   
#define md1       8
#define md2       7
#define pwmd      5   

#define NUM_SENSORS             8  // Número de sensores usados
#define NUM_SAMPLES_PER_SENSOR  4  // Promedio de 4 muestras por lectura de sensor
#define EMITTER_PIN             9  // El emisor es controlado por el pin digital 9

QTRSensorsAnalog qtra((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Declaración de variables
unsigned int position = 0;

int proporcional = 0;
int integral = 0;
int derivativo = 0;
int salida_pwm = 0;
int proporcional_pasado = 0;
int pinReceptor = 2; // DIGITAL INFRAROJO 
bool active = false; // INICIALIZAMOS ACTIVACIÓN INFRAROJO

// Parámetros PID
int p = 0;
double KP = 1.8; // Constante proporcional
double KI = 0.19; // Constante integral
double KD = 4.7; // Constante derivativa
int VelMax = 30; // Velocidad máxima
int VelFreno = 255; // Velocidad de frenado
int Target = 3500; // Punto de referencia

int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

// Parámetros de sensado
int linea = 0; // 0 para línea negra, 1 para línea blanca
int flanco_color = 0;
int en_linea = 500;
int ruido = 30;

void setup() {
    pinMode(led1, OUTPUT); // Configura el pin del LED como salida
    pinMode(led_on, OUTPUT); // Configura el pin del LED ON como salida
    pinMode(mi1, OUTPUT); // Configura el pin del motor izquierdo 1 como salida
    pinMode(mi2, OUTPUT); // Configura el pin del motor izquierdo 2 como salida
    pinMode(pwmi, OUTPUT); // Configura el pin PWM del motor izquierdo como salida
    pinMode(md1, OUTPUT); // Configura el pin del motor derecho 1 como salida
    pinMode(md2, OUTPUT); // Configura el pin del motor derecho 2 como salida
    pinMode(pwmd, OUTPUT); // Configura el pin PWM del motor derecho como salida

    digitalWrite(led1, HIGH); // Enciende el LED
    delay(200); // Espera 200 ms
    digitalWrite(led1, LOW); // Apaga el LED
    delay(200); // Espera 200 ms

    digitalWrite(led1, HIGH); // Enciende el LED para indicar modo de calibración
    for (int i = 0; i < 200; i++) {
        qtra.calibrate(); // Calibra los sensores
    }
    digitalWrite(led1, LOW); // Apaga el LED para indicar fin de la calibración


    while (true) {
        delay(20);
        digitalWrite(led1, HIGH);
        delay(100);
        digitalWrite(led1, LOW);
        delay(100);
        break;
    }
   
    Serial.begin(9600); // Inicializa la comunicación serial
    IrReceiver.begin(pinReceptor, ENABLE_LED_FEEDBACK); // Inicializa el receptor IR
}

void loop() {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.command == 0x45)
            active = true; // Activa el sistema si recibe el comando correspondiente
        if (IrReceiver.decodedIRData.command == 0x46)
            active = false; // Desactiva el sistema si recibe el comando correspondiente
        IrReceiver.resume(); // Prepara el receptor IR para el próximo comando
    }
    if (active) {
        Serial.println("activado"); // Imprime "activado" si el sistema está activo
        IrReceiver.printIRResultShort(&Serial);
        IrReceiver.resume(); 
        pid(1, VelMax, KP, KI, KD); // Ejecuta el control PID
        frenos_contorno(875); // Verifica si es necesario frenar
        Serial.println(position); // Imprime la posición actual
        delay(2); // Espera 2 ms
    } else {
        Serial.println("desactivado"); // Imprime "desactivado" si el sistema está inactivo
    }
}

void pid(int linea, int VelMax, float Kp, float Ki, float Kd) {
    position = qtra.readLine(sensorValues, QTR_EMITTERS_ON, linea, flanco_color, en_linea, ruido); // Lee la posición de la línea
  
    proporcional = ((position) - Target); // Calcula el término proporcional
    Serial.println(position); // Imprime la posición

    derivativo = (proporcional - proporcional_pasado); // Calcula el término derivativo
    integral = error1 + error2 + error3 + error4 + error5 + error6; // Calcula el término integral
  
    proporcional_pasado = proporcional;
    error6 = error5;
    error5 = error4;
    error4 = error3;
    error3 = error2;
    error2 = error1;
    error1 = proporcional;

    salida_pwm = (proporcional * KP) + (derivativo * KD) + (integral * KI); // Calcula la salida PWM
  
    if (salida_pwm > VelMax) salida_pwm = VelMax; // Limita la salida PWM máxima
    if (salida_pwm < -VelMax) salida_pwm = -VelMax; // Limita la salida PWM mínima
  
    if (salida_pwm < 0) {
        int der = VelMax - salida_pwm; // Ajusta la velocidad del motor derecho
        int izq = VelMax + salida_pwm; // Ajusta la velocidad del motor izquierdo
        if (der >= 255) der = 255;
        if (izq <= 0) izq = 0;
        motores(izq, der); // Controla los motores
    }
    if (salida_pwm > 0) {
        int der = VelMax - salida_pwm;
        int izq = VelMax + salida_pwm;
        if (izq >= 255) izq = 255;
        if (der <= 0) der = 0;
        motores(izq, der); // Controla los motores
    }
}

void frenos_contorno(int flanco_comparacion) {
    if (proporcional <= -Target) { // Si se desvía demasiado a la derecha
        while (true) { 
            digitalWrite(led1, HIGH); // Enciende el LED
            motores(true, VelFreno); // Frena el motor
            qtra.read(sensorValues); // Lee los valores de los sensores
            if (sensorValues[0] < flanco_comparacion || sensorValues[1] < flanco_comparacion || sensorValues[2] < flanco_comparacion || sensorValues[3] < flanco_comparacion || sensorValues[4] < flanco_comparacion || sensorValues[5] < flanco_comparacion || sensorValues[6] < flanco_comparacion || sensorValues[7] < flanco_comparacion) {
                break; // Si detecta la línea, sale del bucle
            }
        }
    }

    if (proporcional >= Target) { // Si se desvía demasiado a la izquierda
        while (true) {
            digitalWrite(led1, HIGH); // Enciende el LED
            motores(false, VelFreno); // Frena el motor
            qtra.read(sensorValues); // Lee los valores de los sensores
            if (sensorValues[7] < flanco_comparacion || sensorValues[6] < flanco_comparacion || sensorValues[5] < flanco_comparacion || sensorValues[4] < flanco_comparacion || sensorValues[3] < flanco_comparacion || sensorValues[2] < flanco_comparacion || sensorValues[1] < flanco_comparacion || sensorValues[0] < flanco_comparacion) {
                break; // Si detecta la línea, sale del bucle
            }
        }
    }
    digitalWrite(led1, LOW); // Apaga el LED
}

void motores(int motor_izq, int motor_der) {
    if (motor_izq >= 0) {
        digitalWrite(mi1, LOW); // Configura la dirección del motor izquierdo
        digitalWrite(mi2, HIGH); 
        analogWrite(pwmi, motor_izq); // Establece la velocidad del motor izquierdo
    } else {
        digitalWrite(mi1, HIGH); 
        digitalWrite(mi2, LOW);
        motor_izq = motor_izq * (-1); 
        analogWrite(pwmi, motor_izq); // Establece la velocidad del motor izquierdo
    }

    if (motor_der >= 0) {
        digitalWrite(md1, LOW); // Configura la dirección del motor derecho
        digitalWrite(md2, HIGH);
        analogWrite(pwmd, motor_der); // Establece la velocidad del motor derecho
    } else {
        digitalWrite(md1, HIGH);
        digitalWrite(md2, LOW);
        motor_der = motor_der * (-1);
        analogWrite(pwmd, motor_der); // Establece la velocidad del motor derecho
    }
}
