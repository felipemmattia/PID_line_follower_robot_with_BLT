/*
 * Robô Seguidor de Linha com ESP32 (38 pinos)
 * Sistema PID com Controle de Velocidade Adaptativo
 * Otimizado para curvas fechadas e alta performance
 *
 * Hardware:
 * - ESP32 (38 pinos)
 * - 2 motores DC 6V
 * - Array de 8 sensores de refletância
 * - Driver de motores DRV8833
 *
 * Autor: Felipe Mattia
 * Data: 2025
 */

#include <BluetoothSerial.h>
#include <Arduino.h>

// Configurações dos pinos (compatível com BT_PIDtest.ino)
// Motores - DRV8833 usa 2 pinos PWM por motor
#define MOTOR_ESQUERDA_IN1 33
#define MOTOR_ESQUERDA_IN2 25
#define MOTOR_DIREITA_IN1 36
#define MOTOR_DIREITA_IN2 27

// Pinos dos sensores de refletância (compatível com BT_PIDtest.ino)
const int sensores[] = {21, 16, 15, 17, 18, 19, 23, 14};
const int NUM_SENSORES = 8;

// Botões (compatível com BT_PIDtest.ino)
#define Bcalibrate 34
#define Bstart 35

// Configurações PID (variáveis para permitir ajuste via Bluetooth)
float KP = 2.5f;       // Proporcional
float KI = 0.1f;       // Integral
float KD = 0.8f;       // Derivativo

// Configurações de velocidade (variáveis para ajuste em tempo real)
int VELOCIDADE_BASE = 200;      // Velocidade base (0-255)
int VELOCIDADE_MIN  = 80;       // Velocidade mínima em curvas
int VELOCIDADE_MAX  = 255;      // Velocidade máxima
float THRESHOLD_CURVA = 0.6f;   // Threshold para detectar curvas

// Variáveis adicionais do BT_PIDtest.ino
int maxbasespeed = 75;           // Velocidade base máxima
int basespeed = 0;               // Velocidade base atual
int maxspeed = 125;              // Velocidade máxima (compatível com app)
int minspeed = -125;             // Velocidade mínima (compatível com app)

// Variáveis PID
float erro = 0;
float erro_anterior = 0;
float erro_integral = 0;
float erro_derivativo = 0;
float saida_pid = 0;

// Multiplicadores de escala do PID (compatível com app MIT App Inventor)
// Ganho efetivo = K / 10^multiX
uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;

// Variáveis PID adicionais do BT_PIDtest.ino
float Pvalue, Ivalue, Dvalue;
uint16_t position;

// Variáveis de controle
int velocidade_esquerda = VELOCIDADE_BASE;
int velocidade_direita = VELOCIDADE_BASE;
float erro_medio = 0;
bool curva_detectada = false;

// Configuração de PWM
#define FREQ_PWM 5000
#define RESOLUCAO_PWM 8
// Canais PWM (dois por motor no DRV8833)
#define CH_ESQ_IN1 0
#define CH_ESQ_IN2 1
#define CH_DIR_IN1 2
#define CH_DIR_IN2 3

// Bluetooth
BluetoothSerial BT;
bool bt_conectado = false;

// Controle geral via BT (compatível com BT_PIDtest.ino)
volatile bool robo_ativo = true;
volatile bool solicitar_calibracao = false;
bool calibrate = false;
bool activate_PID = false;
bool Ccalibrate = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Robô Seguidor de Linha - Inicializando...");
  
  // Configuração dos pinos dos motores (DRV8833)
  pinMode(MOTOR_ESQUERDA_IN1, OUTPUT);
  pinMode(MOTOR_ESQUERDA_IN2, OUTPUT);
  pinMode(MOTOR_DIREITA_IN1, OUTPUT);
  pinMode(MOTOR_DIREITA_IN2, OUTPUT);

  // PWM: 2 canais por motor (controle de direção e frenagem ativa se necessário)
  ledcSetup(CH_ESQ_IN1, FREQ_PWM, RESOLUCAO_PWM);
  ledcSetup(CH_ESQ_IN2, FREQ_PWM, RESOLUCAO_PWM);
  ledcSetup(CH_DIR_IN1, FREQ_PWM, RESOLUCAO_PWM);
  ledcSetup(CH_DIR_IN2, FREQ_PWM, RESOLUCAO_PWM);
  ledcAttachPin(MOTOR_ESQUERDA_IN1, CH_ESQ_IN1);
  ledcAttachPin(MOTOR_ESQUERDA_IN2, CH_ESQ_IN2);
  ledcAttachPin(MOTOR_DIREITA_IN1, CH_DIR_IN1);
  ledcAttachPin(MOTOR_DIREITA_IN2, CH_DIR_IN2);
  
  // Configuração dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(sensores[i], INPUT);
  }
  
  // Configuração dos botões (compatível com BT_PIDtest.ino)
  pinMode(Bcalibrate, INPUT);
  pinMode(Bstart, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Aguarda estabilização dos sensores
  delay(1000);
  Serial.println("Sistema inicializado com sucesso!");
  
  // Bluetooth (compatível com BT_PIDtest.ino)
  if (BT.begin("ESP_TEST")) {
    Serial.println("Bluetooth iniciado: ESP_TEST");
  } else {
    Serial.println("Falha ao iniciar Bluetooth");
  }

  Serial.println("Pressione 'c' para iniciar calibração guiada (Branco/Preto/Verde)");
  
  Serial.println("Robô pronto para funcionar!");
}

void loop() {
  lerBluetooth();
  
  // Leitura dos botões físicos (compatível com BT_PIDtest.ino)
  if (digitalRead(Bcalibrate) == HIGH) {
    calibrate = !calibrate;
    delay(200); // Debounce
  }
  if (digitalRead(Bstart) == HIGH) {
    activate_PID = !activate_PID;
    delay(200); // Debounce
  }

  if (!robo_ativo) {
    paradaEmergencia();
    delay(30);
    return;
  }

  if (solicitar_calibracao) {
    solicitar_calibracao = false;
    calib_por_bt = true;
    calibrarSensores();
    calib_por_bt = false;
  }

  if (calibrate == true) {
    activate_PID = false;
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    calibrarSensores();
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    calibrate = false;
    Ccalibrate = true;
  } else if (activate_PID == true && calibrate == false && Ccalibrate == true) {
    // Executa o loop principal do PID
    // Verifica se os sensores foram calibrados
    if (!sensores_calibrados) {
      Serial.println("Sensores não calibrados! Pressione 'c' para calibrar");
      if (Serial.available()) {
        char comando = Serial.read();
        if (comando == 'c' || comando == 'C') {
          calibrarSensores();
        }
      }
      delay(100);
      return;
    }
    
    // Leitura dos sensores com filtro de ruído
    int valores_sensores[NUM_SENSORES];
    lerSensores(valores_sensores);

    // Ação por detecção de VERDE (prioritária)
    if (manobraVerde(valores_sensores)) {
      // Executou manobra especial, pula controle nesta iteração
      delay(5);
      return;
    }
    
    // Cálculo do erro com média ponderada
    calcularErro(valores_sensores);
    
    // Aplicação do PID
    aplicarPID();
    
    // Aplicação dos comandos aos motores
    aplicarComandosMotores();
    
    // Debug (opcional)
    if (millis() % 100 == 0) {
      debugInfo();
    }
    
    // Delay mínimo para estabilidade
    delay(5);
  } else {
    paradaEmergencia();
    delay(30);
    return;
  }
}

void lerSensores(int valores[]) {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int soma = 0;
    for (int j = 0; j < 3; j++) {
      soma += analogRead(sensores[i]);
      delayMicroseconds(30);
    }
    int valor_medio = soma / 3;

    if (sensores_calibrados) {
      // Normaliza 0..1000 (branco->0, preto->1000)
      int val = map(valor_medio, valores_minimos[i], valores_maximos[i], 0, 1000);
      valores[i] = constrain(val, 0, 1000);
    } else {
      valores[i] = valor_medio;
    }
  }
}

void calcularErro(int valores[]) {
  // Cálculo do erro usando média ponderada
  float soma_ponderada = 0;
  float soma_pesos = 0;
  int cont_ativos = 0;
  
  for (int i = 0; i < NUM_SENSORES; i++) {
    float peso = (i - (NUM_SENSORES - 1) / 2.0) * 2.0; // Peso de -7 a 7
    
    // Detecta linha preta (valores altos = linha detectada)
    if (valores[i] > 500) { // Threshold de 500/1000
      soma_ponderada += peso;
      soma_pesos += abs(peso);
      cont_ativos++;
    }
  }
  
  if (soma_pesos > 0) {
    float pos = (soma_ponderada / soma_pesos);
    erro = erro * 0.6 + pos * 0.4;
    ultima_posicao_valida = pos;
    linha_perdida = false;
    tempo_ultima_linha = millis();
  } else {
    // Gap handling otimizado: manter direção do último erro com rampa de busca
    linha_perdida = true;
    unsigned long dt = millis() - tempo_ultima_linha;
    float direcao = (erro >= 0) ? 1.0 : -1.0;
    if (abs(ultima_posicao_valida) > 0.1) {
      direcao = (ultima_posicao_valida >= 0) ? 1.0 : -1.0;
    }
    float ganho_busca = constrain(0.3 + 0.001 * dt, 0.3, 1.0);
    erro = direcao * ganho_busca; // cresce suavemente até 1.0
  }
}

void aplicarPID() {
  // Cálculo PID (compatível com BT_PIDtest.ino)
  int P = (int)erro;
  int I = (int)erro_integral + (int)erro;
  int D = (int)erro - (int)erro_anterior;
  
  // Atualiza variáveis
  erro_integral = (float)I;
  erro_derivativo = (float)D;
  erro_anterior = erro;
  
  // Limitação do erro integral para evitar windup
  if (abs(erro_integral) > 50) {
    erro_integral = (erro_integral > 0) ? 50 : -50;
    I = (int)erro_integral;
  }
  
  // Cálculo da saída PID usando multiplicadores
  Pvalue = (KP / pow(10, multiP)) * P;
  Ivalue = (KI / pow(10, multiI)) * I;
  Dvalue = (KD / pow(10, multiD)) * D;
  
  // Cálculo da velocidade dos motores
  float motorspeed = Pvalue + Ivalue + Dvalue;
  
  // Separando a velocidade esquerda e direita
  int leftspeed = basespeed + (int)motorspeed;
  int rightspeed = basespeed - (int)motorspeed;
  
  // Limitando as velocidades
  leftspeed = constrain(leftspeed, minspeed, maxspeed);
  rightspeed = constrain(rightspeed, minspeed, maxspeed);
  
  // Aplica as velocidades
  velocidade_esquerda = leftspeed;
  velocidade_direita = rightspeed;
}

void controlarVelocidadeAdaptativa() {
  // Detecção de curvas baseada no erro
  float erro_absoluto = abs(erro);
  
  if (erro_absoluto > THRESHOLD_CURVA) {
    curva_detectada = true;
    // Redução progressiva da velocidade baseada no erro
    float fator = constrain((erro_absoluto - THRESHOLD_CURVA) / (1.0 - THRESHOLD_CURVA), 0.0, 1.0);
    int velocidade_alvo = VELOCIDADE_BASE - (int)((VELOCIDADE_BASE - VELOCIDADE_MIN) * fator);
    
    velocidade_esquerda = velocidade_alvo - (int)saida_pid;
    velocidade_direita  = velocidade_alvo + (int)saida_pid;
  } else {
    if (curva_detectada) {
      // Retorno gradual à velocidade normal
      velocidade_esquerda = VELOCIDADE_BASE - (int)saida_pid;
      velocidade_direita  = VELOCIDADE_BASE + (int)saida_pid;
      
      // Verifica se estabilizou
      if (abs(erro) < 0.1) {
        curva_detectada = false;
      }
    } else {
      // Operação normal em linha reta
      velocidade_esquerda = VELOCIDADE_BASE - (int)saida_pid;
      velocidade_direita  = VELOCIDADE_BASE + (int)saida_pid;
    }
  }
  
  // Limitação das velocidades
  velocidade_esquerda = constrain(velocidade_esquerda, 0, VELOCIDADE_MAX);
  velocidade_direita = constrain(velocidade_direita, 0, VELOCIDADE_MAX);
}

void aplicarComandosMotores() {
  // DRV8833: usa 2 PWMs por motor. Direção por qual canal recebe PWM.
  // Motor esquerdo
  if (velocidade_esquerda >= 0) {
    ledcWrite(CH_ESQ_IN1, velocidade_esquerda);
    ledcWrite(CH_ESQ_IN2, 0);
  } else {
    int v = abs(velocidade_esquerda);
    ledcWrite(CH_ESQ_IN1, 0);
    ledcWrite(CH_ESQ_IN2, v);
  }

  // Motor direito
  if (velocidade_direita >= 0) {
    ledcWrite(CH_DIR_IN1, velocidade_direita);
    ledcWrite(CH_DIR_IN2, 0);
  } else {
    int v = abs(velocidade_direita);
    ledcWrite(CH_DIR_IN1, 0);
    ledcWrite(CH_DIR_IN2, v);
  }
}

void debugInfo() {
  Serial.print("Erro: ");
  Serial.print(erro, 3);
  Serial.print(" | PID: ");
  Serial.print(saida_pid, 1);
  Serial.print(" | Vel_E: ");
  Serial.print(velocidade_esquerda);
  Serial.print(" | Vel_D: ");
  Serial.print(velocidade_direita);
  Serial.print(" | Curva: ");
  Serial.println(curva_detectada ? "SIM" : "NAO");
}

void BTmonitor() {
  Serial.print("KP: ");
  Serial.print(KP);
  Serial.print(" KI:");
  Serial.print(KI);
  Serial.print(" KD: ");
  Serial.print(KD);
  Serial.print(" Multi P: ");
  Serial.print(multiP);
  Serial.print(" Multi I: ");
  Serial.print(multiI);
  Serial.print(" Multi D: ");
  Serial.print(multiD);
  Serial.print(" min speed: ");
  Serial.print(minspeed);
  Serial.print(" max speed: ");
  Serial.print(maxspeed);
  Serial.print(" basespeed: ");
  Serial.print(basespeed);
  Serial.println();
}

// Função para parada de emergência
void paradaEmergencia() {
  ledcWrite(CH_ESQ_IN1, 0);
  ledcWrite(CH_ESQ_IN2, 0);
  ledcWrite(CH_DIR_IN1, 0);
  ledcWrite(CH_DIR_IN2, 0);
}

// Variáveis para calibração
int valores_minimos[NUM_SENSORES];
int valores_maximos[NUM_SENSORES];
bool sensores_calibrados = false;
// Flags de calibração via Bluetooth
bool calib_por_bt = false;

// Calibração e faixa do VERDE (percentuais entre branco e preto)
// Em muitas pistas, o verde reflete mais que preto e menos que branco.
// Ajuste fino via BT: GL (0.0..1.0), GH (0.0..1.0)
float verde_low_pct = 0.35f;  // limiar inferior relativo (entre branco e preto)
float verde_high_pct = 0.65f; // limiar superior relativo

// Estado de linha perdida
bool linha_perdida = false;
float ultima_posicao_valida = 0.0f;
unsigned long tempo_ultima_linha = 0;

// Função de calibração
// - Se disparada via Bluetooth (Ca:), roda modo automático por tempo, capturando extremos
// - Se via Serial ('c'), mantém modo guiado simples (branco->preto) usando apenas as teclas
void calibrarSensores() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    valores_minimos[i] = 1023;
    valores_maximos[i] = 0;
  }

  if (calib_por_bt) {
    Serial.println("=== CALIBRAÇÃO AUTOMÁTICA (BT) ===");
    Serial.println("Mova o robô por áreas BRANCAS e PRETAS por ~4s...");
    unsigned long t0 = millis();
    while (millis() - t0 < 4000) {
      for (int i = 0; i < NUM_SENSORES; i++) {
        int v = analogRead(sensores[i]);
        if (v < valores_minimos[i]) valores_minimos[i] = v;
        if (v > valores_maximos[i]) valores_maximos[i] = v;
      }
      delay(5);
    }
    finalizarCalibracao();
    return;
  }

  Serial.println("=== CALIBRAÇÃO GUIADA (SERIAL) ===");
  Serial.println("1) Coloque sobre BRANCO e pressione 'b'");
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'b' || c == 'B') break;
    }
    delay(20);
  }
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int v = analogRead(sensores[i]);
      if (v < valores_minimos[i]) valores_minimos[i] = v;
    }
    delay(10);
  }
  Serial.println("BRANCO ok.");

  Serial.println("2) Coloque sobre PRETO e pressione 'p'");
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p' || c == 'P') break;
    }
    delay(20);
  }
  t0 = millis();
  while (millis() - t0 < 1500) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int v = analogRead(sensores[i]);
      if (v > valores_maximos[i]) valores_maximos[i] = v;
    }
    delay(10);
  }
  Serial.println("PRETO ok.");

  finalizarCalibracao();
}

void calibrarBranco() {
  Serial.println("Calibrando BRANCO, mova o robô sobre a superfície branca por 3 segundos");
  
  unsigned long tempo_inicio = millis();
  while (millis() - tempo_inicio < 3000) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int valor = analogRead(sensores[i]);
      if (valor < valores_minimos[i]) {
        valores_minimos[i] = valor;
      }
    }
    delay(50);
  }
  
  Serial.println("BRANCO calibrado!");
  mostrarCalibracao();
}

void calibrarPreto() {
  Serial.println("Calibrando PRETO, mova o robô sobre a superfície preta por 3 segundos");
  
  unsigned long tempo_inicio = millis();
  while (millis() - tempo_inicio < 3000) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int valor = analogRead(sensores[i]);
      if (valor > valores_maximos[i]) {
        valores_maximos[i] = valor;
      }
    }
    delay(50);
  }
  
  Serial.println("PRETO calibrado!");
  mostrarCalibracao();
}

void finalizarCalibracao() {
  Serial.println("=== CALIBRAÇÃO FINALIZADA ===");
  mostrarCalibracao();
  
  // Verifica se a calibração é válida
  bool calibracao_valida = true;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (valores_maximos[i] - valores_minimos[i] < 100) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" pode ter calibração insuficiente!");
      calibracao_valida = false;
    }
  }
  
  if (calibracao_valida) {
    sensores_calibrados = true;
    Serial.println("Calibração válida! Iniciando robô");
  } else {
    Serial.println("Recomenda-se recalibrar os sensores!");
  }
}

void mostrarCalibracao() {
  Serial.println("Valores de calibração:");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(valores_minimos[i]);
    Serial.print(" Max=");
    Serial.print(valores_maximos[i]);
    Serial.print(" Range=");
    Serial.println(valores_maximos[i] - valores_minimos[i]);
  }
}

void mostrarValoresSensores() {
  Serial.print("Valores atuais: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print(analogRead(sensores[i]));
    Serial.print(" ");
  }
  Serial.println();
}

// -----------------------
// Detecção de Verde
// Usa faixa relativa entre BRANCO (min) e PRETO (max). Considera VERDE dentro de [GL, GH]
// Contabiliza esquerda (0..3) e direita (4..7) para acionar manobra
bool detectarVerdeLadoEsq(const int valores_norm[]) {
  int cont = 0;
  for (int i = 0; i < NUM_SENSORES/2; i++) {
    int faixa = valores_norm[i]; // 0..1000
    int low = (int)(verde_low_pct * 1000);
    int high = (int)(verde_high_pct * 1000);
    if (faixa >= low && faixa <= high) cont++;
  }
  return cont >= 2; // pelo menos 2 sensores
}

bool detectarVerdeLadoDir(const int valores_norm[]) {
  int cont = 0;
  for (int i = NUM_SENSORES/2; i < NUM_SENSORES; i++) {
    int faixa = valores_norm[i]; // 0..1000
    int low = (int)(verde_low_pct * 1000);
    int high = (int)(verde_high_pct * 1000);
    if (faixa >= low && faixa <= high) cont++;
  }
  return cont >= 2;
}

bool manobraVerde(const int valores_norm[]) {
  bool esq = detectarVerdeLadoEsq(valores_norm);
  bool dir = detectarVerdeLadoDir(valores_norm);

  if (esq && dir) {
    // Meia volta: gira no próprio eixo
    giroEmEixo(220, 450); // velocidade, tempo ms (ajustável)
    return true;
  } else if (esq) {
    curvaAcentuada(-1); // esquerda
    return true;
  } else if (dir) {
    curvaAcentuada(1);  // direita
    return true;
  }
  return false;
}

void curvaAcentuada(int direcao) { // -1 esq, 1 dir
  int v = VELOCIDADE_BASE;
  velocidade_esquerda = constrain(v - direcao*200, 0, VELOCIDADE_MAX);
  velocidade_direita  = constrain(v + direcao*200, 0, VELOCIDADE_MAX);
  aplicarComandosMotores();
  delay(120);
}

void giroEmEixo(int velocidade, int tempo_ms) {
  velocidade_esquerda = velocidade;
  velocidade_direita  = -velocidade;
  aplicarComandosMotores();
  delay(tempo_ms);
}

// -----------------------
// Bluetooth: comandos similares ao BT_PIDtest.ino para ajuste em tempo real
// Formato por linhas: KP:, KI:, KD:, VB:, VMIN:, VMAX:, THC:, GL:, GH:, CALW, CALB, START, STOP

bool bt_cmd_cal_w = false;
bool bt_cmd_cal_b = false;

void lerBluetooth() {
  if (!BT.available()) return;
  String data = BT.readStringUntil('\n');
  data.trim();
  if (data.length() == 0) return;

  // Compatível com app MIT App Inventor (BT_PIDtest.ino)
  if (data.startsWith("KP:")) { KP = data.substring(3).toFloat(); }
  else if (data.startsWith("KI:")) { KI = data.substring(3).toFloat(); }
  else if (data.startsWith("KD:")) { KD = data.substring(3).toFloat(); }

  else if (data.startsWith("MP:")) { int v = data.substring(3).toInt(); multiP = (uint8_t)constrain(v, 0, 6); }
  else if (data.startsWith("MI:")) { int v = data.substring(3).toInt(); multiI = (uint8_t)constrain(v, 0, 6); }
  else if (data.startsWith("MD:")) { int v = data.substring(3).toInt(); multiD = (uint8_t)constrain(v, 0, 6); }

  else if (data.startsWith("Min:")) { int v = data.substring(4).toInt(); minspeed = constrain(v, -255, 255); }
  else if (data.startsWith("Max:")) { int v = data.substring(4).toInt(); maxspeed = constrain(v, -255, 255); }

  else if (data.startsWith("MinB:")) { int v = data.substring(5).toInt(); basespeed = constrain(v, 0, 255); }
  else if (data.startsWith("MaxB:")) { int v = data.substring(5).toInt(); maxbasespeed = constrain(v, 0, 255); }

  else if (data.startsWith("Ca:")) { calibrate = !calibrate; }
  else if (data.startsWith("Sa:")) { activate_PID = data.substring(3).toInt(); }
  else if (data.startsWith("So:")) { activate_PID = data.substring(3).toInt(); }
}

