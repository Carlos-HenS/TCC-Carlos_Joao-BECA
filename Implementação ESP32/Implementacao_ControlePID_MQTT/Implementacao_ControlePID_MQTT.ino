/*
  ============================================================================
  PROJETO DE TCC – SISTEMA DE CONTROLE DE VELOCIDADE PARA MOTOR DC
  ============================================================================
  Autores: Carlos Henrique Sibaldo Torres de Lira
           João Paulo Sibaldo Torres de Lira
  Curso: Bacharelado em Engenharia de Controle e Automação
  Descrição: Este código implementa um sistema embarcado baseado na ESP32 
             para controle de velocidade de um motor DC utilizando um 
             controlador PID com anti-windup e filtro derivativo. O sistema 
             também realiza a aquisição de dados via encoder em quadratura 
             e comunicação com um supervisório via protocolo MQTT.
*/

#include <WiFi.h>
#include <PubSubClient.h>

// ========== CONFIGURAÇÕES DO ENCODER ========== 
#define ENCODER_A 19
#define ENCODER_B 18
#define PPR 11                           // Pulsos por rotação do encoder
#define GEAR_RATIO 45.0                  // Relação de redução do motorredutor
#define PPR_TOTAL (PPR * GEAR_RATIO * 4) // Total de pulsos considerando quadratura completa

volatile long contagemPulsos = 0;
volatile bool ultimoEstadoA = 0;
volatile bool ultimoEstadoB = 0;
unsigned long tempoAnterior = 0;
float rpm_motor = 0;
float rpm_saida = 0;

// Tempo acumulado (em centésimos de segundo)
unsigned long tempoAcumulado = 0;        // Tempo acumulado de operação

// ========== CONFIGURAÇÕES DO WIFI ========== 
const char *ssid = "EDIJANIO_VLINK FBIRA";
const char *senha = "jano@7564";

// ========== CONFIGURAÇÕES DE MQTT ========== 
const char *broker_mqtt = "192.168.0.104";
const int porta_mqtt = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

// ========== CONFIGURAÇÕES DO MOTOR ========== 
#define IN1 22
#define IN2 23
#define PWM_PIN 32
int duty_pwm = 0;

// ========== CONTROLE PID COM FILTRO DERIVATIVO E ANTI-WINDUP ========== 
float Kp = 0.467;         // Ganho proporcional
float Ki = 0.467/0.0627;    // Ganho integral
float Kd = 0.467*0.00948;   // Ganho derivativo
float alpha = 0.125;        // Coeficiente do filtro derivativo
float Td = 0.00948;         // Constante de tempo derivativa
float Tt = 0.0244;           // Constante de tempo para anti-windup
float T = 0.05;           // Período de amostragem (50 ms)
float setpoint = 0.0;     // Inicializa o setpoint

// Variáveis de estado do controlador PID
float erro_anterior = 0.0;
float I_anterior = 0.0;   // Termo integral anterior
float d_anterior = 0.0;   // Termo derivativo filtrado anterior
float es = 0.0;  // Erro de saturação 
float es_anterior = 0.0;  // Erro de saturação anterior
float saida_pwm = 0.0;

// Coeficientes pré-calculados para o termo derivativo
float A = 0.0;
float B = 0.0;

// Limites de saturação
float u_min = 0.0;
float u_max = 255.0;

// ========== BOTÕES FÍSICOS DE CONTROLE ========== 
#define BOTAO_LIGAR 4
#define BOTAO_DESLIGAR 21

bool estadoMQTT = false;
bool estadoFisico = false;
bool motorLigado = false;

// ========================== IMPORTANTE ==========================
// Os botões físicos de ligar e desligar foram incluídos no código
// por questões de robustez e possíveis expansões do projeto,
// porém, nesta aplicação prática, o motor é controlado **exclusivamente**
// via supervisórios (local - Node-RED / remoto - Firebase).
// A estrutura dos botões permanece como base para trabalhos futuros.
// ================================================================

// Variáveis para debounce dos botões físicos
unsigned long debounceDelay = 50;
unsigned long ultimoTempoLigar = 0;
unsigned long ultimoTempoDesligar = 0;
int estadoAnteriorLigar = HIGH;
int estadoAnteriorDesligar = HIGH;


// ========== FUNÇÃO DE INICIALIZAÇÃO DO MOTOR ========== 
void iniciar_motor() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  bool sucesso = ledcAttach(PWM_PIN, 1000, 8);
  if (!sucesso) {
    Serial.println("Erro ao configurar PWM!");
    while (true);
  }

  ledcWrite(PWM_PIN, 0);
}

// ========== FUNÇÃO PARA INICIAR OS BOTÕES ========== 
void iniciar_botoes() {
  pinMode(BOTAO_LIGAR, INPUT_PULLUP);
  pinMode(BOTAO_DESLIGAR, INPUT_PULLUP);
}

// ========== INTERRUPÇÕES DO ENCODER (QUADRATURA OTIMIZADA) ========== 
void IRAM_ATTR lerEncoder() {
  // Leitura rápida dos estados atuais
  bool estadoA = digitalRead(ENCODER_A);
  bool estadoB = digitalRead(ENCODER_B);
  
  // Tabela de estados para quadratura (mais eficiente)
  // Combinação dos estados: (ultimoA << 1) | ultimoB para estado anterior
  // (estadoA << 1) | estadoB para estado atual
  int estadoAnterior = (ultimoEstadoA << 1) | ultimoEstadoB;
  int estadoAtual = (estadoA << 1) | estadoB;
  
  // Tabela de lookup para incremento/decremento baseada na transição
  // Sequência horária: 00 -> 01 -> 11 -> 10 -> 00
  // Sequência anti-horária: 00 -> 10 -> 11 -> 01 -> 00
  static const int8_t tabelaQuadratura[16] = {
    0,  1, -1,  0,   // Estado anterior: 00
   -1,  0,  0,  1,   // Estado anterior: 01  
    1,  0,  0, -1,   // Estado anterior: 10
    0, -1,  1,  0    // Estado anterior: 11
  };
  
  // Incrementa/decrementa baseado na transição
  contagemPulsos += tabelaQuadratura[estadoAnterior * 4 + estadoAtual];
  
  // Atualiza estados anteriores
  ultimoEstadoA = estadoA;
  ultimoEstadoB = estadoB;
}

// ========== INICIALIZAÇÃO DA SERIAL E ENCODER ========== 
void iniciar_serial() {
  Serial.begin(115200);
  
  // Configuração dos pinos do encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Leitura inicial dos estados
  ultimoEstadoA = digitalRead(ENCODER_A);
  ultimoEstadoB = digitalRead(ENCODER_B);
  
  // Configuração das interrupções para ambos os canais (qualquer mudança)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), lerEncoder, CHANGE);
}

// ========== INICIALIZAÇÃO DO CONTROLADOR PID ==========
void iniciar_pid() {
  // Pré-cálculo dos coeficientes para o termo derivativo
  A = Kd / (alpha * Td + T/2);
  B = (T/2 - alpha * Td) / (alpha * Td + T/2);
  
  // Inicializa variáveis de estado do PID
  erro_anterior = 0.0;
  I_anterior = 0.0;
  d_anterior = 0.0;
  es = 0.0;
  es_anterior = 0.0;
}

// ========== INICIALIZAÇÃO DO WIFI ==========
void iniciar_wifi() {
  WiFi.begin(ssid, senha);
  Serial.println("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConectado!");
  Serial.print("IP da ESP32: ");
  Serial.println(WiFi.localIP());
}

// ========== INICIALIZAÇÃO DO CLIENTE MQTT ==========
void iniciar_mqtt() {
  client.setServer(broker_mqtt, porta_mqtt);  // Configura o broker MQTT
  client.setCallback(callback_mqtt);          // Define a função de callback para recebimento de mensagens
  while (!client.connected()) {               // Conecta ao broker MQTT
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    Serial.print("Conectando ao broker MQTT... ");
    if (client.connect(client_id.c_str())) {
      Serial.println("Conectado ao broker!");
      client.subscribe("IFPBCajazeiras/usuario01/acionamento");  // Assina o tópico "IFPBCajazeiras/usuario01/acionamento" para controle do motor
      client.subscribe("IFPBCajazeiras/usuario01/setpoint");  // Assina o novo tópico para IFPBCajazeiras/usuario01/setpoint
    } else {
      Serial.print("Falha, código: ");
      Serial.println(client.state());
      delay(2000);  // Espera 2 segundos antes de tentar novamente
    }
  }
}

// ========== FUNÇÃO DE CALLBACK PARA MENSAGENS MQTT ========== 
void callback_mqtt(char *topico, byte *mensagem, unsigned int comprimento) {
  String conteudo = "";
  for (int i = 0; i < comprimento; i++) {
    conteudo += (char)mensagem[i];  // Converte a mensagem recebida para string
  }

  if (String(topico) == "IFPBCajazeiras/usuario01/acionamento") {
    // Controle de acionamento do motor via MQTT
    if (conteudo == "true") {
      estadoMQTT = true;  // Ativa o controle via MQTT
      Serial.println("Node-RED: LIGAR");
    } else if (conteudo == "false") {
      estadoMQTT = false;  // Desativa o controle via MQTT
      Serial.println("Node-RED: DESLIGAR");
    }
  } else if (String(topico) == "IFPBCajazeiras/usuario01/setpoint") {
    // Atualiza o setpoint com a mensagem recebida via MQTT
    setpoint = conteudo.toFloat();  // Converte a string recebida para um valor float
    Serial.print("Novo setpoint recebido: ");
    Serial.println(setpoint);  // Exibe o novo setpoint no Serial Monitor
  }
}

// ========== VERIFICAÇÃO DOS BOTÕES FÍSICOS COM DEBOUNCE ========== 
void verificar_botoes() {
  int leituraLigar = digitalRead(BOTAO_LIGAR);
  int leituraDesligar = digitalRead(BOTAO_DESLIGAR);

  if (estadoAnteriorLigar == HIGH && leituraLigar == LOW && millis() - ultimoTempoLigar > debounceDelay) {
    estadoFisico = true;
    Serial.println("Botão físico LIGAR pressionado");
    ultimoTempoLigar = millis();
  }

  if (estadoAnteriorDesligar == HIGH && leituraDesligar == LOW && millis() - ultimoTempoDesligar > debounceDelay) {
    estadoFisico = false;
    Serial.println("Botão físico DESLIGAR pressionado");
    ultimoTempoDesligar = millis();
  }

  estadoAnteriorLigar = leituraLigar;
  estadoAnteriorDesligar = leituraDesligar;
}

// ========== ATUALIZAÇÃO DO PWM COM PID, ANTI-WINDUP E FILTRO DERIVATIVO ========== 
void atualizar_pwm_pid() {
  // Erro atual
  float erro = setpoint - rpm_saida;
  
  // 1. Termo proporcional
  float P = Kp * erro;
  
  // 2. Termo integral com anti-windup
  float I = I_anterior + (T/2)* Ki * (erro + erro_anterior) + (T / (2*Tt)) * (es + es_anterior);
  
  // 3. Termo derivativo com filtro
  float d = A * (erro - erro_anterior) - B * d_anterior;
  
  // 4. Soma dos termos para obter a saída do controlador
  float u = P + I + d;
  
  // 5. Implementação da saturação
  float u_sat = u;
  if (u > u_max) {
    u_sat = u_max;
  } else if (u < u_min) {
    u_sat = u_min;
  }
  
  // 6. Cálculo do erro de saturação para anti-windup
  es_anterior = es;
  es = u_sat - u;
  
  // 7. Atualização das variáveis de estado para a próxima iteração
  erro_anterior = erro;
  I_anterior = I;
  d_anterior = d;
  
  // Envia erro para visualização remota
  String mensagemErro = String(erro);
  char msgErroChar[20];
  mensagemErro.toCharArray(msgErroChar, 20);
  client.publish("IFPBCajazeiras/usuario01/erro", msgErroChar);

  // Aplica a saída saturada no PWM
  saida_pwm = u_sat;
  ledcWrite(PWM_PIN, (int)saida_pwm);
  
  // Depuração serial
  Serial.print("SP:");
  Serial.print(setpoint);
  Serial.print(" RPM:");
  Serial.print(rpm_saida);
  Serial.print(" Erro:");
  Serial.print(erro);
  Serial.print(" P:");
  Serial.print(P);
  Serial.print(" I:");
  Serial.print(I);
  Serial.print(" D:");
  Serial.print(d);
  Serial.print(" PWM:");
  Serial.println((int)saida_pwm);
}

// ========== CÁLCULO E PUBLICAÇÃO DA VELOCIDADE EM RPM ==========
void atualizar_rpm() {
  unsigned long tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= 50) {
    tempoAnterior = tempoAtual;

    float fator_calibracao = 1.0;
    // Cálculo de RPM com quadratura completa (4x resolução)
    float rpm_calc = (abs(contagemPulsos) / (float)(PPR * 4)) * 1200.0 * fator_calibracao;
    contagemPulsos = 0;

    if (motorLigado) {
      rpm_motor = rpm_calc;
      rpm_saida = rpm_motor / GEAR_RATIO;
      tempoAcumulado += 5;
    } else {
      rpm_motor = 0;
      rpm_saida = 0;
      tempoAcumulado = 0;
    }

    // Publica MQTT
    String mensagem = String(rpm_saida);
    char msgChar[20];
    mensagem.toCharArray(msgChar, 20);
    client.publish("IFPBCajazeiras/usuario01/velocidade", msgChar);

    String tempoMsg = String(tempoAcumulado / 100.0, 2);
    char tempoChar[20];
    tempoMsg.toCharArray(tempoChar, 20);
    client.publish("IFPBCajazeiras/usuario01/Time", tempoChar);

    // Aplica controle PID
    if (motorLigado) {
      atualizar_pwm_pid();
    }
  }
}

// ========== FUNÇÃO DE CONFIGURAÇÃO INICIAL ========== 
void setup() {
  iniciar_serial();
  iniciar_wifi();
  iniciar_mqtt();
  iniciar_motor();
  iniciar_botoes();
  iniciar_pid();  // Inicializa os parâmetros do PID
}

// ========== LOOP PRINCIPAL DO SISTEMA ========== 
void loop() {
  client.loop();
  verificar_botoes();

  motorLigado = estadoMQTT || estadoFisico;

  if (!motorLigado) {
    ledcWrite(PWM_PIN, 0);
    
    // Resetar as variáveis do controlador PID quando o motor estiver desligado
    erro_anterior = 0.0;
    I_anterior = 0.0;
    d_anterior = 0.0;
    es = 0.0;
    es_anterior = 0.0;
  }

  atualizar_rpm();
}