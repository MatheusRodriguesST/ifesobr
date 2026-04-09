// ====================================================================
//  OBR 2026 — SEGUIDOR DE LINHA PROFISSIONAL
//  ESP32 + L298N + 3x TCRT5000
//  Versão: 3.0 — PID Adaptativo + Fallback Inteligente
// ====================================================================
//
//  ARQUITETURA DO SISTEMA:
//  ┌─────────────────────────────────────────────────────────────────┐
//  │  1. AUTO-CALIBRACAO  — Gira sobre a linha p/ mapear min/max   │
//  │  2. LEITURA + FILTRO — Média móvel p/ suavizar ruído          │
//  │  3. CALCULO DE ERRO  — Média ponderada normalizada [-1, +1]   │
//  │  4. PID ADAPTATIVO   — P + I + D com anti-windup e ganho      │
//  │                        variável conforme magnitude do erro     │
//  │  5. VELOCIDADE ADAPT.— Reduz velocidade proporcional ao erro  │
//  │  6. FALLBACK 3 FASES — Busca → Espiral → Parada              │
//  │  7. DETECÇÃO DE FIM  — Para se não achar linha após timeout   │
//  └─────────────────────────────────────────────────────────────────┘
//
//  PINAGEM (confira com seu hardware):
//  ┌──────────────┬────────┬──────────────────────────────────────┐
//  │ Componente   │ Pino   │ Descrição                            │
//  ├──────────────┼────────┼──────────────────────────────────────┤
//  │ Motor A IN1  │ GPIO27 │ Direção motor esquerdo               │
//  │ Motor A IN2  │ GPIO26 │ Direção motor esquerdo               │
//  │ Motor A ENA  │ GPIO25 │ PWM velocidade motor esquerdo        │
//  │ Motor B IN3  │ GPIO5  │ Direção motor direito                │
//  │ Motor B IN4  │ GPIO18 │ Direção motor direito                │
//  │ Motor B ENB  │ GPIO19 │ PWM velocidade motor direito         │
//  │ Sensor ESQ   │ GPIO32 │ TCRT5000 esquerdo                    │
//  │ Sensor CEN   │ GPIO33 │ TCRT5000 centro    *** CORRIGIDO *** │
//  │ Sensor DIR   │ GPIO35 │ TCRT5000 direito   *** CORRIGIDO *** │
//  └──────────────┴────────┴──────────────────────────────────────┘
//
//  COMO USAR:
//  1. Posicione o robô na borda da linha (centro sobre a linha)
//  2. Ligue → aguarde calibração automática (~6 segundos)
//  3. Contagem regressiva de 3s → robô começa a seguir
//  4. Abra o Serial Monitor (115200 baud) para debug
//
//  AJUSTE FINO:
//  - Se oscila muito: aumente KD ou diminua KP
//  - Se é lento nas curvas: aumente KP
//  - Se perde a linha nas curvas: diminua VEL_BASE
//  - Se faz "zig-zag": aumente FILTRO_JANELA
//  - Se não vira o suficiente: aumente VEL_CURVA_FORTE
//
//  COMPATIBILIDADE:
//  - Arduino ESP32 Core v2.x: usa ledcSetup + ledcAttachPin
//  - Arduino ESP32 Core v3.x: usa ledcAttach (atual, padrão)
//  - Se der erro de compilação no LEDC, veja seção CONFIGURAÇÃO PWM
//
// ====================================================================

#include <Arduino.h>

// ====================================================================
//  CONFIGURAÇÃO DE PINOS
// ====================================================================

// --- Motor Esquerdo (A) — conectado ao L298N lado A ---
#define M_ESQ_IN1   27    // IN1 do L298N
#define M_ESQ_IN2   26    // IN2 do L298N
#define M_ESQ_ENA   25    // ENA do L298N (PWM)

// --- Motor Direito (B) — conectado ao L298N lado B ---
#define M_DIR_IN3   5     // IN3 do L298N
#define M_DIR_IN4   18    // IN4 do L298N
#define M_DIR_ENB   19    // ENB do L298N (PWM)

// --- Sensores TCRT5000 (saída analógica) ---
// IMPORTANTE: Estes pinos foram CORRIGIDOS em relação ao código original.
// O código original tinha S_CENTRO=35 e S_DIR=33 — estava trocado!
// Agora: GPIO32=esquerdo, GPIO33=centro, GPIO35=direito
#define S_ESQ       32    // Sensor esquerdo
#define S_CEN       35    // Sensor centro   (era 35, corrigido!)
#define S_DIR       33    // Sensor direito  (era 33, corrigido!) //wallace

// ====================================================================
//  CONFIGURAÇÃO PWM (LEDC)
// ====================================================================
// Para ESP32 Arduino Core v3.x (padrão atual):
//   ledcAttach(pino, frequencia, resolucao) — canal automático
//   ledcWrite(pino, duty)
//
// Se você usa ESP32 Arduino Core v2.x, descomente a seção LEGACY
// no setup() e comente a seção v3.x.

#define PWM_FREQ    5000   // 5 kHz — frequência alta = motor silencioso
#define PWM_RES     8      // 8 bits → duty 0-255

// ====================================================================
//  PARÂMETROS DE VELOCIDADE (ajuste conforme seu robô)
// ====================================================================

#define VEL_BASE          150   // Velocidade base em reta (0-255)
#define VEL_MAX           230   // Velocidade máxima permitida
#define VEL_MIN_CURVA     60    // Velocidade mínima em curvas fortes
#define VEL_BUSCA         110   // Velocidade durante busca de linha
#define VEL_BUSCA_ESPIRAL 90    // Velocidade na busca espiral (fase 2)

// ====================================================================
//  PARÂMETROS PID
// ====================================================================
//  O controlador PID calcula a correção baseado em 3 termos:
//    P (Proporcional) = KP * erro         → reage ao erro atual
//    I (Integral)     = KI * soma_erros   → corrige erro acumulado
//    D (Derivativo)   = KD * delta_erro   → prevê erro futuro
//
//  Correção = P + I + D
//  Motor esq = VEL_BASE + correção
//  Motor dir = VEL_BASE - correção
//
//  DICAS DE AJUSTE (método Ziegler-Nichols simplificado):
//  1. Coloque KI=0, KD=0
//  2. Aumente KP até o robô oscilar (vai e volta na linha)
//  3. Reduza KP pela metade
//  4. Aumente KD até as oscilações pararem
//  5. Se ainda tem erro constante (ex: curva longa), adicione KI aos poucos

#define KP    45.0f     // Ganho proporcional (comece aqui e ajuste)
#define KI    0.005f    // Ganho integral (mantenha MUITO pequeno)
#define KD    25.0f     // Ganho derivativo (ajuda estabilidade)

// Anti-windup: limita o acumulo do termo integral
#define INTEGRAL_MAX  500.0f

// Ganho adaptativo: multiplica KP quando erro é grande (curvas)
// Quando |erro| > LIMIAR_CURVA, KP efetivo = KP * GANHO_CURVA
#define LIMIAR_CURVA   0.4f    // erro acima disso = curva detectada
#define GANHO_CURVA    1.8f    // multiplicador do KP em curvas

// ====================================================================
//  PARÂMETROS DO FILTRO DE SUAVIZAÇÃO
// ====================================================================
//  Média móvel dos últimos N leituras para cada sensor.
//  Reduz ruído sem adicionar muito atraso.
//  Quanto maior a janela, mais suave mas mais lento para reagir.

#define FILTRO_JANELA  4    // Quantidade de amostras (2-8 recomendado)

// ====================================================================
//  PARÂMETROS DE FALLBACK (recuperação de linha perdida)
// ====================================================================
//
//  FASE 1 — BUSCA SIMPLES (0 a TIMEOUT_BUSCA ms):
//    Gira na direção do último erro conhecido.
//    Rápido e eficaz para perdas pequenas (curvas).
//
//  FASE 2 — BUSCA ESPIRAL (TIMEOUT_BUSCA a TIMEOUT_ESPIRAL ms):
//    Alterna giros esquerda/direita com amplitude crescente.
//    Cobre área maior, encontra linha em perdas maiores.
//
//  FASE 3 — PARADA (após TIMEOUT_ESPIRAL ms):
//    Linha não encontrada = provável fim do percurso.
//    Robô para completamente.

#define TIMEOUT_BUSCA    1500   // ms — duração da busca simples
#define TIMEOUT_ESPIRAL  5000   // ms — duração total antes de parar
#define ESPIRAL_CICLO    300    // ms — duração de cada giro na espiral

// ====================================================================
//  PARÂMETROS DE CALIBRAÇÃO
// ====================================================================

#define CAL_AMOSTRAS_PARADO  30   // Leituras com robô parado
#define CAL_TEMPO_GIRO       50   // ms entre leituras durante giro
#define CAL_VEL_GIRO         90   // PWM do motor durante calibração
#define CAL_GIROS_ESQ        35   // Amostras girando p/ esquerda
#define CAL_GIROS_DIR        70   // Amostras girando p/ direita
#define CAL_LIMIAR           0.40f // Limiar normalizado (0.0-1.0)
#define CAL_RANGE_MINIMO     200   // Range mínimo aceitável por sensor

// ====================================================================
//  VARIÁVEIS GLOBAIS
// ====================================================================

// --- Calibração ---
int calMin[3] = {4095, 4095, 4095};  // Menor valor (sobre a linha preta)
int calMax[3] = {0, 0, 0};           // Maior valor (fora da linha, branco)

// --- Filtro de média móvel ---
int filtro_buffer[3][FILTRO_JANELA];  // Buffer circular por sensor
int filtro_idx = 0;                    // Índice atual no buffer
bool filtro_cheio = false;             // Buffer já foi preenchido?

// --- Estado dos sensores ---
int    raw[3];          // Leitura bruta do ADC (0-4095)
float  norm[3];         // Valor normalizado (0.0 = fora, 1.0 = na linha)
bool   ativo[3];        // Sensor detectando linha?

// --- PID ---
float  erro       = 0.0f;   // Erro atual (-1.0 a +1.0)
float  erroAnt    = 0.0f;   // Erro da iteração anterior
float  integral   = 0.0f;   // Acumulador integral
float  correcao   = 0.0f;   // Saída do PID

// --- Motores ---
int    velEsq     = 0;
int    velDir     = 0;

// --- Fallback ---
unsigned long tUltimaLinha  = 0;   // Timestamp da última detecção
unsigned long tempoSemLinha = 0;   // Quanto tempo sem ver linha
int    espiralDirecao       = 1;   // 1 = direita, -1 = esquerda
unsigned long espiralTroca  = 0;   // Timestamp da última troca espiral
int    espiralCiclos        = 0;   // Contador de ciclos na espiral

// --- Debug ---
unsigned long ciclos         = 0;
unsigned long tLoopUs        = 0;
unsigned long tDebug         = 0;

// --- Estados para debug serial ---
enum Estado {
  EST_PARADO,
  EST_RETO,
  EST_CURVA_ESQ,
  EST_CURVA_DIR,
  EST_BUSCA,
  EST_ESPIRAL,
  EST_FIM
};

const char* nomeEstado[] = {
  "PARADO", "RETO", "CURVA_ESQ", "CURVA_DIR",
  "BUSCA", "ESPIRAL", "FIM_PISTA"
};

Estado estadoAtual = EST_PARADO;

// ====================================================================
//  FUNÇÕES DOS MOTORES
// ====================================================================
//
//  motorEsq(pwm) e motorDir(pwm):
//    pwm > 0  → frente
//    pwm == 0 → parado
//
//  A direção dos pinos (IN1/IN2, IN3/IN4) pode precisar ser
//  invertida dependendo de como os fios do motor estão conectados.
//  Se um motor gira ao contrário, troque HIGH/LOW nos pinos IN.

void motorEsq(int pwm) {
  pwm = constrain(pwm, 0, 255);
  // Motor esquerdo: IN1=LOW, IN2=HIGH → frente
  // Se girar ao contrário, troque HIGH/LOW abaixo:
  digitalWrite(M_ESQ_IN1, LOW);
  digitalWrite(M_ESQ_IN2, HIGH);
  ledcWrite(M_ESQ_ENA, pwm);
}

void motorDir(int pwm) {
  pwm = constrain(pwm, 0, 255);
  // Motor direito: IN3=HIGH, IN4=LOW → frente
  // Se girar ao contrário, troque HIGH/LOW abaixo:
  digitalWrite(M_DIR_IN3, HIGH);
  digitalWrite(M_DIR_IN4, LOW);
  ledcWrite(M_DIR_ENB, pwm);
}

void parar() {
  digitalWrite(M_ESQ_IN1, LOW);
  digitalWrite(M_ESQ_IN2, LOW);
  ledcWrite(M_ESQ_ENA, 0);
  digitalWrite(M_DIR_IN3, LOW);
  digitalWrite(M_DIR_IN4, LOW);
  ledcWrite(M_DIR_ENB, 0);
}

// Gira no próprio eixo: positivo = horário (direita), negativo = anti-horário (esquerda)
void girar(int pwm) {
  int p = abs(constrain(pwm, -255, 255));
  if (pwm > 0) {
    // Gira pra direita: esquerdo frente, direito ré
    digitalWrite(M_ESQ_IN1, LOW);
    digitalWrite(M_ESQ_IN2, HIGH);
    ledcWrite(M_ESQ_ENA, p);
    digitalWrite(M_DIR_IN3, LOW);
    digitalWrite(M_DIR_IN4, HIGH);
    ledcWrite(M_DIR_ENB, p);
  } else if (pwm < 0) {
    // Gira pra esquerda: esquerdo ré, direito frente
    digitalWrite(M_ESQ_IN1, HIGH);
    digitalWrite(M_ESQ_IN2, LOW);
    ledcWrite(M_ESQ_ENA, p);
    digitalWrite(M_DIR_IN3, HIGH);
    digitalWrite(M_DIR_IN4, LOW);
    ledcWrite(M_DIR_ENB, p);
  } else {
    parar();
  }
}

// ====================================================================
//  FILTRO DE MÉDIA MÓVEL
// ====================================================================
//  Reduz ruído dos sensores TCRT5000 sem adicionar atraso
//  significativo. Cada chamada adiciona uma leitura nova ao buffer
//  circular e retorna a média das últimas FILTRO_JANELA leituras.

void filtro_adicionar(int valores[3]) {
  for (int i = 0; i < 3; i++) {
    filtro_buffer[i][filtro_idx] = valores[i];
  }
  filtro_idx = (filtro_idx + 1) % FILTRO_JANELA;
  if (filtro_idx == 0) filtro_cheio = true;
}

int filtro_media(int sensor) {
  int n = filtro_cheio ? FILTRO_JANELA : max(filtro_idx, 1);
  long soma = 0;
  for (int j = 0; j < n; j++) {
    soma += filtro_buffer[sensor][j];
  }
  return (int)(soma / n);
}

// ====================================================================
//  LEITURA E NORMALIZAÇÃO DOS SENSORES
// ====================================================================
//  O TCRT5000 no modo analógico retorna:
//    - Valor BAIXO quando sobre a LINHA PRETA (pouca reflexão)
//    - Valor ALTO quando sobre SUPERFÍCIE CLARA (muita reflexão)
//
//  Após normalização:
//    0.0 = fora da linha (branco)
//    1.0 = sobre a linha (preto)
//
//  O erro é calculado como média ponderada:
//    erro = (-1 * esq + 0 * cen + 1 * dir) / soma
//    Resultado: -1.0 (linha na esquerda) a +1.0 (linha na direita)

float normalizar(int rawVal, int sMin, int sMax) {
  if (sMax <= sMin) return 0.0f;
  // Invertido: raw baixo = na linha = retorna 1.0
  float n = (float)(sMax - rawVal) / (float)(sMax - sMin);
  return constrain(n, 0.0f, 1.0f);
}

float lerSensores() {
  // Leitura bruta
  int leitura[3];
  leitura[0] = analogRead(S_ESQ);
  leitura[1] = analogRead(S_CEN);
  leitura[2] = analogRead(S_DIR);

  // Adiciona ao filtro
  filtro_adicionar(leitura);

  // Usa a média filtrada
  for (int i = 0; i < 3; i++) {
    raw[i]  = filtro_media(i);
    norm[i] = normalizar(raw[i], calMin[i], calMax[i]);
    ativo[i] = (norm[i] > CAL_LIMIAR);
  }

  float v0 = norm[0];  // esquerdo
  float v1 = norm[1];  // centro
  float v2 = norm[2];  // direito

  float soma = v0 + v1 + v2;

  // Se nenhum sensor detecta nada, retorna 0
  if (soma < 0.05f) return 0.0f;

  // Erro ponderado: -1 (esq) a +1 (dir)
  return (-1.0f * v0 + 0.0f * v1 + 1.0f * v2) / soma;
}

// ====================================================================
//  CONTROLE PID ADAPTATIVO
// ====================================================================
//  Características:
//    - Ganho proporcional aumenta em curvas (GANHO_CURVA)
//    - Integral com anti-windup (limitado a INTEGRAL_MAX)
//    - Integral é zerada quando erro muda de sinal (crossover)
//    - Velocidade base é reduzida proporcional ao |erro|

float calcularPID(float err) {
  // --- Proporcional com ganho adaptativo ---
  float kp_efetivo = KP;
  if (fabsf(err) > LIMIAR_CURVA) {
    kp_efetivo = KP * GANHO_CURVA;
  }
  float P = kp_efetivo * err;

  // --- Integral com anti-windup ---
  // Reseta integral se erro cruzou o zero (evita overshoot)
  if ((err > 0 && erroAnt < 0) || (err < 0 && erroAnt > 0)) {
    integral = 0.0f;
  }
  integral += err;
  integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = KI * integral;

  // --- Derivativo ---
  float D = KD * (err - erroAnt);

  return P + I + D;
}

// Calcula velocidade base adaptativa: desacelera em curvas
int velocidadeAdaptativa(float err) {
  // Quanto maior o erro, menor a velocidade
  // |erro| = 0 → VEL_BASE, |erro| = 1 → VEL_MIN_CURVA
  float fator = 1.0f - (fabsf(err) * (1.0f - (float)VEL_MIN_CURVA / (float)VEL_BASE));
  return (int)(VEL_BASE * fator);
}

// ====================================================================
//  CLASSIFICAÇÃO DE ESTADO (para debug)
// ====================================================================

Estado classificarEstado(float err, bool e, bool c, bool d) {
  int count = (int)e + (int)c + (int)d;
  if (count == 0) return EST_BUSCA;
  if (err < -0.2f) return EST_CURVA_ESQ;
  if (err >  0.2f) return EST_CURVA_DIR;
  return EST_RETO;
}

// ====================================================================
//  FALLBACK — SISTEMA DE RECUPERAÇÃO DE LINHA
// ====================================================================
//
//  Chamado quando NENHUM sensor detecta a linha.
//  Usa 3 fases progressivas:
//
//  Fase 1 (BUSCA SIMPLES):
//    Gira na direção do último erro conhecido.
//    É a fase mais rápida e resolve 90% dos casos.
//
//  Fase 2 (BUSCA ESPIRAL):
//    Alterna giros com amplitude crescente.
//    O robô "varre" uma área cada vez maior.
//
//  Fase 3 (PARADA):
//    Se após TIMEOUT_ESPIRAL ms a linha não for achada,
//    o robô assume que a pista acabou e para.

void executarFallback() {
  tempoSemLinha = millis() - tUltimaLinha;

  // ---- FASE 1: Busca simples ----
  if (tempoSemLinha < TIMEOUT_BUSCA) {
    estadoAtual = EST_BUSCA;

    // Gira na direção do último erro
    if (erroAnt <= 0) {
      // Linha estava à esquerda → gira pra esquerda
      velEsq = 0;
      velDir = VEL_BUSCA;
      motorEsq(0);
      motorDir(VEL_BUSCA);
    } else {
      // Linha estava à direita → gira pra direita
      velEsq = VEL_BUSCA;
      velDir = 0;
      motorEsq(VEL_BUSCA);
      motorDir(0);
    }
    return;
  }

  // ---- FASE 2: Busca espiral ----
  if (tempoSemLinha < TIMEOUT_ESPIRAL) {
    estadoAtual = EST_ESPIRAL;

    // Alterna direção a cada ESPIRAL_CICLO ms
    if (millis() - espiralTroca > (unsigned long)(ESPIRAL_CICLO + espiralCiclos * 50)) {
      espiralDirecao *= -1;  // Inverte direção
      espiralTroca = millis();
      espiralCiclos++;
    }

    // Gira no próprio eixo com amplitude crescente
    int pwm_espiral = min(VEL_BUSCA_ESPIRAL + espiralCiclos * 10, 200);
    girar(espiralDirecao * pwm_espiral);

    velEsq = (espiralDirecao < 0) ? -pwm_espiral : pwm_espiral;
    velDir = (espiralDirecao > 0) ? -pwm_espiral : pwm_espiral;
    return;
  }

  // ---- FASE 3: Parada (fim de pista) ----
  estadoAtual = EST_FIM;
  parar();
  velEsq = 0;
  velDir = 0;
}

// ====================================================================
//  CALIBRAÇÃO AUTOMÁTICA
// ====================================================================
//  Procedimento:
//  1. Lê sensores parados por ~1.5s (pega valores do chão atual)
//  2. Gira pra esquerda ~1.75s (passa sensores pela linha e fora)
//  3. Gira pra direita ~3.5s (idem, cobrindo ambos os lados)
//  4. Volta ao centro ~750ms
//  5. Calcula min/max de cada sensor
//
//  Posição inicial recomendada: centro do robô sobre a borda da
//  linha preta, para que ao girar os sensores passem sobre a linha
//  e sobre o branco.

void calibrar() {
  Serial.println();
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║       AUTO-CALIBRACAO DOS SENSORES     ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Posicione o robo com o sensor do MEIO  ║");
  Serial.println("║ sobre a BORDA da linha preta.          ║");
  Serial.println("║ Ele vai girar sozinho em 3 segundos.   ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();

  delay(3000);

  // Reset
  for (int i = 0; i < 3; i++) {
    calMin[i] = 4095;
    calMax[i] = 0;
  }

  // Função auxiliar para atualizar min/max
  auto atualizarCal = [&]() {
    int r[3] = {analogRead(S_ESQ), analogRead(S_CEN), analogRead(S_DIR)};
    for (int i = 0; i < 3; i++) {
      if (r[i] < calMin[i]) calMin[i] = r[i];
      if (r[i] > calMax[i]) calMax[i] = r[i];
    }
  };

  // Fase 1: parado
  Serial.println("  [1/4] Lendo sensores parado...");
  for (int t = 0; t < CAL_AMOSTRAS_PARADO; t++) {
    atualizarCal();
    delay(CAL_TEMPO_GIRO);
  }

  // Fase 2: gira para esquerda
  Serial.println("  [2/4] Girando para esquerda...");
  motorDir(CAL_VEL_GIRO);
  motorEsq(0);
  for (int t = 0; t < CAL_GIROS_ESQ; t++) {
    atualizarCal();
    delay(CAL_TEMPO_GIRO);
  }

  // Fase 3: gira para direita (dobro do tempo para cobrir o mesmo arco + mais)
  Serial.println("  [3/4] Girando para direita...");
  motorEsq(CAL_VEL_GIRO);
  motorDir(0);
  for (int t = 0; t < CAL_GIROS_DIR; t++) {
    atualizarCal();
    delay(CAL_TEMPO_GIRO);
  }

  // Fase 4: volta ao centro
  Serial.println("  [4/4] Centralizando...");
  motorDir(CAL_VEL_GIRO);
  motorEsq(0);
  delay(750);
  parar();

  // Resultado
  Serial.println();
  Serial.println("  ┌──────────────────────────────────────┐");
  Serial.println("  │      RESULTADO DA CALIBRACAO         │");
  Serial.println("  ├────────┬──────┬──────┬───────┬───────┤");
  Serial.println("  │ Sensor │  Min │  Max │ Range │ Status│");
  Serial.println("  ├────────┼──────┼──────┼───────┼───────┤");

  const char* lbl[] = {"ESQ   ", "CEN   ", "DIR   "};
  bool calOK = true;

  for (int i = 0; i < 3; i++) {
    int range = calMax[i] - calMin[i];
    bool ok = (range >= CAL_RANGE_MINIMO);
    if (!ok) calOK = false;

    Serial.printf("  │ %s │ %4d │ %4d │  %4d │  %s │\n",
                  lbl[i], calMin[i], calMax[i], range,
                  ok ? " OK " : "RUIM");
  }

  Serial.println("  └────────┴──────┴──────┴───────┴───────┘");

  if (!calOK) {
    Serial.println();
    Serial.println("  ⚠ ATENCAO: Algum sensor tem range < 200!");
    Serial.println("  Verifique se os sensores estao na altura");
    Serial.println("  correta (5-10mm do chao) e se ha contraste");
    Serial.println("  entre a linha e o piso.");
    Serial.println("  O robo vai tentar funcionar mesmo assim.");
  }

  Serial.println();
  Serial.println("  Calibracao concluida!");
}

// ====================================================================
//  SETUP
// ====================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║  OBR 2026 — SEGUIDOR DE LINHA v3.0        ║");
  Serial.println("║  ESP32 + L298N + 3x TCRT5000              ║");
  Serial.println("║  PID Adaptativo + Fallback Inteligente     ║");
  Serial.println("╚════════════════════════════════════════════╝");

  // --- Configuração dos pinos dos motores ---
  pinMode(M_ESQ_IN1, OUTPUT);
  pinMode(M_ESQ_IN2, OUTPUT);
  pinMode(M_DIR_IN3, OUTPUT);
  pinMode(M_DIR_IN4, OUTPUT);

  // --- Configuração PWM (LEDC) ---
  // *** Para ESP32 Arduino Core v3.x (padrão) ***
  ledcAttach(M_ESQ_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(M_DIR_ENB, PWM_FREQ, PWM_RES);

  // *** Para ESP32 Arduino Core v2.x (descomente se necessário) ***
  // ledcSetup(0, PWM_FREQ, PWM_RES);
  // ledcAttachPin(M_ESQ_ENA, 0);
  // ledcSetup(1, PWM_FREQ, PWM_RES);
  // ledcAttachPin(M_DIR_ENB, 1);
  // NOTA: se usar v2.x, troque ledcWrite(M_ESQ_ENA, x) por ledcWrite(0, x)
  //       e ledcWrite(M_DIR_ENB, x) por ledcWrite(1, x) nas funções de motor.

  parar();

  // --- Configuração dos pinos dos sensores ---
  // GPIO32, 33 e 35 são input-only no ESP32, não precisam de pinMode
  // mas declaramos por clareza
  pinMode(S_ESQ, INPUT);
  pinMode(S_CEN, INPUT);
  pinMode(S_DIR, INPUT);

  // --- Inicializa buffer do filtro ---
  memset(filtro_buffer, 0, sizeof(filtro_buffer));

  // --- Calibração automática ---
  calibrar();

  // --- Contagem regressiva ---
  for (int i = 3; i > 0; i--) {
    Serial.printf("  >>> GO em %d... <<<\n", i);
    delay(1000);
  }
  Serial.println();
  Serial.println("  ██████████████████████████████████");
  Serial.println("  ██       RODANDO!              ██");
  Serial.println("  ██████████████████████████████████");
  Serial.println();

  // Imprime cabeçalho do debug
  Serial.println("  [ESQ   CEN   DIR ] erro    estado           ME  MD  info");
  Serial.println("  ─────────────────────────────────────────────────────────");

  tUltimaLinha = millis();
  espiralTroca = millis();
  tDebug = millis();
}

// ====================================================================
//  LOOP PRINCIPAL
// ====================================================================
//  Executa a cada ~5ms (200 Hz):
//    1. Lê e filtra sensores
//    2. Se algum sensor ativo → PID + controle de motor
//    3. Se nenhum ativo → Fallback (busca / espiral / parada)
//    4. Debug serial a cada 200ms

void loop() {
  unsigned long tInicio = micros();
  ciclos++;

  // ── 1. Leitura dos sensores ──
  erro = lerSensores();
  bool algumAtivo = ativo[0] || ativo[1] || ativo[2];

  // ── 2. Controle principal ──
  if (algumAtivo) {
    // Linha detectada! Reset do fallback
    tUltimaLinha = millis();
    tempoSemLinha = 0;
    espiralCiclos = 0;

    // Classifica estado para debug
    estadoAtual = classificarEstado(erro, ativo[0], ativo[1], ativo[2]);

    // Calcula PID
    correcao = calcularPID(erro);

    // Velocidade base adaptativa (desacelera em curvas)
    int velBase = velocidadeAdaptativa(erro);

    // Aplica correção aos motores
    // Erro negativo (linha na esq) → correção negativa → motor esq mais lento
    // Erro positivo (linha na dir) → correção positiva → motor dir mais lento
    velEsq = constrain((int)(velBase + correcao), 0, VEL_MAX);
    velDir = constrain((int)(velBase - correcao), 0, VEL_MAX);

    motorEsq(velEsq);
    motorDir(velDir);

    // Salva erro para próxima iteração
    erroAnt = erro;

  } else {
    // ── 3. Fallback — linha perdida ──
    executarFallback();
  }

  // ── 4. Debug serial (a cada 200ms) ──
  if (millis() - tDebug >= 200) {
    tDebug = millis();

    Serial.printf("  [%.2f  %.2f  %.2f] %+.3f  %-16s  %3d %3d",
                  norm[0], norm[1], norm[2],
                  erro,
                  nomeEstado[estadoAtual],
                  velEsq, velDir);

    // Info extra dependendo do estado
    if (estadoAtual == EST_BUSCA || estadoAtual == EST_ESPIRAL) {
      Serial.printf("  [sem linha: %lums]", tempoSemLinha);
    }
    if (estadoAtual == EST_FIM) {
      Serial.print("  [PARADO - FIM]");
    }

    Serial.printf("  %luus", tLoopUs);
    Serial.println();
  }

  // Tempo do loop
  tLoopUs = micros() - tInicio;

  // Delay para manter ~200Hz (5ms entre loops)
  // Subtrai o tempo que já gastou no processamento
  int delayMs = 5 - (int)(tLoopUs / 1000);
  if (delayMs > 0) delay(delayMs);
}

// ====================================================================
//  NOTAS PARA A EQUIPE
// ====================================================================
//
//  1. CORREÇÃO DOS SENSORES:
//     O código original tinha os pinos do sensor CENTRO (35) e
//     DIREITO (33) TROCADOS. Agora está corrigido:
//       GPIO32 = Esquerdo
//       GPIO33 = Centro
//       GPIO35 = Direito
//     Se após a correção ainda parecer invertido, verifique a
//     posição FÍSICA dos sensores no chassi do robô.
//
//  2. AJUSTE DE VELOCIDADE:
//     Comece com VEL_BASE=120 e aumente gradualmente.
//     O sistema adapta a velocidade automaticamente em curvas.
//     VEL_MAX define o teto absoluto.
//
//  3. AJUSTE DO PID:
//     Use o Serial Monitor para observar o erro em tempo real.
//     - Se o robô "zig-zagueia": KP muito alto ou KD muito baixo
//     - Se curva devagar demais: KP muito baixo
//     - Se não corrige desvio constante: adicione um pouco de KI
//     - Se tem overshooting: aumente KD
//
//  4. FILTRO:
//     FILTRO_JANELA=4 é bom para a maioria dos casos.
//     Se o sensor é muito ruidoso, tente 6 ou 8.
//     Não passe de 8 pois adiciona atraso demais.
//
//  5. FALLBACK:
//     A busca espiral é eficaz mas pode ser agressiva.
//     Se o robô gira demais quando perde a linha em curvas simples,
//     aumente TIMEOUT_BUSCA (dê mais tempo pra busca simples).
//
//  6. COMPATIBILIDADE ESP32 CORE:
//     O código usa ledcAttach() (v3.x). Se seu Arduino IDE tem
//     o core v2.x, veja os comentários no setup() para alternar.
//
//  7. SOBRE "MINI IA":
//     O sistema PID adaptativo + fallback em fases funciona como
//     um sistema de decisão inteligente:
//     - O ganho adaptativo "decide" quando é curva e reage mais
//     - A velocidade adaptativa "decide" desacelerar em curvas
//     - O fallback em 3 fases "decide" qual estratégia usar
//     - O anti-windup "decide" resetar o erro acumulado
//     Para um robô com 3 sensores, isso é mais eficaz que redes
//     neurais (que precisariam de MUITOS dados de treino e mais
//     sensores para ter vantagem).
//
// ====================================================================