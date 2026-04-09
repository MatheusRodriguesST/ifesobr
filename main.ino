// ============================================================
//  SEGUIDOR DE LINHA ULTRA ESTÁVEL — OBR 2026
//  VERSÃO ANTI-CAMBALEIO
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

const char* WIFI_SSID = "OBR-Robo";
const char* WIFI_PASS = "12345678";
WebServer server(80);

// ======================== PINOS ========================
static const uint8_t M1_IN1 = 27, M1_IN2 = 26, M1_ENA = 25;
static const uint8_t M2_IN3 = 5,  M2_IN4 = 18, M2_ENB = 19;
static const uint8_t PIN_S[3] = {32, 33, 35};

static const uint32_t PWM_FREQ = 1000;
static const uint8_t  PWM_RES  = 8;

// ======================== CONFIG ULTRA CONSERVADOR ========================
struct Config {
  // VELOCIDADES - COMEÇAMOS BEM DEVAGAR!
  int velReto       = 100;    // BAIXO de propósito!
  int velCurvaForte = 85;     // Roda de fora
  int velCurvaFreio = 35;     // Roda de dentro
  int velBusca      = 80;
  
  // LIMIAR - MAIS TOLERANTE
  float limiarLinha = 0.30f;  // Mais sensível
  int amostrasADC   = 8;      // MAIS amostras = mais estável
  
  // FILTRO ANTI-CAMBALEIO
  int minCiclosEstado = 3;    // Precisa ver o mesmo estado X vezes seguidas
  float zonamorta = 0.15f;    // Ignora variações pequenas de norm
  
  // RAMPA DE ACELERAÇÃO
  int acelRampa = 5;          // Quanto aumenta por ciclo
  
  unsigned long buscaTimeout = 2000;
} cfg;

// ======================== CALIBRACAO ====================
int calMin[3], calMax[3];

// ======================== ESTADOS =======================
enum Estado { 
  PARADO, 
  RETO, 
  CURVA_ESQ_LEVE,
  CURVA_DIR_LEVE,
  CURVA_ESQ_FORTE, 
  CURVA_DIR_FORTE,
  CRUZAMENTO,
  BUSCA 
};

const char* nomeEstado[] = {
  "PARADO", "RETO", "CURVA ESQ LEVE", "CURVA DIR LEVE",
  "CURVA ESQ FORTE", "CURVA DIR FORTE", "CRUZAMENTO", "BUSCA"
};

// ======================== DADOS =========================
struct {
  int   raw[3];
  float norm[3];
  float normFiltrada[3];  // NOVO: valores filtrados
  bool  on[3];
  
  int   velE, velD;
  int   velEalvo, velDalvo;  // NOVO: velocidades alvo para rampa
  
  Estado estado;
  Estado estadoAnterior;
  int contadorEstado;  // NOVO: conta ciclos no mesmo estado
  
  int ultimaDirecao;  // -1=esq, 0=centro, +1=dir
  
  unsigned long ciclos, tempoLoop, tempoSemLinha;
  unsigned long tUltimaLinha;
} R;

// ======================== FILTRO EXPONENCIAL ========================
// Suaviza leituras para evitar cambaleio
float filtroExp(float valorNovo, float valorAntigo, float alpha = 0.3f) {
  return (alpha * valorNovo) + ((1.0f - alpha) * valorAntigo);
}

// ============================================================
//  MOTORES
// ============================================================
void motE(int p) {
  p = constrain(p, 0, 255);
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  ledcWrite(M1_ENA, p);
}

void motD(int p) {
  p = constrain(p, 0, 255);
  digitalWrite(M2_IN3, HIGH);
  digitalWrite(M2_IN4, LOW);
  ledcWrite(M2_ENB, p);
}

void parar() {
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN3, LOW); digitalWrite(M2_IN4, LOW);
  ledcWrite(M1_ENA, 0);
  ledcWrite(M2_ENB, 0);
}

// ============================================================
//  SENSORES COM FILTRO
// ============================================================
float normalizar(int raw, int mn, int mx) {
  if (mx <= mn) return 0.0f;
  return constrain((float)(raw - mn) / (float)(mx - mn), 0.0f, 1.0f);
}

void lerSensores() {
  for (int s = 0; s < 3; s++) {
    long soma = 0;
    for (int i = 0; i < cfg.amostrasADC; i++) {
      soma += analogRead(PIN_S[s]);
      delayMicroseconds(50);  // Pequeno delay entre leituras
    }
    R.raw[s] = soma / cfg.amostrasADC;
    
    float normNova = normalizar(R.raw[s], calMin[s], calMax[s]);
    
    // FILTRO EXPONENCIAL - suaviza muito!
    R.normFiltrada[s] = filtroExp(normNova, R.normFiltrada[s], 0.25f);
    R.norm[s] = R.normFiltrada[s];
    
    // ZONA MORTA - ignora pequenas variações
    if (R.norm[s] < cfg.zonamorta) {
      R.norm[s] = 0.0f;
    } else if (R.norm[s] > (1.0f - cfg.zonamorta)) {
      R.norm[s] = 1.0f;
    }
    
    R.on[s] = (R.norm[s] > cfg.limiarLinha);
  }
  
  bool linhaDetectada = R.on[0] || R.on[1] || R.on[2];
  if (linhaDetectada) {
    R.tUltimaLinha = millis();
    R.tempoSemLinha = 0;
  } else {
    R.tempoSemLinha = millis() - R.tUltimaLinha;
  }
}

// ============================================================
//  DECIDIR ESTADO - COM FILTRO DE ESTABILIDADE
// ============================================================
Estado calcularEstado() {
  bool E = R.on[0];
  bool C = R.on[1];
  bool D = R.on[2];
  
  // CRUZAMENTO - todos veem
  if (E && C && D) {
    return CRUZAMENTO;
  }
  
  // BIFURCAÇÃO - laterais sem centro
  if (E && !C && D) {
    return CRUZAMENTO;
  }
  
  // RETO - só centro
  if (!E && C && !D) {
    return RETO;
  }
  
  // CURVA LEVE ESQUERDA - centro + esquerda
  if (E && C && !D) {
    return CURVA_ESQ_LEVE;
  }
  
  // CURVA LEVE DIREITA - centro + direita
  if (!E && C && D) {
    return CURVA_DIR_LEVE;
  }
  
  // CURVA FORTE ESQUERDA - só esquerda
  if (E && !C && !D) {
    return CURVA_ESQ_FORTE;
  }
  
  // CURVA FORTE DIREITA - só direita
  if (!E && !C && D) {
    return CURVA_DIR_FORTE;
  }
  
  // NENHUM - busca
  return BUSCA;
}

// ============================================================
//  APLICAR RAMPA SUAVE NAS VELOCIDADES
// ============================================================
void aplicarRampa() {
  // Aproxima velocidade atual da alvo gradualmente
  if (R.velE < R.velEalvo) {
    R.velE = min(R.velE + cfg.acelRampa, R.velEalvo);
  } else if (R.velE > R.velEalvo) {
    R.velE = max(R.velE - cfg.acelRampa, R.velEalvo);
  }
  
  if (R.velD < R.velDalvo) {
    R.velD = min(R.velD + cfg.acelRampa, R.velDalvo);
  } else if (R.velD > R.velDalvo) {
    R.velD = max(R.velD - cfg.acelRampa, R.velDalvo);
  }
}

// ============================================================
//  DECIDIR E EXECUTAR
// ============================================================
void decidirEExecutar() {
  Estado estadoNovo = calcularEstado();
  
  // FILTRO DE ESTABILIDADE - só muda se ver o mesmo estado várias vezes
  if (estadoNovo == R.estadoAnterior) {
    R.contadorEstado++;
  } else {
    R.contadorEstado = 0;
    R.estadoAnterior = estadoNovo;
  }
  
  // Só muda estado se for estável OU se for busca (emergência)
  if (R.contadorEstado >= cfg.minCiclosEstado || estadoNovo == BUSCA) {
    R.estado = estadoNovo;
  }
  
  // Define velocidades ALVO baseado no estado
  switch (R.estado) {
    case RETO:
      R.velEalvo = cfg.velReto;
      R.velDalvo = cfg.velReto;
      R.ultimaDirecao = 0;
      break;
      
    case CURVA_ESQ_LEVE:
      R.velEalvo = cfg.velCurvaFreio;
      R.velDalvo = cfg.velCurvaForte;
      R.ultimaDirecao = -1;
      break;
      
    case CURVA_DIR_LEVE:
      R.velEalvo = cfg.velCurvaForte;
      R.velDalvo = cfg.velCurvaFreio;
      R.ultimaDirecao = 1;
      break;
      
    case CURVA_ESQ_FORTE:
      R.velEalvo = 0;
      R.velDalvo = cfg.velCurvaForte;
      R.ultimaDirecao = -1;
      break;
      
    case CURVA_DIR_FORTE:
      R.velEalvo = cfg.velCurvaForte;
      R.velDalvo = 0;
      R.ultimaDirecao = 1;
      break;
      
    case CRUZAMENTO:
      R.velEalvo = cfg.velReto;
      R.velDalvo = cfg.velReto;
      break;
      
    case BUSCA:
      if (R.ultimaDirecao <= 0) {
        R.velEalvo = 0;
        R.velDalvo = cfg.velBusca;
      } else {
        R.velEalvo = cfg.velBusca;
        R.velDalvo = 0;
      }
      break;
      
    default:
      R.velEalvo = 0;
      R.velDalvo = 0;
      break;
  }
  
  // Aplica rampa suave
  aplicarRampa();
  
  // Envia para motores
  motE(R.velE);
  motD(R.velD);
}

// ============================================================
//  CALIBRACAO
// ============================================================
void calibrar() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║     CALIBRAÇÃO AUTOMÁTICA              ║");
  Serial.println("║  Posicione sobre a LINHA (preto)       ║");
  Serial.println("╚════════════════════════════════════════╝");
  delay(3000);

  for (int i = 0; i < 3; i++) {
    calMin[i] = 4095;
    calMax[i] = 0;
  }

  auto amostra = [&]() {
    for (int i = 0; i < 3; i++) {
      int v = analogRead(PIN_S[i]);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
  };

  Serial.println("\n[1/4] ⏸️  Amostrando parado sobre linha...");
  for (int i = 0; i < 30; i++) { amostra(); delay(50); }

  Serial.println("[2/4] ↶  Girando esquerda (preto+branco)...");
  motD(70); motE(0);
  for (int i = 0; i < 40; i++) { amostra(); delay(50); }

  Serial.println("[3/4] ↷  Girando direita (preto+branco)...");
  motE(70); motD(0);
  for (int i = 0; i < 80; i++) { amostra(); delay(50); }

  Serial.println("[4/4] ⏹️  Centralizando...");
  motD(70); motE(0);
  delay(1000);
  parar();

  // Expande ranges
  for (int i = 0; i < 3; i++) {
    int range = calMax[i] - calMin[i];
    calMin[i] = max(0, calMin[i] - range / 10);
    calMax[i] = min(4095, calMax[i] + range / 10);
  }

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  RESULTADO DA CALIBRAÇÃO               ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  const char* lb[] = {"ESQ", "CEN", "DIR"};
  for (int i = 0; i < 3; i++) {
    int range = calMax[i] - calMin[i];
    Serial.printf("║  %s: %4d - %4d (range=%4d) %s\n",
                  lb[i], calMin[i], calMax[i], range,
                  range > 200 ? "✓ OK     ║" : "✗ RUIM!  ║");
  }
  Serial.println("╚════════════════════════════════════════╝");

  // Teste DETALHADO
  Serial.println("\n📊 TESTE DETALHADO:\n");
  Serial.println("Mova o robô: LINHA → FORA → LINHA → FORA\n");
  
  for (int t = 0; t < 12; t++) {
    lerSensores();
    Serial.printf("  [%d%d%d] raw=[%4d %4d %4d] norm=[%.2f %.2f %.2f]\n",
                  R.on[0], R.on[1], R.on[2],
                  R.raw[0], R.raw[1], R.raw[2],
                  R.norm[0], R.norm[1], R.norm[2]);
    delay(400);
  }
  
  Serial.println("\n✓ Calibração concluída!");
}

// ============================================================
//  DASHBOARD
// ============================================================
const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OBR Estável</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#111;color:#eee;padding:15px}
h1{text-align:center;font-size:1.4em;color:#4a9eff;margin-bottom:15px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;max-width:1000px;margin:0 auto}
.c{background:#1a1a1a;border:2px solid #333;border-radius:12px;padding:15px}
.c h2{font-size:.75em;color:#888;text-transform:uppercase;letter-spacing:2px;margin-bottom:12px;border-bottom:1px solid #333;padding-bottom:6px}
.est{font-size:1.8em;font-weight:700;text-align:center;padding:12px 0;border-radius:8px;background:#222}
.info{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-top:10px}
.box{background:#0a0a0a;padding:10px;border-radius:6px;text-align:center;border:1px solid #333}
.val{font-size:1.5em;font-weight:700;font-family:monospace;color:#4a9eff}
.lbl{font-size:.6em;color:#888;margin-top:4px;text-transform:uppercase}
.sens{display:flex;justify-content:space-around;padding:10px 0}
.s{text-align:center;flex:1}
.bar{width:50px;height:100px;background:#0a0a0a;border:2px solid #333;border-radius:8px;margin:0 auto;position:relative;overflow:hidden}
.fill{position:absolute;bottom:0;width:100%;transition:height .1s}
.slbl{font-size:.7em;color:#888;margin-top:6px}
.sval{font-size:.65em;font-family:monospace;color:#aaa;margin-top:3px}
.dot{width:12px;height:12px;border-radius:50%;margin:5px auto}
.mtr{display:flex;justify-content:space-around;padding:15px 0}
.m{text-align:center}
.mval{font-size:2.5em;font-weight:700;font-family:monospace}
.mlbl{font-size:.65em;color:#888;margin-top:5px}
.mbar{width:90px;height:10px;background:#0a0a0a;border-radius:6px;margin:8px auto;overflow:hidden;border:1px solid #333}
.mfill{height:100%;transition:width .1s}
.status{text-align:center;font-size:.7em;padding:8px;margin-top:10px;border-radius:6px}
.ok{background:#1a3a1a;color:#4f4}
.er{background:#3a1a1a;color:#f44}
</style></head><body>
<h1>🤖 SEGUIDOR ULTRA ESTÁVEL</h1>
<div class="g">
<div class="c"><h2>Estado Atual</h2><div class="est" id="est">---</div><div class="status" id="st">...</div></div>
<div class="c"><h2>Sensores</h2><div class="sens">
<div class="s"><div class="bar"><div class="fill" id="f0"></div></div><div class="slbl">ESQ</div><div class="sval" id="v0">-</div><div class="dot" id="d0"></div></div>
<div class="s"><div class="bar"><div class="fill" id="f1"></div></div><div class="slbl">CEN</div><div class="sval" id="v1">-</div><div class="dot" id="d1"></div></div>
<div class="s"><div class="bar"><div class="fill" id="f2"></div></div><div class="slbl">DIR</div><div class="sval" id="v2">-</div><div class="dot" id="d2"></div></div>
</div></div>
<div class="c"><h2>Motores</h2><div class="mtr">
<div class="m"><div class="mval" id="me">0</div><div class="mlbl">ESQUERDO</div><div class="mbar"><div class="mfill" id="be"></div></div></div>
<div class="m"><div class="mval" id="md">0</div><div class="mlbl">DIREITO</div><div class="mbar"><div class="mfill" id="bd"></div></div></div>
</div></div>
<div class="c"><h2>Informações</h2><div class="info">
<div class="box"><div class="val" id="i1">0</div><div class="lbl">Ciclos</div></div>
<div class="box"><div class="val" id="i2">0ms</div><div class="lbl">Sem Linha</div></div>
<div class="box"><div class="val" id="i3">---</div><div class="lbl">Padrão</div></div>
</div></div>
</div>
<script>
let lt=0;
function u(){fetch('/d').then(r=>r.json()).then(d=>{lt=Date.now();
let st=document.getElementById('st');st.className='status ok';st.textContent='● ONLINE';
let e=document.getElementById('est');e.textContent=d.e;
e.style.color=d.e==='PARADO'?'#666':d.e==='RETO'?'#4f4':d.e.includes('BUSCA')?'#fa0':d.e.includes('FORTE')?'#f44':'#fb0';
for(let i=0;i<3;i++){
let p=Math.round(d.n[i]*100);
document.getElementById('f'+i).style.height=p+'%';
document.getElementById('f'+i).style.background=d.o[i]?'linear-gradient(to top,#4f4,#6f6)':'#333';
document.getElementById('v'+i).textContent=d.r[i]+' ('+p+'%)';
document.getElementById('d'+i).style.background=d.o[i]?'#4f4':'#444';
}
document.getElementById('me').textContent=d.ml;document.getElementById('md').textContent=d.mr;
let ce=d.ml>80?'#4f4':d.ml>30?'#fb0':'#666',cd=d.mr>80?'#4f4':d.mr>30?'#fb0':'#666';
document.getElementById('me').style.color=ce;document.getElementById('md').style.color=cd;
document.getElementById('be').style.width=(d.ml/255*100)+'%';document.getElementById('be').style.background=ce;
document.getElementById('bd').style.width=(d.mr/255*100)+'%';document.getElementById('bd').style.background=cd;
document.getElementById('i1').textContent=d.cy;
document.getElementById('i2').textContent=d.sl+'ms';
document.getElementById('i3').textContent=(d.o[0]?'●':'○')+(d.o[1]?'●':'○')+(d.o[2]?'●':'○');
}).catch(()=>{if(Date.now()-lt>3000){let st=document.getElementById('st');st.className='status er';st.textContent='● OFFLINE'}})}
setInterval(u,100);
</script></body></html>
)rawliteral";

void handleRoot() { 
  server.send(200, "text/html", PAGINA); 
}

void handleData() {
  char buf[350];
  snprintf(buf, sizeof(buf),
    "{\"e\":\"%s\",\"r\":[%d,%d,%d],\"n\":[%.2f,%.2f,%.2f],\"o\":[%d,%d,%d],"
    "\"ml\":%d,\"mr\":%d,\"cy\":%lu,\"sl\":%lu}",
    nomeEstado[R.estado],
    R.raw[0], R.raw[1], R.raw[2],
    R.norm[0], R.norm[1], R.norm[2],
    R.on[0], R.on[1], R.on[2],
    R.velE, R.velD,
    R.ciclos, R.tempoSemLinha);
  server.send(200, "application/json", buf);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║   SEGUIDOR ULTRA ESTÁVEL v2.0          ║");
  Serial.println("║   Anti-Cambaleio Profissional          ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // Motores
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  ledcAttach(M1_ENA, PWM_FREQ, PWM_RES);
  pinMode(M2_IN3, OUTPUT); pinMode(M2_IN4, OUTPUT);
  ledcAttach(M2_ENB, PWM_FREQ, PWM_RES);
  parar();
  
  // Sensores
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_S[i], INPUT);
    R.normFiltrada[i] = 0.0f;
  }

  // WiFi
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  server.on("/", handleRoot);
  server.on("/d", handleData);
  server.begin();
  Serial.printf("📡 Dashboard: http://%s\n\n", WiFi.softAPIP().toString().c_str());

  // Teste motores
  Serial.println("🔧 Testando motores...");
  motE(100); delay(600); motE(0); delay(200);
  motD(100); delay(600); motD(0); delay(200);
  motE(100); motD(100); delay(700); parar();
  Serial.println("✓ OK\n");

  // Calibração
  calibrar();

  // Countdown
  Serial.println();
  for (int i = 3; i > 0; i--) {
    Serial.printf("⏱️  Iniciando em %d...\n", i);
    delay(1000);
  }
  
  Serial.println("\n🚀 RODANDO COM CONTROLE ESTÁVEL!\n");
  
  R.tUltimaLinha = millis();
  R.ultimaDirecao = 0;
  R.estado = PARADO;
  R.estadoAnterior = PARADO;
  R.contadorEstado = 0;
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  unsigned long t0 = micros();
  R.ciclos++;

  server.handleClient();
  lerSensores();

  bool temLinha = R.on[0] || R.on[1] || R.on[2];

  if (temLinha || R.tempoSemLinha < cfg.buscaTimeout) {
    decidirEExecutar();
  } else {
    // Timeout total
    R.estado = PARADO;
    R.velEalvo = 0;
    R.velDalvo = 0;
    aplicarRampa();
    motE(R.velE);
    motD(R.velD);
  }

  R.tempoLoop = micros() - t0;

  // Debug
  static unsigned long tDebug = 0;
  if (millis() - tDebug >= 200) {
    tDebug = millis();
    Serial.printf("[%d%d%d] %-17s E=%3d D=%3d norm=[%.2f %.2f %.2f]\n",
                  R.on[0], R.on[1], R.on[2],
                  nomeEstado[R.estado],
                  R.velE, R.velD,
                  R.norm[0], R.norm[1], R.norm[2]);
  }

  delay(5);  // Delay maior = mais estável
}
