# Debug Logs Aprimorados - Análise de Travamento de Vídeo

## Novos Logs Implementados para Identificar Travamentos

Baseado no travamento após frame #130 sem logs úteis, implementei logs mais detalhados e específicos:

### 1. **Logs Críticos de DQBUF**
```
I (xxxx) example: video0: [CRITICAL] About to DQBUF for frame #131
I (xxxx) example: video0: [CRITICAL] DQBUF completed for frame #131 in 25000 us
```
- **Localização**: Antes e depois da chamada `ioctl(VIDIOC_DQBUF)`
- **Propósito**: Identificar se o travamento está no DQBUF (operação mais comum de travamento)
- **Timing**: Mede quanto tempo o DQBUF leva para completar

### 2. **Watchdog de Sistema**
```
I (xxxx) example: video0: [WATCHDOG] Still alive at frame #135, time: 1234567890
```
- **Frequência**: A cada 10 segundos
- **Propósito**: Mostrar que o loop principal ainda está executando
- **Uso**: Se não aparecer por mais de 10 segundos, indica travamento total

### 3. **Logs Detalhados de HTTP**
```
D (xxxx) example: video0: [HTTP] About to send header for frame #131
D (xxxx) example: video0: [HTTP] Header sent successfully for frame #131
D (xxxx) example: video0: [HTTP] About to send JPEG data (45678 bytes) for frame #131
E (xxxx) example: video0: [HTTP] Failed to send JPEG data for frame #131, ret=-1 (client disconnected?)
```
- **Propósito**: Detectar desconexão de cliente HTTP
- **Identificação**: Distingue entre travamento do driver e problema de rede

### 4. **Monitoramento de Saúde do Sistema**
```
I (xxxx) example: [HEALTH] Frame #200: Free heap: 85432 bytes, Min free: 52000 bytes
W (xxxx) example: [HEALTH] Low memory warning at frame #205: 45000 bytes free
```
- **Frequência**: A cada 100 frames
- **Métricas**: Heap livre atual e mínimo histórico
- **Alertas**: Warning se heap < 50KB

### 5. **Logs de Sucesso de Frame**
```
I (xxxx) example: video0: [SUCCESS] Frame #131 completed successfully
```
- **Propósito**: Confirmar que cada frame foi processado completamente
- **Uso**: O último frame com [SUCCESS] mostra onde parou

## Como Interpretar os Logs Durante Travamento

### **Cenário 1: Travamento em DQBUF**
```
I (26664) example: video0: Frame #130, time_since_last: 142667 us
I (26665) example: video0: [CRITICAL] About to DQBUF for frame #131
(sem mais logs)
```
**Diagnóstico**: Driver da câmera travou ou parou de produzir frames
**Próximos passos**: Verificar power/clock da câmera, reset do driver

### **Cenário 2: Travamento em HTTP**
```
I (26664) example: video0: Frame #130, time_since_last: 142667 us
I (26665) example: video0: [CRITICAL] About to DQBUF for frame #131
I (26670) example: video0: [CRITICAL] DQBUF completed for frame #131 in 5000 us
D (26671) example: video0: [HTTP] About to send header for frame #131
(sem mais logs)
```
**Diagnóstico**: Cliente HTTP desconectou ou rede travou
**Próximos passos**: Verificar conexão de rede, implementar reconexão

### **Cenário 3: Problema de Memória**
```
I (26664) example: video0: Frame #130, time_since_last: 142667 us
W (26600) example: [HEALTH] Low memory warning at frame #130: 45000 bytes free
I (26665) example: video0: [CRITICAL] About to DQBUF for frame #131
(travamento ou crash)
```
**Diagnóstico**: Falta de memória causando instabilidade
**Próximos passos**: Reduzir buffers, otimizar uso de memória

### **Cenário 4: Shell Interference**
```
I (26600) example: Shell activity detected at time: 1234567890
W (26664) example: video0: Frame #130 processing 64000 us after shell activity
I (26665) example: video0: [CRITICAL] About to DQBUF for frame #131
(sem mais logs)
```
**Diagnóstico**: Comando shell interferiu com pipeline de vídeo
**Próximos passos**: Identificar comando específico, implementar proteção

## Comandos para Monitoramento

### **Durante Execução Normal**
```bash
idf.py monitor | grep -E "(CRITICAL|WATCHDOG|SUCCESS|HEALTH)"
```

### **Para Capturar Travamento**
```bash
idf.py monitor | tee video_debug.log
# Reproduzir problema mexendo no shell
# Analisar últimas linhas do video_debug.log
```

### **Filtrar por Problemas**
```bash
idf.py monitor | grep -E "(error|failed|Low memory|CRITICAL.*DQBUF|HTTP.*Failed)"
```

## Próximos Passos para Diagnóstico

1. **Execute** com os novos logs
2. **Reproduza** o travamento mexendo no shell
3. **Identifique** qual dos cenários acima corresponde ao seu log
4. **Analise** o último log antes do travamento:
   - Se parou em `[CRITICAL] About to DQBUF` → Problema de driver
   - Se parou em `[HTTP] About to send` → Problema de rede
   - Se teve `Low memory warning` → Problema de memória
   - Se teve `Shell activity detected` recente → Interferência de shell

## Melhorias Implementadas

- ✅ **Timing detalhado** de operações críticas
- ✅ **Watchdog timer** para detectar travamentos totais
- ✅ **Monitoramento de memória** para detectar vazamentos
- ✅ **Correlação temporal** com atividade do shell
- ✅ **Detecção de desconexão** de cliente HTTP
- ✅ **Logs de confirmação** de sucesso por frame

Com esses logs, conseguiremos identificar exatamente onde e por que o vídeo está travando.
