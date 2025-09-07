# Solução para Travamento em DQBUF - Driver da Câmera

## Problema Identificado

Baseado nos logs fornecidos:
```
I (60314) example: video0: [CRITICAL] About to DQBUF for frame #445
(travamento - sem resposta do DQBUF)
```

**Root Cause**: O driver V4L2 da câmera está travando na operação `ioctl(VIDIOC_DQBUF)`.

## Novas Melhorias Implementadas

### 1. **Timeout para DQBUF (1 segundo)**
```c
// Usa select() com timeout de 1 segundo antes do DQBUF
I (xxxx) example: video0: [TIMEOUT] Starting 1-second timeout for DQBUF frame #445
E (xxxx) example: video0: [TIMEOUT] DQBUF timeout (1 second) for frame #445 - driver appears stuck
```
- **Previne travamento infinito**
- **Detecta quando driver não responde**

### 2. **Diagnóstico de Driver State**
```c
W (xxxx) example: video0: [DIAGNOSIS] Starting driver diagnosis for frame #445
I (xxxx) example: video0: [DIAGNOSIS] Device is responsive: IMX708
I (xxxx) example: video0: [DIAGNOSIS] Buffer queue status: 16 buffers
```
- **Verifica se device ainda responde**
- **Checa status de buffers**
- **Mostra formato atual**

### 3. **Detecção de Performance Degradada**
```c
W (xxxx) example: video0: [PERFORMANCE] Slow DQBUF #442: 25460 us (consecutive slow: 1)
W (xxxx) example: video0: [PERFORMANCE] Slow DQBUF #443: 30000 us (consecutive slow: 2)
E (xxxx) example: video0: [WARNING] 3 consecutive slow DQBUFs detected - driver may be struggling
```
- **Detecta quando DQBUF > 20ms**
- **Conta operações consecutivas lentas**
- **Alerta quando driver está deteriorando**

### 4. **Recovery Automático**
```c
W (xxxx) example: video0: [RECOVERY] Attempting stream restart after timeout
```
- **STREAMOFF + STREAMON** para reset
- **Tenta recuperar sem restart total**

### 5. **Health Check Periódico do Driver**
```c
I (xxxx) example: video0: [DRIVER-CHECK] Before DQBUF frame #450 - checking driver health
I (xxxx) example: video0: [DRIVER-OK] Driver responsive before frame #450
```
- **A cada 50 frames verifica QUERYCAP**
- **Detecta problemas antes do travamento**

## Como os Logs Vão Aparecer Agora

### **Cenário Normal**
```
I (xxxx) example: video0: [TIMEOUT] Frame data available, proceeding with DQBUF frame #445
I (xxxx) example: video0: [CRITICAL] DQBUF completed for frame #445 in 35 us
I (xxxx) example: video0: [SUCCESS] Frame #445 completed successfully
```

### **Cenário de Degradação (Warning)**
```
W (xxxx) example: video0: [PERFORMANCE] Slow DQBUF #443: 25460 us (consecutive slow: 1)
W (xxxx) example: video0: [PERFORMANCE] Slow DQBUF #444: 30000 us (consecutive slow: 2)
W (xxxx) example: video0: [PERFORMANCE] Slow DQBUF #445: 35000 us (consecutive slow: 3)
E (xxxx) example: video0: [WARNING] 3 consecutive slow DQBUFs detected - driver may be struggling
W (xxxx) example: video0: [DIAGNOSIS] Starting driver diagnosis for frame #445
```

### **Cenário de Timeout (Recovery)**
```
I (xxxx) example: video0: [TIMEOUT] Starting 1-second timeout for DQBUF frame #445
E (xxxx) example: video0: [TIMEOUT] DQBUF timeout (1 second) for frame #445 - driver appears stuck
W (xxxx) example: video0: [DIAGNOSIS] Starting driver diagnosis for frame #445
W (xxxx) example: video0: [RECOVERY] Attempting stream restart after timeout
```

## Root Causes Possíveis

### 1. **Driver IMX708 Issue**
- **Sintoma**: Timeout em DQBUF
- **Causa**: Bug no driver do sensor
- **Solução**: Update driver ou workaround

### 2. **Buffer Pool Exhausted**
- **Sintoma**: DQBUFs ficando lentos progressivamente
- **Causa**: Buffers não sendo retornados rápido o suficiente
- **Solução**: Aumentar número de buffers

### 3. **Hardware Clock/Power Issue**
- **Sintoma**: Driver responde mas sem frames
- **Causa**: Clock da câmera instável
- **Solução**: Verificar alimentação/clock

### 4. **Shell Command Interference**
- **Sintoma**: Travamento após comando shell
- **Causa**: Comando shell bloqueia recurso compartilhado
- **Solução**: Proteger recursos críticos

## Próximos Passos

1. **Teste com Recovery**: Veja se o timeout + recovery resolve temporariamente
2. **Analise Pattern**: Veja se sempre trava no mesmo número de frame ou é aleatório
3. **Monitor Performance**: Observe se há degradação antes do travamento
4. **Check Shell Correlation**: Veja se travamento coincide com comandos shell

## Configurações Recomendadas

### **Para Debug Máximo**
```c
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER=32  // Dobrar buffers
```

### **Para Monitoramento**
```bash
idf.py monitor | grep -E "(TIMEOUT|DIAGNOSIS|PERFORMANCE|RECOVERY|CRITICAL.*DQBUF)"
```

## Comandos para Teste

1. **Reproduzir problema**:
   - Execute streaming
   - Mexe no shell até travar
   - Observe se timeout salva a situação

2. **Monitoring script**:
```bash
#!/bin/bash
idf.py monitor | while read line; do
    echo "[$(date)] $line"
    if [[ $line == *"TIMEOUT"* ]] || [[ $line == *"consecutive slow"* ]]; then
        echo "*** PROBLEMA DETECTADO ***" | tee -a debug.log
    fi
done
```

Com essas melhorias, quando o problema acontecer novamente:
- ✅ **Não vai travar infinitamente** (timeout de 1s)
- ✅ **Vai diagnosticar o estado do driver**
- ✅ **Vai tentar recovery automático**
- ✅ **Vai detectar degradação antes do travamento**
- ✅ **Vai dar logs específicos sobre a causa**
