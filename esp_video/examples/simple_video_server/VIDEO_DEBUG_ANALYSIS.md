# Video Debug Logs - Análise de Travamento de Vídeo

## Resumo das Melhorias Implementadas

Este documento descreve as melhorias adicionadas ao `simple_video_server` para identificar e debugar travamentos de vídeo que ocorrem principalmente durante o uso do shell.

## Logs Adicionados

### 1. Pipeline de Captura de Frames (`image_stream_handler`)

**Localização**: Função `image_stream_handler()` no arquivo `simple_video_server_example.c`

**Logs Implementados**:
- **Frame Timing**: Log a cada 30 frames ou quando há atraso > 100ms
- **DQBUF Operations**: Log detalhado das operações de dequeue de buffer
- **QBUF Operations**: Log das operações de queue de buffer de volta
- **Encoding Process**: Log do processo de encoding (se não for JPEG direto)
- **HTTP Transmission**: Log detalhado do envio HTTP de cada frame
- **Error Handling**: Log melhorado de erros com contexto

### 2. Captura de Imagem (`capture_video_image`)

**Localização**: Função `capture_video_image()` no arquivo `simple_video_server_example.c`

**Logs Implementados**:
- **DQBUF/QBUF Operations**: Log das operações V4L2
- **Buffer Status**: Verificação detalhada do status dos buffers
- **Encoding Process**: Log do processo de encoding para captura de imagem
- **HTTP Response**: Log do envio da resposta HTTP

### 3. Correlação com Atividade do Shell

**Arquivos**: `video_debug.h` e modificações em `camera_shell.c`

**Funcionalidades**:
- **Shell Activity Tracking**: Registra quando comandos shell são executados
- **Temporal Correlation**: Correlaciona atividade do shell com problemas de vídeo
- **Time Window**: Monitora 5 segundos após atividade do shell para detectar problemas

## Como Usar os Logs

### 1. Compilar com Logs Habilitados

Certifique-se que o nível de log está configurado apropriadamente:

```bash
idf.py menuconfig
# Component config -> Log output -> Default log verbosity -> Info ou Debug
```

### 2. Monitorar os Logs

Execute o monitor:
```bash
idf.py monitor
```

### 3. Identificar Padrões de Travamento

#### Logs de Frame Normal:
```
I (12345) example: video0: Frame #150, time_since_last: 33333 us
D (12345) example: video0: DQBUF success for frame #151, buf.index=2, bytesused=45678, flags=0x1
D (12345) example: video0: Frame #151 processing complete
```

#### Logs de Problema com DQBUF:
```
E (12345) example: video0: DQBUF failed for frame #151, errno: 11 (Resource temporarily unavailable)
```

#### Logs de Correlação com Shell:
```
I (12345) example: Shell activity detected at time: 1234567890
W (12346) example: video0: Frame #152 processing 50000 us after shell activity
```

#### Logs de Problema HTTP:
```
E (12345) example: video0: Failed to send JPEG data for frame #151, ret=-1
```

## Problemas Mais Comuns Identificados

### 1. **DQBUF Timeout**
- **Sintoma**: `DQBUF failed, errno: 11 (Resource temporarily unavailable)`
- **Causa**: Camera parou de produzir frames
- **Investigação**: Verificar se há problema de clock/power da câmera

### 2. **HTTP Connection Dropped**
- **Sintoma**: `Failed to send JPEG data, ret=-1`
- **Causa**: Cliente HTTP desconectou
- **Solução**: Implementar reconexão automática

### 3. **Buffer Starvation**
- **Sintoma**: Frames com delay > 100ms frequentemente
- **Causa**: Buffers insuficientes ou processamento lento
- **Solução**: Aumentar `CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER`

### 4. **Shell Interference**
- **Sintoma**: Video trava logo após comando shell
- **Causa**: Concorrência entre shell e video pipeline
- **Investigação**: Verificar se comandos shell estão bloqueando recursos

## Configurações Recomendadas para Debug

### 1. Aumentar Buffer Count
```
CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER=16
```

### 2. Habilitar Debug Logs
```
CONFIG_LOG_DEFAULT_LEVEL_DEBUG=y
```

### 3. Monitorar Heap
```
CONFIG_HEAP_TRACING=y
```

## Próximos Passos para Diagnóstico

1. **Compile e Execute** com os novos logs
2. **Reproduza o Problema** mexendo no shell durante streaming
3. **Analise os Logs** procurando por:
   - Padrões temporais entre shell activity e video freeze
   - Tipos específicos de erro (DQBUF, HTTP, etc.)
   - Timing irregulares entre frames
4. **Correlacione** atividade do shell com problemas de vídeo
5. **Identifique Root Cause** baseado nos padrões encontrados

## Arquivo de Debug Adicional

Foi criado o arquivo `video_debug.h` que fornece:
- Função `register_shell_activity()` para tracking
- Interface limpa para expansão futura de debug

## Comandos Shell Instrumentados

Os seguintes comandos shell agora registram atividade:
- `cam_status`
- `hw_awb`

Para adicionar mais comandos, simplesmente chame `register_shell_activity()` no início da função.

## Logs Importantes para Capturar

Durante a reprodução do problema, capture:
1. **Timestamp** exato quando o vídeo trava
2. **Comando shell** que foi executado (se houver)
3. **Últimos frames** processados com sucesso
4. **Mensagens de erro** específicas
5. **Tempo entre frames** antes do travamento

Com essas informações, será possível identificar a root cause do travamento de vídeo.
