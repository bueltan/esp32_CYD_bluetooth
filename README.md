# Prueba PCM por Bluetooth

Prueba independiente de PIUU_CORE. Emula la salida del decodificador Opus con
`oct4_audio_half.raw`, incorporado en flash. No necesita SD, Wi-Fi, UDP ni Opus.
Se interpreta como PCM **s16le, mono, 16000 Hz**: 1.920.000 bytes = 60 segundos.
RAW no tiene cabecera; esos parámetros son una suposición basada en el tamaño
junto con la frecuencia indicada para el archivo.

Flujo: RAW → bloques de 320 muestras / 20 ms → interpolación lineal con estado
16 → **44,1 kHz** → duplicación mono a estéreo → cola de 32 KiB → A2DP/SBC.
El archivo se repite. El salto entre su final y su inicio puede ser audible.
El remuestreador es una referencia simple para validar funcionamiento y costo;
no implementa el filtro de reconstrucción de un remuestreador de alta fidelidad.
No reinicia la fase entre bloques: conserva duración y tono.

## Ejecutar

Desde esta carpeta:

```sh
pio run
pio run -t upload
pio device monitor
```

Poner los auriculares en modo emparejamiento. El destino es `WH-CH720N`;
cambiar `targetName` en `src/main.cpp` para otro parlante. Comando Serial:
`p` pausa/reanuda. Volumen inicial A2DP: 80/127.

La partición usa los 4 MB de flash para una sola aplicación más NVS/PHY.
No incluye OTA ni filesystem. El RAW ocupa 1,92 MB de la aplicación y no se
carga entero en RAM. La versión MP3 anterior está en `src/main.cpp.mp3.bak`.

## Diagnóstico cada ~3 segundos

- `TIME resample`: llamadas, total, promedio y máximo en ms del remuestreo.
- `TIME RAW+cola`: lectura RAW, remuestreo y encolado. Incluye `resample`;
  no sumar ambas cifras. `TIME callback`: copia PCM/silencio para A2DP.
- `TIME cores`: cores donde efectivamente se ejecutó cada etapa en esa ventana.
- `CPU core=... ocupado~=`: estimación por core usando el tiempo no dedicado a
  la tarea IDLE. Incluye Bluetooth y otras tareas, no solo nuestro código.
- `TASK`: tiempo CPU real programado por FreeRTOS en ms y porcentaje por tarea,
  afinidad y mínimo de stack libre. 100% equivale a un core completo; una tarea
  con afinidad libre puede migrar. Se omiten tareas con menos de 0,5 ms salvo
  `loopTask`. Una tarea nueva se informa tras obtener su segunda muestra.
- `SCHED`: bloques demorados más de 2 ms y mayor atraso de entrega en la ventana.
- `AUDIO`: posición del productor en el RAW, vueltas, cola en bytes y ms,
  callbacks, underruns y frames reemplazados por silencio (acumulados), heap.

Los tiempos `TIME` son duración de pared en microsegundos convertida a ms:
pueden incluir interrupciones o desalojo por otras tareas. Para consumo de CPU
usar `TASK` y `CPU`. Los contadores CPU usan diferencias entre snapshots y
admiten el wrap del contador de 32 bits. La primera muestra establece la base.
La pausa ayuda a comparar la carga con/sin remuestreo; Bluetooth sigue enviando
silencio y codificando SBC, por lo que no representa un Bluetooth apagado.

La producción corre en `loopTask` (core 1 en este SDK) y cede con `delay(1)`.
Bluedroid y el controlador Bluetooth están fijados al core 0. Por tanto se usan
**ambos cores**, sin reservarlos exclusivamente ni ejecutar esperas activas.
Los logs corren en `audio_diag`, separada del productor, para que Serial no
frene la entrega de bloques. El callback no bloquea esperando PCM; ante falta
rellena con silencio. Las estadísticas tienen un costo propio, visible en
`audio_diag`; la ocupación es una estimación del planificador, no un perfilador
de hardware de interrupciones.

## Integración posterior

`enqueueDecodedPcm(samples, count)` acepta PCM mono s16 a 16 kHz, hasta 320
muestras por llamada, desde **un solo productor**. Si retorna false, conservar
el bloque y reintentarlo: hay contrapresión. Para paquetes mayores, dividirlos
sin reiniciar el remuestreador. Sustituir `produceRawBlock()` por la salida del
decodificador; la temporización real deberá seguir los timestamps/jitter UDP.
Esta prueba no emula pérdida de paquetes ni jitter de red.

Solo se entrega PCM estéreo s16 a 44,1 kHz al callback A2DP. El SDK Arduino
precompilado tiene deshabilitado `CONFIG_BT_A2DP_USE_EXTERNAL_CODEC`, por lo que
no se intenta configurar SBC a 16 kHz. Cambiar únicamente la tasa de las muestras
del callback alteraría velocidad y tono, sin negociar 16 kHz con el parlante.

## Prueba del remuestreador en host

```sh
g++ -std=c++11 -Wall -Wextra -Werror -Iinclude test/pcm_resampler/test_resampler.cpp -o /tmp/bluetooth-pcm-test
/tmp/bluetooth-pcm-test
```

Valida 60 s a 44,1 kHz, continuidad entre tamaños de bloque, canales idénticos,
señal constante y límites de salida. La escucha, los underruns y la carga de
los cores deben verificarse en el ESP32 conectado al parlante.
