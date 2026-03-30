# Cambios de migracion a FreeRTOS (rama rtos)

## Objetivo
Se inicio una primera fase de adaptacion para alinear el firmware con practicas de concurrencia y temporizacion nativas de FreeRTOS en ESP32.

## Archivo modificado
- transmitter-dummy/src/main.cpp
- receiver-dummy/src/main.cpp

## Resumen de cambios

### Receiver: productor/consumidor con cola FreeRTOS
- Antes: coordinacion por contador atomico (packageCounter), indice de cola manual (bufferTailIndex) y polling en la tarea de envio.
- Ahora: coordinacion por QueueHandle_t (readyPackageQueue) con xQueueCreate(), xQueueSend() y xQueueReceive().

Cambios concretos:
- Se agregaron includes de FreeRTOS para colas:
  - freertos/FreeRTOS.h
  - freertos/queue.h
- Se creo una cola de paquetes listos con capacidad PACKAGE_SLOTS.
- La tarea sendBuffer dejo de usar polling y ahora bloquea en xQueueReceive(portMAX_DELAY).
- readPhotorresistor ahora encola el indice de inicio de cada paquete completo.
- Cuando la cola esta llena, se descarta el paquete mas antiguo para mantener el mas reciente (politica drop-oldest).

Por que se hizo:
- Elimina espera activa y reduce uso innecesario de CPU.
- Evita condiciones de carrera entre productor y consumidor por variables compartidas manuales.
- Mejora el desacoplo entre adquisicion de datos y envio HTTP.

### 1) Sustitucion de std::mutex por mutex de FreeRTOS
- Antes: se usaba std::mutex y std::lock_guard para proteger el cliente HTTP compartido.
- Ahora: se usa SemaphoreHandle_t con xSemaphoreCreateMutex(), xSemaphoreTake() y xSemaphoreGive().

Cambios concretos:
- Se eliminaron includes de C++ mutex.
- Se agregaron includes de FreeRTOS:
  - freertos/FreeRTOS.h
  - freertos/semphr.h
- Se creo el mutex en setup() y se valido su creacion.

Por que se hizo:
- En firmware embebido con tareas RTOS, los mutex de FreeRTOS son la opcion mas predecible e integrada con el scheduler.
- Se mejora la consistencia del modelo de concurrencia al no mezclar primitivas de libreria estandar con primitivas del RTOS.

### 2) Reemplazo de sleep(2) por vTaskDelay(pdMS_TO_TICKS(2000))
- Antes: sleep(2) en loop principal para espaciar consultas de estado.
- Ahora: vTaskDelay() con conversion a ticks.

Por que se hizo:
- vTaskDelay() cede CPU de forma explicita al scheduler de FreeRTOS.
- La temporizacion queda alineada con la unidad de planificacion del RTOS, lo que mejora comportamiento bajo carga.

### 3) Proteccion explicita de secciones criticas HTTP en loop() y sendLoop()
- Antes: proteccion con lock_guard.
- Ahora: proteccion con take/give del mutex RTOS.
- Se aseguro liberar el mutex en rutas de error donde se hace break.

Por que se hizo:
- Evita condiciones de carrera cuando dos tareas intentan usar el mismo flujo HTTP simultaneamente.
- Reduce riesgo de bloqueo por recursos compartidos en escenarios de reconexion o errores de red.

## Impacto esperado
- Mejor integracion con FreeRTOS en ESP32 Arduino.
- Menor riesgo de comportamientos no deterministas por mezcla de mecanismos de sincronizacion.
- Base mas solida para siguientes pasos de migracion (event groups y separacion de tareas por rol).

## Alcance de esta fase
Esta fase no cambia el protocolo de red ni la logica funcional principal de envio. El objetivo fue estabilizar sincronizacion y temporizacion en transmitter, y coordinacion de productor/consumidor en receiver.

## Siguientes pasos recomendados
1. Introducir EventGroup para estado de conectividad WiFi y transiciones de estado del firmware.
2. Revisar el flujo SPI (readADC) para que use la misma cola de paquetes y evitar caminos de codigo mixtos.
3. Sustituir delays bloqueantes restantes en conexion WiFi por esperas RTOS donde sea viable.
4. Validar en hardware con prueba de carga y reconexion para confirmar ausencia de deadlocks y perdidas de paquetes.
