# Guardián de Inundación Comunitario

Sistema IoT de bajo costo para que los vecinos reciban una **alarma cuando el agua sube**, sobre todo de noche, cuando nadie se da cuenta de que el agua ya llegó a la casa.

Proyecto integrador de **Práctica Profesionalizante II** (Instituto Superior San José, Corrientes, Argentina).

## ¿Cómo funciona?

Un nodo ESP8266 mide el nivel del agua con un sensor ultrasónico impermeable y confirma con un sensor de agua en el piso. Según el nivel, suena un buzzer con tonos distintos y se enciende un semáforo LED. Los datos se envían a ThingSpeak y se ven en una web de monitoreo.

| Nivel | Agua sobre el piso | -- Estado - |
|-------|--------------------|-------------|
| - 0 - | - menos de 5 cm -- | - Seguro -- |
| - 1 - | ------ 5 cm ------ | -- Aviso -- |
| - 2 - | ----- 15 cm ------ | - Alerta -- |
| - 3 - | ----- 30 cm ------ | - Peligro - |
| - 4 - | ----- 60 cm ------ |  Evacuación |

De noche (22:00 a 07:00) la alarma sube un escalón. Una subida rápida (3 cm/min o más) también sube un nivel.

## Contenido del repositorio

| -------------- Archivo ------------ | ----------------------------------- Qué es ------------------------------------------ |
|-------------------------------------|---------------------------------------------------------------------------------------|
| ------- `guardian_nodo.ino` ------- | ------- Firmware del nodo (ESP32 / ESP8266), envío a ThingSpeak con `millis()` ------ |
| ---------- `index.html` ----------- | - Aplicación web: demo con tormenta simulada y modo lectura del canal de ThingSpeak - |
| - `guardian_analisis_colab.ipynb` - | ------- Notebook de Google Colab: descarga, limpieza, estadísticas y gráficos ------- |

## Hardware

- ESP32 DevKit (o NodeMCU ESP8266)
- Sensor ultrasónico JSN-SR04T (impermeable)
- Sensor de agua para el piso
- Buzzer pasivo y semáforo LED
- Pulsadores para silenciar y responder

> El pin Echo del JSN-SR04T entrega 5 V: hay que bajarlo a 3,3 V con un divisor de 1 kΩ y 2 kΩ antes de conectarlo a la placa.

## Canal de ThingSpeak

Canal `3533124`. Campos:

1. `nivel_cm`
2. `agua_piso_pct`
3. `velocidad_cm_min`
4. `estado`
5. `respuesta`
6. `dist_cruda_cm`
7. `baseline_cm`
8. `modo_noche`

## Cómo usarlo

1. **Firmware:** abrir `guardian_nodo.ino` en Arduino IDE, completar `WIFI_SSID`, `WIFI_PASS` y `TS_WRITE_KEY`, elegir la placa y cargar.
2. **Web:** abrir el sitio publicado con GitHub Pages (ver enlace más abajo).
3. **Análisis:** abrir el notebook en Google Colab. Con `USAR_DATOS_DE_PRUEBA = True` funciona sin canal; en `False` descarga los datos reales.

## Enlaces

- Web: _(https://luis-fernando-dagon.github.io/GUARDIAN-INUNDACION/)_
- Canal de ThingSpeak: _(completar cuando el canal sea público)_

## Limitaciones

- El sistema **no reemplaza** a los servicios de emergencia (911 / 103 / 100).
- Las alertas dependen de WiFi y de energía en la casa.
- El código del firmware todavía está en etapa de prueba con el hardware real.
- No se guardan direcciones: el mapa trabaja con zonas.

## Equipo

Carlos Luis Fernando Rodriguez — Tecnicatura en Desarrollo de Sofware.
