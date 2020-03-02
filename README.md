# Receptor
Aqui se encuentra el codigo del sistema de la estacion de tierra para el rover challenge. Este dispositivo es el encargado de recibir todos los datos enviados del sistema de telemetria, para que así estos sean visualizados.

Se tendra una revision tanto más **técnica** como detalles acerca de la programación.

- Configuración de Pines e Interfaces de Comunicación
- Inicializacion de Componentes
    - Antena de Comunicación NRF24L01
    - Pantalla LCD
- Programa principal
    - Recepción, procesamiento, envío y visualizacion de datos
    
**Tomar en cuenta que todo el codigo aqui mostrado se encuentra en el repositorio.**

**Las cabeceras Core/Inc/**

**Las implementaciones Core/Src/**

**IMPORTANTE: El diseño del PCB de este sistema de telemetría se encuentra aqui: https://easyeda.com/cancioalmaraz/recepcion**

## Configuración de Pines e Interfaces de Comunicación
Para poder aclarar la distribucion de los pines, hay mencionar que para este disposito estamos usando una placa de STmicroelectronics, mas precisamente la placa stm32 nucleo F401RE y el microcontrolador que lleva dentro es el STM32F401RET6, para mas detalles chekear el link https://os.mbed.com/platforms/ST-Nucleo-F401RE/.

![image PF260000 en feature-description-include-personalized-no-cpn-medium](https://user-images.githubusercontent.com/47458067/75709718-9a7f3100-5c99-11ea-8cdf-8303fa527c03.jpg)

La configuración de los pines del microcontrolador es importante, para poder saber a que pines van conectados cada componente que se utiliza y se comunica con el MCU. Aqui tenemos la configuración:

![Captura de Pantalla 2020-03-02 a la(s) 15 23 54](https://user-images.githubusercontent.com/47458067/75709846-d6b29180-5c99-11ea-9d9b-ac68ba9dc3bd.png)

## Inicialización de Componentes
En este apartado se detallará cada inicializacion por componente, es importante saber que tipo de configuraciones tienen para poder aprovechar todas sus bondades al máximo.

### Antena de Comunicación NRF24L01
Este componente trabaja sobre los 2,4Ghz, tiene codificación por tubos de direcciones de hasta 5 bytes (0 - (2ˆ(40)-1) ) y canales rf (0-124), también tenemos la opción de poder elegir la velocidad a la que trabaja. Nos da la posibilidad de recibir todos los datos enviados desde la estacion móvil (Sistema de telemetría).

![Captura de Pantalla 2020-03-02 a la(s) 15 33 03](https://user-images.githubusercontent.com/47458067/75710527-1d54bb80-5c9b-11ea-87f7-6fc1beedb65e.png)

### Pantalla LCD
Este componente simplemente nos ayudará a visualizar los datos obtenidos por el sistema.

![Captura de Pantalla 2020-03-02 a la(s) 15 39 17](https://user-images.githubusercontent.com/47458067/75710988-fc409a80-5c9b-11ea-9b0c-c62e657d623d.png)

## Programa principal
Simplemtente el codigo de este dispositvo se encarga de recibir los datos, enviarlos a la PC a través de puerto serial y (opcionalmente) si se quiere, mostrar los datos por la pantalla LCD conectado a una interfaz I2C del dispositvo.

### Recepción, procesamiento, envío y visualizacion de datos
![Captura de Pantalla 2020-03-02 a la(s) 15 59 44](https://user-images.githubusercontent.com/47458067/75712759-d799f200-5c9e-11ea-8a65-ee6e601d14f6.png)
