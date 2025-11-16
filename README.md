# 🚀 e22\_stm32 - Implementações LoRa (E22) para STM32F1 & STM32F4

This repository contains the development of embedded firmware for the **E22 LoRa module (SX1262)**, with implementations for the **STM32F1** and **STM32F4** families. The project was developed within the context of the **Propulsion and Aerospace Technology Team (EPTA) at the Federal University of Uberlândia (UFU)** for testing and validating LoRa communication.

## 📡 Project Overview

This project explores the communication between the STM32 platform and the E22 LoRa module, serving as a basis for telemetry systems. The repository contains separate implementations for different microcontroller families.

The system consists of:

  - **Microcontrollers:** STM32F1 (e.g., STM32F103C6) and STM32F4 (e.g., STM32F411CE)
  - **LoRa Module:** [E22-900T22D (SX1262 Chip)](https://www.cdebyte.com/products/E22-900T22D)
  - **Communication:** UART
  - **Development Platform:** STM32CubeIDE

The EPTA team develops these technologies for participation in the **Latin American Space Challenge (LASC)**, one of the largest international model rocket competitions. 🌎🚀

## 🔗 Development Base

This project was inspired by the work of the channel **Useful Electronics**, using as reference:

  - 📺 Channel: [Useful Electronics](https://www.youtube.com/@usefulelectronics)
  - 🎥 Video Tutorial: [STM32 FreeRTOS and E22 LoRa](https://www.youtube.com/watch?v=fPgSf5HOfO8&t=1s)
  - 📂 Repository: [GitHub - UsefulElectronics](https://github.com/UsefulElectronics/stm32-freertos-e22-lora)
  - 📖 Blog: [Useful Electronics Blog](https://www.usefulelectronics.net/)

## 📁 Repository Structure

The repository is divided into folders, one for each microcontroller family implementation.

```
/
├── stm32f1/            # Project for the STM32F1 family
│   ├── Core/           # Main source code
│   ├── Drivers/        # STM32 drivers
│   ├── Middlewares/
│   ├── PAY_lora_maldito.ioc    # STM32CubeMX configuration
│   └── ...
│
├── stm32f4/            # Project for the STM32F4 family
│   ├── Core/           # Main source code
│   ├── Drivers/        # STM32 drivers
│   ├── stm32_ssd1306_e22_lora.ioc # STM32CubeMX configuration
│   └── ...
│
└── README.md           # This file
```

## 🚀 How to Configure and Compile

1.  Clone the repository:
    ```bash
    git clone https://github.com/GuilhermeAsura/e22_stm32.git
    ```
2.  Navigate to the desired project folder (e.g., `cd stm32f1` or `cd stm32f4`).
3.  Open the project in STM32CubeIDE (you can import it using the `.ioc` file).
4.  In the STM32CubeMX view (by opening the `.ioc` file), check the UART communication settings according to the hardware used.
5.  Compile and upload the firmware to the STM32 board.

## 🤝 Contributions

This project was developed in collaboration with **Equipe de Propulsão e Tecnologia Aeroespacial (EPTA/UFU)** and any contribution for improvements is welcome\! If you have any suggestions or want to report problems, feel free to open an issue or pull request.

-----

PT-BR

# 🚀 e22\_stm32 - Implementações LoRa (E22) para STM32F1 & STM32F4

Este repositório contém o desenvolvimento de firmwares embarcados para o **módulo LoRa E22 (SX1262)**, com implementações para as famílias **STM32F1** e **STM32F4**. O projeto foi desenvolvido no contexto da **Equipe de Propulsão e Tecnologia Aeroespacial (EPTA) da Universidade Federal de Uberlândia (UFU)** para testes e validação da comunicação LoRa.

## 📡 Visão Geral do Projeto

Este projeto explora a comunicação entre a plataforma STM32 e o módulo LoRa E22, servindo como base para sistemas de telemetria. O repositório contém implementações separadas para diferentes famílias de microcontroladores.

O sistema consiste em:

  - **Microcontroladores:** STM32F1 (ex: STM32F103C6) e STM32F4 (ex: STM32F411CE)
  - **Módulo LoRa:** [E22-900T22D (Chip SX1262)](https://www.cdebyte.com/products/E22-900T22D)
  - **Comunicação:** UART
  - **Plataforma de Desenvolvimento:** STM32CubeIDE

A equipe da EPTA desenvolve essas tecnologias para participação na **Latin American Space Challenge (LASC)**, uma das maiores competições internacionais de foguetemodelismo. 🌎🚀

## 🔗 Base de Desenvolvimento

Este projeto foi inspirado no trabalho do canal **Useful Electronics**, utilizando como referência:

  - 📺 Canal: [Useful Electronics](https://www.youtube.com/@usefulelectronics)
  - 🎥 Vídeo Tutorial: [STM32 FreeRTOS e E22 LoRa](https://www.youtube.com/watch?v=fPgSf5HOfO8&t=1s)
  - 📂 Repositório: [GitHub - UsefulElectronics](https://github.com/UsefulElectronics/stm32-freertos-e22-lora)
  - 📖 Blog: [Useful Electronics Blog](https://www.usefulelectronics.net/)

## 📁 Estrutura do Repositório

O repositório é dividido em pastas, uma para cada implementação por família de microcontrolador.

```
/
├── stm32f1/            # Projeto para a família STM32F1
│   ├── Core/           # Código-fonte principal
│   ├── Drivers/        # Drivers do STM32
│   ├── Middlewares/
│   ├── PAY_lora_maldito.ioc    # Configuração do STM32CubeMX
│   └── ...
│
├── stm32f4/            # Projeto para a família STM32F4
│   ├── Core/           # Código-fonte principal
│   ├── Drivers/        # Drivers do STM32
│   ├── stm32_ssd1306_e22_lora.ioc # Configuração do STM32CubeMX
│   └── ...
│
└── README.md           # Este arquivo
```

## 🚀 Como Configurar e Compilar

1.  Clone o repositório:
    ```bash
    git clone https://github.com/GuilhermeAsura/e22_stm32.git
    ```
2.  Navegue até a pasta do projeto desejado (ex: `cd stm32f1` ou `cd stm32f4`).
3.  Abra o projeto no STM32CubeIDE (é possível importá-lo usando o arquivo `.ioc`).
4.  Na visualização do STM32CubeMX (abrindo o arquivo `.ioc`), verifique a configuração da comunicação UART de acordo com o hardware utilizado.
5.  Compile e faça o upload do firmware para a placa STM32.

## 🤝 Contribuições

Este projeto foi desenvolvido em colaboração com a **Equipe de Propulsão e Tecnologia Aeroespacial (EPTA/UFU)** e qualquer contribuição para melhorias é bem-vinda\! Caso tenha sugestões ou queira relatar problemas, sinta-se à vontade para abrir uma issue ou pull request.
