# 🚀 e22_nashira - LoRa Communication System for Payload

This repository contains the development of an embedded firmware for the LoRa radio system of the **Nashira rocket Payload**, designed by the **Propulsion and Aerospace Technology Team (EPTA) of the Federal University of Uberlândia (UFU)**. The system aims to transmit sensory data collected in flight to a ground base, using LoRa communication.

## 📡 Project Overview

This project is part of the **Payload communication subsystem**, a module embedded in the Nashira rocket that collects and processes sensory data for fusion and noise minimization, ensuring greater accuracy in measurements. The system consists of:

- **Microcontroller:** STM32
- **LoRa Module:** [E22-900T22D (SX1262 Chip)](https://www.cdebyte.com/products/E22-900T22D)
- **Communication:** UART
- **Development Platform:** STM32CubeIDE
- **PCB Design:** Altium Designer

The EPTA team is developing this rocket to participate in the **Latin American Space Challenge (LASC)**, one of the largest international model rocket competitions. 🌎🚀

## 🔗 Development Base

This project was inspired by the work of the channel **Useful Electronics**, using as reference:
- 📺 Channel: [Useful Electronics](https://www.youtube.com/@usefulelectronics)
- 🎥 Video Tutorial: [STM32 FreeRTOS and E22 LoRa](https://www.youtube.com/watch?v=fPgSf5HOfO8&t=1s)
- 📂 Repository: [GitHub - UsefulElectronics](https://github.com/UsefulElectronics/stm32-freertos-e22-lora)
- 📖 Blog: [Useful Electronics Blog](https://www.usefulelectronics.net/)

## 📁 Project Structure Repository

```
/
├── Core/ # Main source code
├── Drivers/ # STM32 drivers
├── E22_LoRa/ # LoRa radio driver implementation
├── Inc/ # Header files
├── Src/ # Source files implementation
├── README.md # This file
└── ...
```

## 🚀 How to Configure and Compile

1. Clone the repository:
```bash
git clone https://github.com/GuilhermeAsura/e22_nashira.git
```

2. Open the project in STM32CubeIDE.

3. Configure the UART communication on the STM32CubeMX according to the hardware used.

4. Compile and upload the firmware to the STM32 board.

## 🤝 Contributions

This project was developed in collaboration with **EPTA - UFU** and any contribution for improvements is welcome! If you have any suggestions or want to report problems, feel free to open an issue or pull request.

## 📜 License

This project is under the MIT license. See the `LICENSE` file for more details.    

---

PT-BR
# 🚀 e22_nashira - Sistema de Comunicação LoRa para Payload

Este repositório contém o desenvolvimento de um firmware embarcado para o sistema de rádio LoRa da **Payload do foguete Nashira**, projetado pela **Equipe de Propulsão e Tecnologia Aeroespacial (EPTA) da Universidade Federal de Uberlândia (UFU)**. O sistema tem como objetivo a transmissão de dados sensoriais coletados em voo para uma base em solo, utilizando comunicação LoRa.

## 📡 Visão Geral do Projeto

Este projeto faz parte do **subsistema de comunicação da Payload**, um módulo embarcado no foguete Nashira que coleta e processa dados sensoriais para fusão e minimização de ruídos, garantindo maior precisão nas medições. O sistema consiste em:

- **Microcontrolador:** STM32
- **Módulo LoRa:** [E22-900T22D (Chip SX1262)](https://www.cdebyte.com/products/E22-900T22D)
- **Comunicação:** UART
- **Plataforma de Desenvolvimento:** STM32CubeIDE
- **Design da PCB:** Altium Designer

A equipe da EPTA está desenvolvendo esse foguete para participação na **Latin American Space Challenge (LASC)**, uma das maiores competições internacionais de foguetemodelismo. 🌎🚀

## 🔗 Base de Desenvolvimento

Este projeto foi inspirado no trabalho do canal **Useful Electronics**, utilizando como referência:
- 📺 Canal: [Useful Electronics](https://www.youtube.com/@usefulelectronics)
- 🎥 Vídeo Tutorial: [STM32 FreeRTOS e E22 LoRa](https://www.youtube.com/watch?v=fPgSf5HOfO8&t=1s)
- 📂 Repositório: [GitHub - UsefulElectronics](https://github.com/UsefulElectronics/stm32-freertos-e22-lora)
- 📖 Blog: [Useful Electronics Blog](https://www.usefulelectronics.net/)

## 📁 Estrutura do Repositório

```
/
├── Core/               # Código-fonte principal
├── Drivers/            # Drivers do STM32
├── E22_LoRa/           # Implementação do driver do rádio LoRa
├── Inc/                # Arquivos de cabeçalho
├── Src/                # Implementação dos arquivos fonte
├── README.md           # Este arquivo
└── ...
```

## 🚀 Como Configurar e Compilar

1. Clone o repositório:
   ```bash
   git clone https://github.com/GuilhermeAsura/e22_nashira.git
   ```

2. Abra o projeto no STM32CubeIDE.

3. Configure a comunicação UART no STM32CubeMX de acordo com o hardware utilizado.

4. Compile e faça o upload do firmware para a placa STM32.

## 🤝 Contribuições

Este projeto foi desenvolvido em colaboração com a **EPTA - UFU** e qualquer contribuição para melhorias é bem-vinda! Caso tenha sugestões ou queira relatar problemas, sinta-se à vontade para abrir uma issue ou pull request.

## 📜 Licença

Este projeto está sob a licença MIT. Consulte o arquivo `LICENSE` para mais detalhes.

