
# Sistema de Controle de Velocidade para Motor CC com Supervisórios Local e Remoto

## 🎓 Trabalho de Conclusão de Curso (TCC)

Projeto desenvolvido para o curso de **Bacharelado em Engenharia de Controle e Automação** pelo IFPB - Campus Cajazeiras.

**Autores:**
- Carlos Henrique Sibaldo Torres de Lira
- João Paulo Sibaldo Torres de Lira

---

## 📘 Descrição Geral

Este repositório contém os arquivos referentes à implementação embarcada do controle de velocidade de um motor CC com encoder, utilizando a plataforma ESP32, além de simulações realizadas no Scilab/Xcos com diferentes métodos de sintonia PID.

O sistema inclui:

- Controle PID digital embarcado na ESP32
- Comunicação via MQTT
- Supervisório local (Node-RED)
- Supervisório remoto (Web via Firebase)
- Simulações no Scilab/Xcos com métodos clássicos de sintonia

---

## ⚙️ Estrutura do Projeto

```
📦Arquivos de Simulação e Implementação ESP32
 ┣ 📂Implementação ESP32
 ┃ ┗ 📂Implementacao_ControlePID_MQTT
 ┃   ┗ 📜Implementacao_ControlePID_MQTT.ino
 ┣ 📂Simulação Scilab
 ┃ ┣ 📜Simulação Individual.zcos
 ┃ ┗ 📜Simulação(Curvas em um unico gráfico).zcos
```

---

## 🧠 Métodos de Sintonia Avaliados

O controle PID foi sintonizado utilizando três métodos distintos:

| Método               | Descrição                                                       |
|----------------------|------------------------------------------------------------------|
| Ziegler-Nichols      | Sintonia clássica baseada na curva S 			         |
| Cohen-Coon           | Sintonia baseada em modelo de 1ª ordem com tempo morto          |
| IMC (Internal Model Control) | Método baseado em robustez e rejeição de distúrbios               |

---

## 🧪 Arquivos de Simulação (Scilab/Xcos)

### `Simulação Individual.zcos`

Permite simular **separadamente** a resposta do sistema para cada método de sintonia PID. O usuário pode alternar entre os métodos no diagrama e observar a resposta individualmente.

### `Simulação(Curvas em um unico gráfico).zcos`

Gera **um único gráfico comparativo** com as três curvas de resposta (Ziegler-Nichols, Cohen-Coon e IMC), permitindo uma análise visual clara do desempenho de cada método.

---

## 💡 Implementação na ESP32

A ESP32 é responsável por:

- Realizar a leitura da velocidade do motor via encoder
- Aplicar o controle PID digital
- Enviar e receber dados via MQTT
- Controlar o motor com base nos comandos dos supervisórios

Arquivo principal:

📄 `Implementacao_ControlePID_MQTT.ino`

---

## 🔌 Requisitos

### Para Simulação:
- [Scilab 6.1.x](https://www.scilab.org/)
- Módulo Xcos instalado

### Para Implementação:
- ESP32 DevKit V1
- Motor CC com encoder óptico
- Ponte H (L298N)
- Broker MQTT (Mosquitto local)
- Conexão Wi-Fi

---

## 🌐 Supervisórios

- **Local (Node-RED):** Interface gráfica para controle, setpoint, visualização de velocidade e erro em tempo real.
- **Remoto (Web Firebase):** Interface online com login e controle do motor via dashboard acessível na nuvem.

---

## 📄 Licença

Este projeto tem fins exclusivamente acadêmicos. Uso e modificação permitidos para fins educacionais.

---

## 📬 Contato

Para dúvidas ou sugestões:

- 📧 carlos_henrique.sb@hotmail.com
- 📧 joao_paulo.sb@hotmail.com
