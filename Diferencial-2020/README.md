# Diferencial-2020

Repositório do projeto do firmware do diferencial 2020.


### Máquina de estados
O diferencial vai operar em uma máquina de estados composta da seguinte forma:
- Init_State: Estado inicial de operação, aguarda as condições de Ready to Drive estarem satisfeitas para mudar de estado (pedal de freio pressionado, botão ready to drive pressionado e sistema de tração acionado).
- Comm_State: Estado de comunicação, le as entradas do APPS, SWAS e sinal de freio. Calcula cada rotação e inicia a comunicação CAN.
- Error_State: Estado de erro, envia mensagem de erro via CAN em caso de detecção de alguma implausibilidade.
